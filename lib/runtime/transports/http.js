// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

const http = require('node:http');
const debug = require('debug')('rclnodejs:runtime:http');
const { Connection } = require('../connection.js');
const { TransportAdapter } = require('../transport_adapter.js');

// Map dispatcher error codes to HTTP status codes. Anything not listed
// here falls back to 500 — see the README for the full table.
const _STATUS_BY_CODE = Object.freeze({
  not_exposed: 404,
  not_implemented: 501,
  unknown_kind: 400,
  invalid_frame: 400,
  invalid_json: 400,
  binary_unsupported: 400,
  missing_id: 400,
  duplicate_id: 400,
  unknown_sub_id: 400,
  missing_sub_id: 400,
  internal_error: 500,
  call_failed: 500,
  publish_failed: 500,
  schema_violation: 400,
});

const _MAX_BODY_BYTES = 1 * 1024 * 1024; // 1 MiB cap on request bodies

/**
 * A request-scoped {@link Connection} used by the HTTP transport.
 *
 * Wraps a single inbound HTTP request/response pair: emits exactly one
 * `'message'` frame to the dispatcher, captures the dispatcher's single
 * reply via `send()`, and translates that reply back into HTTP semantics
 * (status code, JSON body, or 204 for void replies).
 *
 * The dispatcher remains transport-agnostic — it sees the same
 * `{kind, capability, payload, id}` envelope as on WebSocket. The only
 * thing this class does is collapse the long-lived `Connection`
 * abstraction into one round-trip.
 */
class HttpRequestConnection extends Connection {
  /**
   * @param {import('http').IncomingMessage} req
   * @param {import('http').ServerResponse} res
   * @param {'call'|'publish'} kind
   * @param {string} capability
   * @param {*} payload
   */
  constructor(req, res, kind, capability, payload) {
    super();
    this.req = req;
    this.res = res;
    this._kind = kind;
    this._capability = capability;
    this._payload = payload;
    this._replied = false;
    // Synthesised frame id so the dispatcher can correlate even though
    // the HTTP path doesn't carry one on the wire.
    this._id = '__http__';
  }

  /**
   * Kick off dispatch. Must be called immediately after the dispatcher
   * is wired up to this connection. Emits the synthesised frame.
   */
  begin() {
    this.emit('message', {
      id: this._id,
      kind: this._kind,
      capability: this._capability,
      payload: this._payload,
    });
  }

  /**
   * Receive a reply from the dispatcher. Translates `{ok, payload,
   * error, code}` into the appropriate HTTP response. Subsequent calls
   * are no-ops — HTTP is one-shot.
   */
  send(frame) {
    if (this._replied) return;
    this._replied = true;

    if (frame.event === 'message') {
      // Subscribe deliveries are not valid over HTTP; the dispatcher
      // shouldn't send them for kind=call/publish, but be defensive.
      this._writeError(
        400,
        'streaming_unsupported',
        'streaming events are not supported over HTTP'
      );
      return;
    }

    if (frame.ok === true) {
      if (this._kind === 'publish') {
        this.res.writeHead(204).end();
      } else {
        // call: serialise the payload (may be undefined for void replies).
        const body = JSON.stringify(frame.payload ?? null);
        this.res.writeHead(200, {
          'content-type': 'application/json; charset=utf-8',
          'content-length': Buffer.byteLength(body),
        });
        this.res.end(body);
      }
      return;
    }

    // Failure path.
    const code = frame.code || 'internal_error';
    const status = _STATUS_BY_CODE[code] || 500;
    this._writeError(status, code, frame.error || 'request failed');
  }

  /** Force-close the response. The dispatcher's `cleanup` will follow. */
  // eslint-disable-next-line no-unused-vars
  close(code, reason) {
    if (!this._replied) {
      this._writeError(500, 'internal_error', 'connection closed');
    }
    this.emit('close');
  }

  _writeError(status, code, message) {
    const body = JSON.stringify({ ok: false, error: message, code });
    try {
      this.res.writeHead(status, {
        'content-type': 'application/json; charset=utf-8',
        'content-length': Buffer.byteLength(body),
      });
      this.res.end(body);
    } catch (e) {
      debug('writeError failed: %s', e.message);
    }
  }
}

/**
 * Layer-2 HTTP adapter for the Web Runtime.
 *
 * Exposes `call` and `publish` capabilities over plain HTTP:
 *
 *     POST /capability/call/<name>
 *     POST /capability/publish/<name>
 *
 * Subscriptions stay on the WebSocket transport (see WEB_RUNTIME_ROADMAP.md
 * "Why no SSE?"). Anything that isn't `POST /capability/call/<name>` or
 * `POST /capability/publish/<name>` returns a 404 / 405 — the adapter
 * does not serve unrelated routes.
 *
 * @example
 *   const runtime = createRuntime({
 *     node,
 *     transports: [
 *       new WebSocketTransport({ port: 9000 }),
 *       new HttpTransport({ port: 9001 }),
 *     ],
 *   });
 */
class HttpTransport extends TransportAdapter {
  /**
   * @param {object} [options]
   * @param {number} [options.port=9001]
   * @param {string} [options.host='::']
   * @param {string} [options.basePath='/capability']
   * @param {(req: import('http').IncomingMessage) => boolean} [options.verifyRequest]
   *   Optional auth hook called with the raw request. Return `false` to
   *   reject the request with 401. Mirrors `WebSocketTransport.verifyClient`.
   */
  constructor(options = {}) {
    super();
    this.port = options.port != null ? options.port : 9001;
    this.host = options.host != null ? options.host : '::';
    this.basePath = options.basePath || '/capability';
    this.verifyRequest = options.verifyRequest;
    this._server = null;
    this._onConnection = null;
  }

  async start({ onConnection }) {
    if (typeof onConnection !== 'function') {
      throw new TypeError(
        'HttpTransport.start: onConnection must be a function'
      );
    }
    this._onConnection = onConnection;
    return new Promise((resolve, reject) => {
      const server = http.createServer((req, res) => this._route(req, res));
      this._server = server;
      server.on('error', reject);
      server.listen(this.port, this.host, () => {
        const addr = server.address();
        this.port = typeof addr === 'object' && addr ? addr.port : this.port;
        debug(
          'HTTP capability runtime listening on %s:%d%s',
          (addr && addr.address) || this.host,
          this.port,
          this.basePath
        );
        resolve(addr);
      });
    });
  }

  async stop() {
    if (!this._server) return;
    const server = this._server;
    this._server = null;
    return new Promise((resolve) => server.close(() => resolve()));
  }

  // ---------- internals ----------

  _route(req, res) {
    if (this.verifyRequest && this.verifyRequest(req) === false) {
      return _writeJson(res, 401, {
        ok: false,
        error: 'unauthorized',
        code: 'unauthorized',
      });
    }

    let pathname;
    try {
      pathname = new URL(req.url || '/', 'http://localhost').pathname;
    } catch {
      return _writeJson(res, 400, {
        ok: false,
        error: 'invalid request URL',
        code: 'invalid_url',
      });
    }

    if (!pathname.startsWith(this.basePath + '/')) {
      return _writeJson(res, 404, {
        ok: false,
        error: `not a capability route: ${pathname}`,
        code: 'not_found',
      });
    }

    const tail = pathname.slice(this.basePath.length + 1); // strip "/capability/"
    // tail = "<kind>/<rest...>"; first segment is the kind, rest is the
    // ROS name (which itself can contain slashes).
    const slash = tail.indexOf('/');
    if (slash <= 0) {
      return _writeJson(res, 404, {
        ok: false,
        error: `expected ${this.basePath}/<kind>/<name>`,
        code: 'not_found',
      });
    }
    const kind = tail.slice(0, slash);
    const name = '/' + decodeURIComponent(tail.slice(slash + 1));

    if (kind !== 'call' && kind !== 'publish') {
      return _writeJson(res, 404, {
        ok: false,
        error: `unsupported kind over HTTP: ${kind} (use WebSocket for subscribe/action)`,
        code: 'unsupported_kind',
      });
    }

    if (req.method !== 'POST') {
      res.setHeader('allow', 'POST');
      return _writeJson(res, 405, {
        ok: false,
        error: `method not allowed: ${req.method}`,
        code: 'method_not_allowed',
      });
    }

    _readJsonBody(req, _MAX_BODY_BYTES, (err, payload) => {
      if (err) {
        const code = err.code || 'invalid_json';
        const status = code === 'payload_too_large' ? 413 : 400;
        return _writeJson(res, status, { ok: false, error: err.message, code });
      }
      const conn = new HttpRequestConnection(req, res, kind, name, payload);
      try {
        this._onConnection(conn);
        conn.begin();
      } catch (e) {
        debug('onConnection threw: %s', e.stack || e.message);
        conn.close();
      }
    });
  }
}

function _writeJson(res, status, body) {
  const json = JSON.stringify(body);
  res.writeHead(status, {
    'content-type': 'application/json; charset=utf-8',
    'content-length': Buffer.byteLength(json),
  });
  res.end(json);
}

function _readJsonBody(req, maxBytes, cb) {
  const ctype = (req.headers['content-type'] || '').toLowerCase();
  if (
    req.method === 'POST' &&
    !ctype.includes('application/json') &&
    !ctype.startsWith('application/json')
  ) {
    // Be lenient: empty bodies + missing content-type are treated as
    // "publish with no fields", same as `{}`. Only reject explicit
    // non-JSON content types.
    if (ctype && ctype !== '') {
      const e = new Error(
        `unsupported content-type: ${ctype} (expected application/json)`
      );
      e.code = 'invalid_content_type';
      return cb(e);
    }
  }
  let total = 0;
  const chunks = [];
  req.on('data', (chunk) => {
    total += chunk.length;
    if (total > maxBytes) {
      req.destroy();
      const e = new Error(`request body exceeds ${maxBytes} bytes`);
      e.code = 'payload_too_large';
      return cb(e);
    }
    chunks.push(chunk);
  });
  req.on('end', () => {
    const raw = Buffer.concat(chunks).toString('utf8');
    if (!raw) return cb(null, {});
    try {
      cb(null, JSON.parse(raw));
    } catch (e) {
      const err = new Error(`invalid JSON: ${e.message}`);
      err.code = 'invalid_json';
      cb(err);
    }
  });
  req.on('error', (e) => {
    e.code = e.code || 'request_error';
    cb(e);
  });
}

module.exports = { HttpTransport, HttpRequestConnection };
