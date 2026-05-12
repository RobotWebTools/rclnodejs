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

// Map dispatcher error codes to HTTP status codes. Codes not listed
// here fall back to 500.
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
    this._closed = false;
    // Placeholder frame id. The dispatcher reads `frame.id` from incoming
    // frames and echoes it on the reply; over HTTP we discard the reply id
    // and write the body directly, so the value just needs to be present.
    this._id = '__http__';

    // NOTE: we deliberately do *not* wire `req.on('close')` /
    // `res.on('close')` to `_emitCloseOnce()` here. Doing so would
    // catch client-side disconnects, but the close event also fires
    // on normal end-of-response, racing the still-in-flight rcl
    // reply callback and tearing down the dispatcher's lazy Client
    // mid-flight (rmw throws "client will not receive response" and
    // crashes the process). For now, dispatcher cleanup runs from
    // `send()` only — a long-running call that the client gave up
    // on holds onto its Publisher/Client until `send()` completes.
    // The leak is bounded (one entry per capability per request)
    // and acceptable until we have a way to drive cleanup that
    // doesn't race the rcl reply path.
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
   * are no-ops — HTTP is one-shot. Always emits `'close'` exactly once
   * after the reply is written so the dispatcher's per-connection
   * cleanup runs and lazy-created Publishers/Clients are released.
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
      return this._emitCloseOnce();
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
      return this._emitCloseOnce();
    }

    // Failure path.
    const code = frame.code || 'internal_error';
    const status = _STATUS_BY_CODE[code] || 500;
    this._writeError(status, code, frame.error || 'request failed');
    this._emitCloseOnce();
  }

  /** Force-close the response. The dispatcher's `cleanup` will follow. */
  // eslint-disable-next-line no-unused-vars
  close(code, reason) {
    if (!this._replied) {
      this._writeError(500, 'internal_error', 'connection closed');
    }
    this._emitCloseOnce();
  }

  /**
   * Idempotent `'close'` emitter — guarantees the dispatcher's
   * cleanup runs exactly once even if `send()` and `close()` race.
   */
  _emitCloseOnce() {
    if (this._closed) return;
    this._closed = true;
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
 * HTTP adapter for the Web Runtime.
 *
 * Exposes `call` and `publish` capabilities over plain HTTP:
 *
 *     POST /capability/call/<name>
 *     POST /capability/publish/<name>
 *
 * Subscriptions stay on the WebSocket transport (one HTTP connection
 * per inbound message would burn the browser's per-origin budget).
 * Anything that isn't `POST /capability/call/<name>` or
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
    this.basePath = _normaliseBasePath(options.basePath);
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
    return new Promise((resolve) => {
      // Force-close keep-alive sockets so server.close() doesn't hang
      // waiting for clients (notably Node's fetch/undici, which keeps
      // sockets warm by default). Available since Node 18.2.
      if (typeof server.closeAllConnections === 'function') {
        server.closeAllConnections();
      }
      server.close(() => resolve());
    });
  }

  // ---------- internals ----------

  _route(req, res) {
    if (this.verifyRequest) {
      let allowed;
      try {
        allowed = this.verifyRequest(req);
      } catch (e) {
        debug('verifyRequest threw: %s', (e && e.stack) || e);
        return _writeJson(res, 500, {
          ok: false,
          error: 'verifyRequest hook failed',
          code: 'internal_error',
        });
      }
      if (allowed === false) {
        return _writeJson(res, 401, {
          ok: false,
          error: 'unauthorized',
          code: 'unauthorized',
        });
      }
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

    const tail = pathname.slice(this.basePath.length + 1); // strip basePath + '/'
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
    let name;
    try {
      name = '/' + decodeURIComponent(tail.slice(slash + 1));
    } catch {
      return _writeJson(res, 400, {
        ok: false,
        error: `invalid percent-encoding in capability name: ${tail.slice(slash + 1)}`,
        code: 'invalid_url',
      });
    }

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
  // Guard against double-callback: the body may exceed maxBytes (we call
  // cb(err) and req.destroy()), and the destroy itself can synchronously
  // emit 'error' which would otherwise reach cb(err) a second time.
  let done = false;
  const finish = (err, value) => {
    if (done) return;
    done = true;
    cb(err, value);
  };

  const ctype = (req.headers['content-type'] || '').toLowerCase();
  // Only the media-type segment matters; ignore parameters like
  // `; charset=utf-8` and reject sneaky values such as
  // `text/plain;application/json` that include the right substring
  // but mean something else.
  const mediaType = ctype.split(';')[0].trim();
  if (
    req.method === 'POST' &&
    ctype !== '' &&
    mediaType !== 'application/json'
  ) {
    // Be lenient: empty bodies + missing content-type are treated as
    // "publish with no fields", same as `{}`. Only reject explicit
    // non-JSON content types.
    const e = new Error(
      `unsupported content-type: ${ctype} (expected application/json)`
    );
    e.code = 'invalid_content_type';
    return finish(e);
  }
  let total = 0;
  const chunks = [];
  req.on('data', (chunk) => {
    if (done) return;
    total += chunk.length;
    if (total > maxBytes) {
      const e = new Error(`request body exceeds ${maxBytes} bytes`);
      e.code = 'payload_too_large';
      finish(e);
      req.destroy();
      return;
    }
    chunks.push(chunk);
  });
  req.on('end', () => {
    if (done) return;
    const raw = Buffer.concat(chunks).toString('utf8');
    if (!raw) return finish(null, {});
    try {
      finish(null, JSON.parse(raw));
    } catch (e) {
      const err = new Error(`invalid JSON: ${e.message}`);
      err.code = 'invalid_json';
      finish(err);
    }
  });
  req.on('error', (e) => {
    e.code = e.code || 'request_error';
    finish(e);
  });
}

function _normaliseBasePath(value) {
  if (value === undefined || value === null || value === '') {
    return '/capability';
  }
  if (typeof value !== 'string') {
    throw new TypeError(
      `HttpTransport: basePath must be a string, got ${typeof value}`
    );
  }
  // Ensure single leading slash, no trailing slash.
  let p = value.replace(/\/+$/, '');
  if (!p.startsWith('/')) p = '/' + p;
  if (p === '') {
    throw new TypeError('HttpTransport: basePath must not be empty');
  }
  return p;
}

module.exports = { HttpTransport };
