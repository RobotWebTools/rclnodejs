// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Pure-JavaScript browser SDK for the rclnodejs Web Runtime.
//
// Authored as **ESM** so the browser's native module loader (and any
// modern bundler) can consume it directly without a CJS-to-ESM
// transform. Ships with a `type: module` web/package.json so Node
// also treats this file as ESM.
//
// In a real browser, `WebSocket` and `fetch` are globals. In Node —
// used by the integration tests and SSR scenarios — we fall back to
// the optional `ws` package via dynamic import (Node ≥ 18 has fetch
// natively, no extra dep needed).
//
// This module never imports rclnodejs itself — zero native deps,
// safe to bundle for the browser.
//
// Two transports are supported, picked from the URL scheme:
//
//   - ws:// / wss://       → WebSocket only (call/publish/subscribe).
//   - http:// / https://   → HTTP for call/publish; subscribe lazily
//                            falls through to a sibling WebSocket.
//   - { http, ws }         → explicit endpoint pair.

let WS = globalThis.WebSocket;
if (!WS) {
  try {
    const wsModule = await import('ws');
    WS = wsModule.WebSocket || wsModule.default;
  } catch {
    // No WebSocket implementation available in this environment.
  }
}

// Frame ids are UUIDs so they never collide across sessions or modules
// that share a single RosClient. Use the platform RNG when available
// (browsers and Node ≥ 16.7) and fall back to a small RFC-4122 v4
// implementation otherwise.
const _genId =
  globalThis.crypto && typeof globalThis.crypto.randomUUID === 'function'
    ? () => globalThis.crypto.randomUUID()
    : () => {
        return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
          const r = (Math.random() * 16) | 0;
          const v = c === 'x' ? r : (r & 0x3) | 0x8;
          return v.toString(16);
        });
      };

// -------------------------------------------------------------------
// Internal "link" — the WebSocket transport. Hidden from the public API.
// -------------------------------------------------------------------

/**
 * WebSocket link. Speaks the capability frame protocol used by
 * `WebSocketTransport` on the server. Long-lived; supports `call`,
 * `publish`, `subscribe`, `unsubscribe` (and reserved `action`).
 */
class _WsLink {
  constructor(url) {
    this.url = url;
    this._ws = null;
    this._pending = new Map();
    this._subs = new Map();
    this._closed = false;
  }

  connect() {
    return new Promise((resolve, reject) => {
      if (!WS) {
        return reject(
          new Error(
            'no WebSocket implementation available; provide a global WebSocket or install the optional `ws` package'
          )
        );
      }
      const ws = new WS(this.url);
      this._ws = ws;
      const onOpen = () => {
        ws.removeEventListener
          ? ws.removeEventListener('error', onError)
          : ws.off && ws.off('error', onError);
        resolve();
      };
      const onError = (err) => {
        if (this._pending.size === 0 && !this._closed) {
          reject(err && err.error ? err.error : err);
        } else {
          this._failAll(err);
        }
      };
      const onClose = () => {
        this._closed = true;
        this._failAll(new Error('connection closed'));
        this._subs.clear();
      };
      const onMessage = (ev) => this._onMessage(ev);

      if (ws.addEventListener) {
        ws.addEventListener('open', onOpen, { once: true });
        ws.addEventListener('error', onError);
        ws.addEventListener('close', onClose);
        ws.addEventListener('message', onMessage);
      } else {
        ws.once('open', onOpen);
        ws.on('error', onError);
        ws.on('close', onClose);
        ws.on('message', (data) => onMessage({ data }));
      }
    });
  }

  call(capability, payload) {
    return this._request({ kind: 'call', capability, payload });
  }
  publish(capability, payload) {
    return this._request({ kind: 'publish', capability, payload });
  }
  subscribe(capability, callback) {
    const id = _genId();
    return new Promise((resolve, reject) => {
      this._pending.set(id, {
        resolve: () => {
          this._subs.set(id, { capability, callback });
          resolve({
            subId: id,
            close: () => this._unsubscribe(id),
          });
        },
        reject,
      });
      this._sendRaw({ id, kind: 'subscribe', capability });
    });
  }
  _unsubscribe(subId) {
    this._subs.delete(subId);
    return this._request({ kind: 'unsubscribe', subId });
  }

  async close() {
    if (!this._ws || this._closed) return;
    return new Promise((resolve) => {
      const ws = this._ws;
      const onClose = () => {
        if (ws.removeEventListener) ws.removeEventListener('close', onClose);
        else ws.off && ws.off('close', onClose);
        resolve();
      };
      if (ws.addEventListener)
        ws.addEventListener('close', onClose, { once: true });
      else ws.once('close', onClose);
      try {
        ws.close();
      } catch (_) {
        resolve();
      }
    });
  }

  _request(frame) {
    return new Promise((resolve, reject) => {
      if (this._closed) return reject(new Error('connection closed'));
      const id = _genId();
      frame.id = id;
      this._pending.set(id, { resolve, reject });
      this._sendRaw(frame);
    });
  }

  _sendRaw(frame) {
    try {
      this._ws.send(JSON.stringify(frame));
    } catch (e) {
      const pend = this._pending.get(frame.id);
      if (pend) {
        this._pending.delete(frame.id);
        pend.reject(e);
      }
    }
  }

  _onMessage(ev) {
    let frame;
    try {
      const data =
        typeof ev.data === 'string'
          ? ev.data
          : ev.data && ev.data.toString
            ? ev.data.toString('utf8')
            : String(ev.data);
      frame = JSON.parse(data);
    } catch (_) {
      return;
    }
    if (frame.event === 'message') {
      const sub = this._subs.get(frame.subId);
      if (sub) {
        try {
          sub.callback(frame.payload);
        } catch (_) {
          // user callback errors don't break the dispatch loop
        }
      }
      return;
    }
    const pend = this._pending.get(frame.id);
    if (!pend) return;
    this._pending.delete(frame.id);
    if (frame.ok) pend.resolve(frame.payload);
    else
      pend.reject(
        Object.assign(new Error(frame.error || 'request failed'), {
          code: frame.code,
        })
      );
  }

  _failAll(err) {
    const cause = err instanceof Error ? err : new Error(String(err));
    for (const p of this._pending.values()) {
      try {
        p.reject(cause);
      } catch (_) {}
    }
    this._pending.clear();
  }
}

/**
 * HTTP link. Speaks the L2 HTTP capability protocol used by
 * `HttpTransport` on the server. Stateless — every `call`/`publish`
 * is a one-shot `fetch()`. Does not support subscribe; the public
 * client falls through to the WebSocket link for streaming verbs.
 */
class _HttpLink {
  constructor(baseUrl) {
    // Normalise: strip trailing slash so we can append `/capability/<kind>/<name>`
    this.baseUrl = baseUrl.replace(/\/+$/, '');
  }

  async connect() {
    // Stateless — nothing to open. We don't probe the server here so
    // that connect() stays cheap; the first call() will surface any
    // wrong-URL or wrong-port errors instead.
  }

  async close() {
    // No-op for HTTP.
  }

  call(capability, payload) {
    return this._fetch('call', capability, payload, /* expectBody */ true);
  }

  publish(capability, payload) {
    return this._fetch('publish', capability, payload, /* expectBody */ false);
  }

  async _fetch(kind, capability, payload, expectBody) {
    const url =
      this.baseUrl + '/capability/' + kind + '/' + _encodeRosName(capability);
    let res;
    try {
      res = await fetch(url, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify(payload ?? {}),
      });
    } catch (e) {
      throw Object.assign(new Error(`HTTP request failed: ${e.message}`), {
        code: 'network_error',
      });
    }

    if (res.status === 204) return undefined;

    const contentType = res.headers.get('content-type') || '';
    let body;
    if (contentType.includes('application/json')) {
      try {
        body = await res.json();
      } catch (e) {
        throw Object.assign(
          new Error(`invalid JSON in HTTP response: ${e.message}`),
          { code: 'invalid_response' }
        );
      }
    } else if (res.ok) {
      body = await res.text();
    } else {
      body = { ok: false, error: await res.text(), code: 'http_' + res.status };
    }

    if (!res.ok) {
      const err = body && typeof body === 'object' ? body : {};
      throw Object.assign(new Error(err.error || `HTTP ${res.status}`), {
        code: err.code || 'http_' + res.status,
        status: res.status,
      });
    }
    return expectBody ? body : undefined;
  }
}

// ROS names always start with `/`. Encode each path segment so that
// names with `~`, `:`, etc. survive routing while keeping the leading
// slash stripped (the server adds it back).
function _encodeRosName(name) {
  return name.replace(/^\/+/, '').split('/').map(encodeURIComponent).join('/');
}

// -------------------------------------------------------------------
// Public surface
// -------------------------------------------------------------------

/**
 * Thin client for the rclnodejs Web Runtime capability protocol.
 *
 * Picks a transport from the URL scheme:
 *
 *   - `ws://`, `wss://`      → WebSocket only (call/publish/subscribe).
 *   - `http://`, `https://`  → HTTP for `call`/`publish`; `subscribe`
 *     lazily falls through to a sibling WebSocket endpoint at the
 *     same host with `/capability` appended.
 *   - object `{http, ws}`    → both URLs spelled out explicitly.
 *
 * **Path conventions.** The single-URL forms assume the server uses
 * the default `/capability` layout for both transports. If you change
 * `--path` / `--http-base-path` on the server (or sit it behind a
 * path-rewriting proxy), pass the full URLs via `{http, ws}` so the
 * SDK does not have to guess.
 *
 * @example
 *   import { connect } from 'rclnodejs/web';
 *
 *   // WebSocket-only
 *   const ros = await connect('ws://robot.local:9000/capability');
 *
 *   // HTTP for call/publish, automatic WS sibling for subscribe
 *   const ros = await connect('http://robot.local:9001');
 *
 *   // Split endpoints (e.g. WS behind a different proxy)
 *   const ros = await connect({
 *     http: 'https://robot.example/api',
 *     ws:   'wss://robot.example/capability',
 *   });
 */
export class RosClient {
  /**
   * @param {string|{http?:string, ws?:string}} url
   * @param {object} [options]
   * @param {boolean} [options.reconnect=false] Reserved; not yet implemented.
   */
  constructor(url, options = {}) {
    this.options = options;
    if (options.reconnect) {
      // eslint-disable-next-line no-console
      console.warn(
        'rclnodejs/web: reconnect is not yet implemented; ignoring option'
      );
    }
    const { httpUrl, wsUrl, wsExplicit } = _resolveUrls(url);
    this.url = httpUrl || wsUrl;
    this._http = httpUrl ? new _HttpLink(httpUrl) : null;
    this._wsUrl = wsUrl;
    this._ws = null; // built lazily by _ensureWs() unless eager
    // Eagerly open WS only when the user *explicitly* asked for it
    // (passed `ws://...` or `{ws}`). When we *derived* a sibling WS
    // URL from an HTTP base, it's lazy — most HTTP-only callers
    // never subscribe.
    this._wsEager = !!wsExplicit;
    this._wsConnect = null; // memoised connect promise
  }

  /** Open the link(s). */
  async connect() {
    // Open HTTP eagerly (it's a no-op anyway). Defer the WebSocket
    // open until the user actually calls subscribe() — that way an
    // HTTP-only deployment with no WS sibling works for call/publish
    // without blowing up here.
    if (this._http) await this._http.connect();
    if (this._wsEager) await this._ensureWs();
    return this;
  }

  /** Close the underlying link(s). */
  async close() {
    const tasks = [];
    if (this._ws) tasks.push(this._ws.close());
    if (this._http) tasks.push(this._http.close());
    await Promise.all(tasks);
  }

  /**
   * Build (and cache) the WebSocket link on first use. Throws a
   * structured error if no WS URL is available or the open fails.
   */
  async _ensureWs() {
    if (!this._wsUrl) {
      throw Object.assign(
        new Error(
          'subscribe requires a WebSocket endpoint; connect() was given an HTTP-only URL with no WS sibling'
        ),
        { code: 'transport_unavailable' }
      );
    }
    if (this._ws) return this._ws;
    if (!this._wsConnect) {
      const link = new _WsLink(this._wsUrl);
      this._wsConnect = link.connect().then(
        () => {
          this._ws = link;
          return link;
        },
        (err) => {
          this._wsConnect = null; // allow a retry on next subscribe()
          throw Object.assign(
            new Error(
              `failed to open WebSocket sibling at ${this._wsUrl}: ${err && err.message ? err.message : String(err)}`
            ),
            { code: 'transport_unavailable', cause: err }
          );
        }
      );
    }
    return this._wsConnect;
  }

  // -- verb API --

  call(capability, payload) {
    return (this._http || this._ws).call(capability, payload);
  }

  publish(capability, payload) {
    return (this._http || this._ws).publish(capability, payload);
  }

  async subscribe(capability, callback) {
    if (typeof callback !== 'function') {
      throw new TypeError('subscribe(capability, callback): callback required');
    }
    const ws = await this._ensureWs();
    return ws.subscribe(capability, callback);
  }
}

/**
 * Resolve the user-supplied `connect()` URL into an `{httpUrl, wsUrl}`
 * pair. Either may be absent — then the corresponding link is not
 * constructed and the client returns `transport_unavailable` for any
 * verb that needs it.
 */
function _resolveUrls(url) {
  if (url && typeof url === 'object' && !Array.isArray(url)) {
    return {
      httpUrl: url.http || null,
      wsUrl: url.ws || null,
      wsExplicit: !!url.ws,
    };
  }
  if (typeof url !== 'string' || !url) {
    throw new TypeError(
      'connect(url): url must be a non-empty string or {http, ws}'
    );
  }
  if (/^wss?:\/\//i.test(url)) {
    return { httpUrl: null, wsUrl: url, wsExplicit: true };
  }
  if (/^https?:\/\//i.test(url)) {
    // HTTP base URL: derive a sibling WS URL by swapping the scheme
    // and pointing at /capability. Lazy — we don't open it until the
    // user actually calls subscribe().
    let wsUrl;
    try {
      const u = new URL(url);
      u.protocol = u.protocol === 'https:' ? 'wss:' : 'ws:';
      u.pathname = (u.pathname.replace(/\/+$/, '') || '') + '/capability';
      wsUrl = u.toString();
    } catch (_) {
      wsUrl = null;
    }
    return { httpUrl: url, wsUrl, wsExplicit: false };
  }
  throw new TypeError(
    `connect(url): unrecognised URL scheme: ${url} (expected ws://, wss://, http://, or https://)`
  );
}

/**
 * Open a connection to a Web Runtime capability endpoint.
 *
 * See {@link RosClient} for the URL-scheme → transport mapping and
 * the path conventions assumed for the default `rclnodejs-web` server
 * layout (`/capability` on both transports). For non-default
 * `basePath` / `path` configurations, pass `{http, ws}` explicitly.
 *
 * @param {string|{http?:string, ws?:string}} url
 * @param {object} [options]
 * @returns {Promise<RosClient>}
 */
export async function connect(url, options) {
  const c = new RosClient(url, options);
  await c.connect();
  return c;
}
