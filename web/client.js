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
let _wsResolved = !!WS;

// Resolve a Node WebSocket implementation lazily. This avoids a top-level
// `await import('ws')`, which is unavailable in the CommonJS build. In browsers
// `globalThis.WebSocket` is used and the `ws` package is never touched.
async function _ensureWS() {
  if (_wsResolved) {
    return WS;
  }
  _wsResolved = true;
  try {
    const wsModule = await import('ws');
    WS = wsModule.WebSocket || wsModule.default;
  } catch {
    // No WebSocket implementation available in this environment.
  }
  return WS;
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

// Exponential backoff with jitter (half fixed, half random) shared by WS reconnect and HTTP retry.
function _backoffDelay(attempt, { baseMs = 500, maxMs = 30000 } = {}) {
  const capped = Math.min(baseMs * 2 ** attempt, maxMs);
  return capped / 2 + Math.random() * (capped / 2);
}

function _sleep(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

// -------------------------------------------------------------------
// Internal "link" — the WebSocket transport. Hidden from the public API.
// -------------------------------------------------------------------

/**
 * WebSocket link. Speaks the capability frame protocol used by
 * `WebSocketTransport` on the server. Long-lived; supports `call`,
 * `publish`, `subscribe`, `unsubscribe` (and reserved `action`).
 *
 * With `options.reconnect`, a drop after a successful open reopens with
 * backoff and replays subscriptions under the same `subId`. The *first*
 * connection attempt is unaffected — a bad URL still rejects once.
 */
class _WsLink {
  constructor(url, options = {}) {
    this.url = url;
    this._reconnect = !!options.reconnect;
    this._onEvent = options.onEvent || (() => {});
    this._ws = null;
    this._pending = new Map();
    this._subs = new Map();
    this._closed = false;
    this._userClosed = false;
    this._reconnecting = false;
    this._reconnectAttempt = 0;
    this._reconnectTimer = null;
  }

  /** First-time open. Rejects once if the initial connection fails. */
  async connect() {
    return this._open();
  }

  /** Open (or reopen, on reconnect) the underlying WebSocket. */
  async _open() {
    await _ensureWS();
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
      // `settled`: this open attempt's promise has resolved/rejected.
      // `opened`: onOpen actually fired for this attempt (distinct from
      // `settled`, which onError can also set on a first-attempt failure —
      // without this, a subsequent 'close' would wrongly reach _handleClose()).
      let settled = false;
      let opened = false;
      const onOpen = () => {
        settled = true;
        opened = true;
        this._reconnectAttempt = 0;
        this._reconnecting = false;
        this._resubscribeAll();
        resolve();
      };
      const onError = (err) => {
        // Ignore post-open: 'close' always follows and _handleClose() owns failing pending requests.
        if (!settled) {
          settled = true;
          reject(err && err.error ? err.error : err);
        }
      };
      const onClose = () => {
        if (opened) {
          this._handleClose();
          return;
        }
        if (!settled) {
          settled = true;
          reject(new Error('connection closed before it was established'));
        }
      };
      const onMessage = (ev) => this._onMessage(ev);

      if (ws.addEventListener) {
        ws.addEventListener('open', onOpen, { once: true });
        ws.addEventListener('error', onError);
        ws.addEventListener('close', onClose, { once: true });
        ws.addEventListener('message', onMessage);
      } else {
        ws.once('open', onOpen);
        ws.on('error', onError);
        ws.once('close', onClose);
        ws.on('message', (data) => onMessage({ data }));
      }
    });
  }

  /** Handle an unexpected close: fail in-flight requests, then finalize or reconnect. */
  _handleClose() {
    if (this._userClosed) {
      this._failAll(new Error('connection closed'));
      this._finalizeClosed();
      return;
    }
    this._onEvent('disconnected', undefined);
    if (!this._reconnect) {
      this._failAll(_connectionLostError(false));
      this._finalizeClosed();
      return;
    }
    this._failAll(_connectionLostError(true));
    this._reconnecting = true;
    this._scheduleReconnect();
  }

  /** Shared terminal state for a deliberate close or a non-reconnecting drop. */
  _finalizeClosed() {
    this._closed = true;
    this._reconnecting = false;
    this._subs.clear();
  }

  _scheduleReconnect() {
    const attempt = this._reconnectAttempt++;
    const delay = _backoffDelay(attempt);
    this._onEvent('reconnecting', { attempt: attempt + 1, delay });
    this._reconnectTimer = setTimeout(() => {
      this._reconnectTimer = null;
      this._open().then(
        () => this._onEvent('reconnected', undefined),
        // A close() during this in-flight attempt aborts it and lands here
        // too \u2014 don't keep retrying past a deliberate close.
        () => {
          if (!this._userClosed) this._scheduleReconnect();
        }
      );
    }, delay);
  }

  /**
   * Replay every active subscription after a reopen, reusing each `subId`.
   * Known gap: a `not_exposed` ack for a since-removed capability has no
   * pending entry to reject, so it's silently dropped.
   */
  _resubscribeAll() {
    for (const [subId, { capability }] of this._subs) {
      this._sendRaw({ id: subId, kind: 'subscribe', capability });
    }
  }

  call(capability, payload) {
    return this._request({ kind: 'call', capability, payload });
  }
  publish(capability, payload) {
    return this._request({ kind: 'publish', capability, payload });
  }
  subscribe(capability, callback) {
    if (this._reconnecting) {
      return Promise.reject(_connectionLostError());
    }
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
    this._userClosed = true;
    if (this._reconnectTimer) {
      clearTimeout(this._reconnectTimer);
      this._reconnectTimer = null;
    }
    if (!this._ws || this._closed) {
      this._finalizeClosed();
      return;
    }
    const ws = this._ws;
    // Already closed (e.g. mid-backoff) means 'close' won't fire again, so
    // finalize directly instead of leaving state as if still reconnecting.
    if (ws.readyState === 3) {
      this._finalizeClosed();
      return;
    }
    return new Promise((resolve) => {
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
      if (this._reconnecting) return reject(_connectionLostError());
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
      p.reject(cause);
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
  constructor(baseUrl, options = {}) {
    // Normalise: strip trailing slash so we can append the path. If
    // the user URL already ends with `/capability` (e.g. they spelled
    // it out explicitly, mirroring the WS form), don't double-prefix
    // it on every fetch. The default-runtime layout still works for
    // the bare-host form `'http://host:9001'` — we just append the
    // default path ourselves below.
    const trimmed = baseUrl.replace(/\/+$/, '');
    this.baseUrl = trimmed.endsWith('/capability')
      ? trimmed
      : trimmed + '/capability';
    this._retries = Math.max(0, options.retries || 0);
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

  /** Retries only network errors and 5xx; a 4xx won't succeed on retry. */
  async _fetch(kind, capability, payload, expectBody) {
    for (let attempt = 0; ; attempt++) {
      try {
        return await this._fetchOnce(kind, capability, payload, expectBody);
      } catch (e) {
        if (attempt >= this._retries || !_isRetryableHttpError(e)) throw e;
        await _sleep(_backoffDelay(attempt));
      }
    }
  }

  async _fetchOnce(kind, capability, payload, expectBody) {
    const url = this.baseUrl + '/' + kind + '/' + _encodeRosName(capability);
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

function _isRetryableHttpError(err) {
  return !!err && (err.code === 'network_error' || (err.status ?? 0) >= 500);
}

function _connectionLostError(reconnecting = true) {
  return Object.assign(
    new Error(
      reconnecting ? 'connection lost; reconnecting' : 'connection lost'
    ),
    { code: 'connection_lost' }
  );
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
 * **Path conventions.** When a `ws://` / `wss://` URL is passed
 * without a path (or with just `/`), the SDK appends the runtime's
 * default `/capability` path automatically — `'ws://host:9000'` and
 * `'ws://host:9000/capability'` therefore behave identically. Pass
 * an explicit non-default path if your server changed `--path` /
 * `--http-base-path` or sits behind a path-rewriting proxy.
 *
 * @example
 *   import { connect } from 'rclnodejs/web';
 *
 *   // WebSocket-only (path defaults to /capability)
 *   const ros = await connect('ws://robot.local:9000');
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
   * @param {boolean} [options.reconnect=false] Reopen the WS link with
   *   backoff after a drop, replaying subscriptions. Enables 'reconnecting' /
   *   'reconnected' \u2014 see {@link RosClient#on}; 'disconnected' fires on any
   *   unexpected drop regardless. The first connect attempt still rejects
   *   once rather than retrying forever.
   * @param {number} [options.httpRetries=0] Retry a failed HTTP
   *   call()/publish() this many times (network errors and 5xx only).
   */
  constructor(url, options = {}) {
    this.options = options;
    this._listeners = new Map(); // event name -> Set<handler>
    const { httpUrl, wsUrl, wsExplicit } = _resolveUrls(url);
    this.url = httpUrl || wsUrl;
    // Shared by both _WsLink construction sites below (eager and lazy).
    this._wsOptions = {
      reconnect: !!options.reconnect,
      onEvent: (name, detail) => this._emit(name, detail),
    };
    this._http = httpUrl
      ? new _HttpLink(httpUrl, { retries: options.httpRetries })
      : null;
    this._wsUrl = wsUrl;
    // Eagerly construct (but don't yet open) the WS link when the user
    // explicitly asked for it. When the WS URL was *derived* from an
    // HTTP base, leave construction lazy — most HTTP-only callers
    // never subscribe and never need the WS sibling at all.
    this._ws = wsExplicit && wsUrl ? new _WsLink(wsUrl, this._wsOptions) : null;
    this._wsEager = !!wsExplicit;
    this._wsConnect = null; // memoised connect promise (in-flight or settled)
  }

  /**
   * Subscribe to an SDK lifecycle event: 'disconnected' (fires on any
   * unexpected drop, regardless of `reconnect`), 'reconnecting'
   * ({attempt, delay}), or 'reconnected' (the latter two only with
   * {reconnect: true}).
   */
  on(event, handler) {
    if (!this._listeners.has(event)) this._listeners.set(event, new Set());
    this._listeners.get(event).add(handler);
    return this;
  }

  /** Remove a listener added with {@link RosClient#on}. */
  off(event, handler) {
    this._listeners.get(event)?.delete(handler);
    return this;
  }

  _emit(event, detail) {
    const handlers = this._listeners.get(event);
    if (!handlers) return;
    for (const handler of handlers) {
      try {
        handler(detail);
      } catch (_) {
        // a listener throwing must not break the reconnect loop
      }
    }
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
    if (this._http) tasks.push(this._http.close());
    if (this._wsConnect) {
      // WS is connecting or connected — wait for the open to settle,
      // then close. Swallow the open error: nothing to close in that case.
      tasks.push(
        this._wsConnect.then(
          (link) => link.close(),
          () => undefined
        )
      );
    } else if (this._ws) {
      // Constructed eagerly but connect() never ran — no-op close.
      tasks.push(this._ws.close());
    }
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
          'no WebSocket endpoint available; connect() was given an HTTP-only URL with no WS sibling'
        ),
        { code: 'transport_unavailable' }
      );
    }
    if (this._wsConnect) return this._wsConnect; // open is in-flight or done.
    // Reuse the eagerly-constructed link if present, otherwise build it
    // now (HTTP-derived sibling case).
    if (!this._ws) this._ws = new _WsLink(this._wsUrl, this._wsOptions);
    const link = this._ws;
    this._wsConnect = link.connect().then(
      () => link,
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
    return this._wsConnect;
  }

  // -- verb API --

  async call(capability, payload) {
    if (this._http) return this._http.call(capability, payload);
    const ws = await this._ensureWs();
    return ws.call(capability, payload);
  }

  async publish(capability, payload) {
    if (this._http) return this._http.publish(capability, payload);
    const ws = await this._ensureWs();
    return ws.publish(capability, payload);
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
 * pair plus a `wsExplicit` flag (true when the caller named a WS URL
 * themselves, false when we derived one from an HTTP base). Either
 * URL may be `null` — the corresponding link then isn't constructed
 * and verbs needing it reject with `transport_unavailable`.
 */
function _resolveUrls(url) {
  // Form 1: explicit { http, ws } pair.
  if (url && typeof url === 'object' && !Array.isArray(url)) {
    const httpUrl = _validateEndpoint(url.http, 'http');
    const wsUrl = _normaliseWsPath(_validateEndpoint(url.ws, 'ws'));
    if (!httpUrl && !wsUrl) {
      throw new TypeError(
        'connect({http, ws}): at least one of http or ws must be provided'
      );
    }
    return { httpUrl, wsUrl, wsExplicit: !!wsUrl };
  }

  // Form 2: single URL string. Pick the transport from the scheme.
  if (typeof url !== 'string' || !url) {
    throw new TypeError(
      'connect(url): url must be a non-empty string or {http, ws}'
    );
  }
  if (/^wss?:\/\//i.test(url)) {
    return { httpUrl: null, wsUrl: _normaliseWsPath(url), wsExplicit: true };
  }
  if (/^https?:\/\//i.test(url)) {
    // HTTP base URL: derive a sibling WS URL lazily — we don't open
    // it until the user actually calls subscribe().
    return { httpUrl: url, wsUrl: _deriveWsSibling(url), wsExplicit: false };
  }
  throw new TypeError(
    `connect(url): unrecognised URL scheme: ${url} (expected ws://, wss://, http://, or https://)`
  );
}

// Append the runtime's default `/capability` path when the caller
// passed a bare host (`ws://host:9000` or `ws://host:9000/`). Leave
// any explicit non-empty path untouched so users behind a
// path-rewriting proxy or with a non-default `WebSocketTransport({
// path })` keep working. Returns the input unchanged on parse error
// (let the underlying WebSocket constructor surface the bad URL).
function _normaliseWsPath(wsUrl) {
  if (!wsUrl) return wsUrl;
  try {
    const u = new URL(wsUrl);
    if (u.pathname === '' || u.pathname === '/') {
      u.pathname = '/capability';
      return u.toString();
    }
    return wsUrl;
  } catch (_) {
    return wsUrl;
  }
}

// Swap http(s) → ws(s) and append the default `/capability` path.
// Returns null if the URL can't be parsed; verbs that need WS will
// then surface a clear `transport_unavailable` error themselves.
function _deriveWsSibling(httpUrl) {
  try {
    const u = new URL(httpUrl);
    u.protocol = u.protocol === 'https:' ? 'wss:' : 'ws:';
    u.pathname = u.pathname.replace(/\/+$/, '') + '/capability';
    return u.toString();
  } catch (_) {
    return null;
  }
}

// Per-field config for `_validateEndpoint`. Keeps the validator
// itself a straight three-step check (presence, type, scheme).
const _ENDPOINT_FIELDS = {
  http: { schemeRe: /^https?:\/\//i, expected: 'http:// or https://' },
  ws: { schemeRe: /^wss?:\/\//i, expected: 'ws:// or wss://' },
};

/**
 * Validate one endpoint of an `{http, ws}` pair. Returns the URL
 * unchanged on success, `null` when the field is absent, or throws
 * a clear `TypeError` when present but malformed.
 */
function _validateEndpoint(value, field) {
  if (value === undefined || value === null) return null;
  const { schemeRe, expected } = _ENDPOINT_FIELDS[field];
  if (typeof value !== 'string' || !value) {
    throw new TypeError(
      `connect({${field}}): ${field} must be a non-empty string, got ${typeof value}`
    );
  }
  if (!schemeRe.test(value)) {
    throw new TypeError(
      `connect({${field}}): ${field} must start with ${expected} (got ${value})`
    );
  }
  return value;
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
