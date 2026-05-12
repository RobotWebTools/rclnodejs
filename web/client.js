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
// In a real browser, `WebSocket` is a global. In Node — used by the
// integration tests and SSR scenarios — we fall back to the optional
// `ws` package via dynamic import.
//
// This module never imports rclnodejs itself — zero native deps,
// safe to bundle for the browser.
//
// Today the SDK only speaks the WebSocket transport. An HTTP transport
// is planned and will be added behind a `connect({http, ws})` form
// once the server-side `HttpTransport` ships.

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

// -------------------------------------------------------------------
// Public surface
// -------------------------------------------------------------------

/**
 * Thin client for the rclnodejs Web Runtime capability protocol.
 *
 * Today the only supported transport is WebSocket — pass a `ws://` or
 * `wss://` URL. HTTP support is planned and will be wired in once the
 * server-side `HttpTransport` ships.
 *
 * **Path conventions.** The default `WebSocketTransport` listens on
 * `/capability`, so `connect('ws://host:9000/capability')` is the
 * normal form. If you change `--path` on the server (or sit it behind
 * a path-rewriting proxy), pass the full URL accordingly.
 *
 * @example
 *   import { connect } from 'rclnodejs/web';
 *
 *   const ros = await connect('ws://robot.local:9000/capability');
 *   const reply = await ros.call('/add_two_ints', { a: '2n', b: '40n' });
 */
export class RosClient {
  /**
   * @param {string} url WebSocket URL (`ws://` or `wss://`).
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
    this.url = _resolveWsUrl(url);
    this._ws = new _WsLink(this.url);
  }

  /** Open the WebSocket. */
  async connect() {
    await this._ws.connect();
    return this;
  }

  /** Close the WebSocket. */
  async close() {
    await this._ws.close();
  }

  // -- verb API --

  call(capability, payload) {
    return this._ws.call(capability, payload);
  }

  publish(capability, payload) {
    return this._ws.publish(capability, payload);
  }

  async subscribe(capability, callback) {
    if (typeof callback !== 'function') {
      throw new TypeError('subscribe(capability, callback): callback required');
    }
    return this._ws.subscribe(capability, callback);
  }
}

/**
 * Validate the user-supplied `connect()` URL. Only `ws://` and `wss://`
 * are accepted today; anything else throws so HTTP-only callers fail
 * fast at construction time instead of much later on first verb.
 */
function _resolveWsUrl(url) {
  if (typeof url !== 'string' || !url) {
    throw new TypeError('connect(url): url must be a non-empty string');
  }
  if (!/^wss?:\/\//i.test(url)) {
    throw new TypeError(
      `connect(url): unsupported URL scheme: ${url} (expected ws:// or wss://; HTTP transport is not yet supported)`
    );
  }
  return url;
}

/**
 * Open a connection to a Web Runtime capability endpoint.
 *
 * See {@link RosClient} for path conventions. Today only WebSocket
 * URLs (`ws://`, `wss://`) are accepted.
 *
 * @param {string} url
 * @param {object} [options]
 * @returns {Promise<RosClient>}
 */
export async function connect(url, options) {
  const c = new RosClient(url, options);
  await c.connect();
  return c;
}
