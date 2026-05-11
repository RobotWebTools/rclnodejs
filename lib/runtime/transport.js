// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

const EventEmitter = require('events');

/**
 * @typedef {Object} CapabilityFrame
 * @property {string|number} [id]      - Caller-assigned request id (echoed in reply).
 * @property {'call'|'publish'|'subscribe'|'unsubscribe'|'action'} [kind] - Request kind (C→S only). `'action'` is reserved and currently returns `code: 'not_implemented'`.
 * @property {string} [capability]     - Capability name, e.g. `/cmd_vel`.
 * @property {*}     [payload]         - Message payload (call request, publish msg, sub event).
 * @property {string|number} [subId]   - For unsubscribe: id of the original subscribe.
 * @property {boolean} [ok]            - Reply success flag (S→C only).
 * @property {string} [event]          - `'message'` for streamed subscription deliveries.
 * @property {string} [error]          - Human-readable error message on failure.
 * @property {string} [code]           - Stable machine-readable error code.
 */

/**
 * Per-connection abstraction handed to the runtime by a transport adapter.
 *
 * The transport owns the wire (WebSocket today; other adapters may follow)
 * and exposes each connection as a `Connection` so the runtime can stay
 * transport-agnostic. Concrete adapters subclass `Connection` and call
 * `this.emit('message', frame)` / `this.emit('close')` as frames arrive on
 * the wire, and implement `send(frame)` / `close(code, reason)` to push
 * frames back out.
 *
 * @experimental — the base class shape may grow (e.g. backpressure hooks)
 * and is not part of the SemVer-stable surface for third-party adapter
 * authors. The first-party `WebSocketTransport` is stable and recommended
 * for production use.
 *
 * @event Connection#message
 * @type {CapabilityFrame}
 *
 * @event Connection#close
 */
class Connection extends EventEmitter {
  /**
   * Push a frame to the remote peer. Implementations should silently drop
   * sends on a closed connection rather than throwing.
   * @param {CapabilityFrame} frame
   */
  // eslint-disable-next-line no-unused-vars
  send(frame) {
    throw new Error('Connection.send() must be implemented by the transport');
  }

  /**
   * Close the connection.
   * @param {number} [code]
   * @param {string} [reason]
   */
  // eslint-disable-next-line no-unused-vars
  close(code, reason) {
    throw new Error('Connection.close() must be implemented by the transport');
  }
}

/**
 * Base class / contract for Layer-2 transport adapters.
 *
 * A transport adapter:
 *   1. Owns the network listener (WS / HTTP / SSE / ...).
 *   2. Builds a {@link Connection} per accepted client.
 *   3. Calls `onConnection(conn)` from the start options once per client.
 *
 * The runtime never speaks to the wire directly — it only consumes the
 * Connection event surface. This is the seam that lets future adapters
 * (HTTP, etc.) drop in without touching the runtime or the SDK.
 *
 * @experimental — same forward-compat caveat as {@link Connection}: the
 * adapter contract may grow to support new transports. First-party
 * `WebSocketTransport` is stable.
 */
class TransportAdapter {
  /**
   * Begin accepting connections.
   * @param {object} options
   * @param {(conn: Connection) => void} options.onConnection
   * @returns {Promise<{address: string, port: number}>}
   */
  // eslint-disable-next-line no-unused-vars
  async start(options) {
    throw new Error('TransportAdapter.start() must be implemented');
  }

  /** Stop accepting and close all open connections. */
  async stop() {
    throw new Error('TransportAdapter.stop() must be implemented');
  }
}

module.exports = { Connection, TransportAdapter };
