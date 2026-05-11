// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

/**
 * Base class / contract for transport adapters.
 *
 * A transport adapter:
 *   1. Owns the network listener (WS / HTTP / SSE / ...).
 *   2. Builds a {@link import('./connection.js').Connection} per accepted client.
 *   3. Calls `onConnection(conn)` from the start options once per client.
 *
 * The runtime never speaks to the wire directly — it only consumes the
 * Connection event surface. This is the seam that lets future adapters
 * (HTTP, etc.) drop in without touching the runtime or the SDK.
 *
 * @experimental — same forward-compat caveat as
 * {@link import('./connection.js').Connection}: the adapter contract may
 * grow to support new transports. First-party `WebSocketTransport` is
 * stable.
 */
class TransportAdapter {
  /**
   * Begin accepting connections.
   * @param {object} options
   * @param {(conn: import('./connection.js').Connection) => void} options.onConnection
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

module.exports = { TransportAdapter };
