// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

import { CapabilityRegistry } from './capability_registry.js';
import { Dispatcher } from './dispatcher.js';
import { Connection } from './connection.js';
import { TransportAdapter } from './transport_adapter.js';
import { WebSocketTransport } from './transports/ws.js';
import { HttpTransport } from './transports/http.js';

/**
 * The Web Runtime composes a capability registry, a dispatcher, and one or
 * more transport adapters around an existing rclnodejs `Node`.
 *
 * Lifecycle:
 *   1. Construct via {@link createRuntime}.
 *   2. Register capabilities with {@link Runtime#expose}.
 *   3. Begin accepting connections with {@link Runtime#start}.
 *   4. {@link Runtime#stop} when shutting down.
 *
 * The runtime is **additive** to the existing rclnodejs Node — it does not
 * take ownership of the Node and does not stop it on `runtime.stop()`. The
 * caller is responsible for `rclnodejs.shutdown()`.
 */
class Runtime {
  /**
   * @param {object} options
   * @param {import('../node.js')} options.node
   * @param {CapabilityRegistry} options.registry
   * @param {TransportAdapter[]} options.transports
   */
  constructor({ node, registry, transports }) {
    this.node = node;
    this.registry = registry;
    this.transports = transports;
    this.dispatcher = new Dispatcher({ node, registry });
    this._started = false;
  }

  /**
   * Forward to the registry. Returns `this` for chaining.
   * @param {Parameters<CapabilityRegistry['expose']>[0]} spec
   */
  expose(spec) {
    this.registry.expose(spec);
    return this;
  }

  /**
   * Start every configured transport. Each transport begins listening and
   * routes its connections through the shared dispatcher.
   *
   * If a later transport fails to start, every transport that *did* start
   * is stopped before the original error propagates so the runtime never
   * leaves stray listeners behind.
   *
   * @returns {Promise<Runtime>}
   */
  async start() {
    if (this._started) return this;
    const onConnection = (conn) => this.dispatcher.handle(conn);
    const started = [];
    try {
      for (const t of this.transports) {
        await t.start({ onConnection });
        started.push(t);
      }
    } catch (err) {
      for (const t of started.reverse()) {
        try {
          await t.stop();
        } catch {
          /* swallow rollback errors so the original failure surfaces */
        }
      }
      throw err;
    }
    this._started = true;
    return this;
  }

  /** Stop every configured transport. */
  async stop() {
    for (const t of this.transports) {
      try {
        await t.stop();
      } catch {
        /* best-effort: continue stopping the remaining transports */
      }
    }
    this._started = false;
  }
}

/**
 * Convenience factory. Defaults to a single {@link WebSocketTransport} on
 * port 9000, path `/capability`. Pass `transport` or `transports` to swap
 * or add other adapters.
 *
 * @param {object} options
 * @param {import('../node.js')} options.node
 * @param {TransportAdapter} [options.transport]
 * @param {TransportAdapter[]} [options.transports]
 * @returns {Runtime}
 */
function createRuntime({ node, transport, transports } = {}) {
  if (!node) throw new TypeError('createRuntime: options.node is required');
  let resolvedTransports;
  if (Array.isArray(transports) && transports.length > 0) {
    resolvedTransports = transports;
  } else if (transport) {
    resolvedTransports = [transport];
  } else {
    resolvedTransports = [new WebSocketTransport()];
  }
  return new Runtime({
    node,
    registry: new CapabilityRegistry(),
    transports: resolvedTransports,
  });
}

export {
  createRuntime,
  Runtime,
  CapabilityRegistry,
  Dispatcher,
  Connection,
  TransportAdapter,
  WebSocketTransport,
  HttpTransport,
};
