// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

const debug = require('debug')('rclnodejs:runtime');
const { toJSONSafe, reviveBigInts } = require('../message_serialization.js');

/**
 * Per-connection request handler. Statelessly routes inbound capability
 * frames to the registered ROS 2 endpoints, lazily creating publishers,
 * subscriptions, and clients on first use and tearing them down on close.
 *
 * Rejects any frame whose capability is not in the {@link CapabilityRegistry}
 * — this is the security contract that distinguishes the runtime from the
 * raw rosocket bridge.
 */
class Dispatcher {
  /**
   * @param {object} options
   * @param {import('../node.js')} options.node
   * @param {import('./registry.js').CapabilityRegistry} options.registry
   */
  constructor({ node, registry }) {
    if (!node) throw new TypeError('Dispatcher: options.node is required');
    if (!registry)
      throw new TypeError('Dispatcher: options.registry is required');
    this.node = node;
    this.registry = registry;
  }

  /**
   * Bind handlers for a freshly-accepted connection. The dispatcher takes
   * full ownership of the connection's lifecycle from this point on.
   * @param {import('./transport.js').Connection} conn
   */
  handle(conn) {
    const state = {
      publishers: new Map(), // capability name -> publisher
      subscriptions: new Map(), // subId -> { name, subscription }
      clients: new Map(), // capability name -> client
    };

    const cleanup = () => {
      for (const { subscription } of state.subscriptions.values()) {
        try {
          this.node.destroySubscription(subscription);
        } catch {
          /* destroy is best-effort during connection teardown */
        }
      }
      state.subscriptions.clear();
      for (const pub of state.publishers.values()) {
        try {
          this.node.destroyPublisher(pub);
        } catch {
          /* destroy is best-effort during connection teardown */
        }
      }
      state.publishers.clear();
      for (const cli of state.clients.values()) {
        try {
          this.node.destroyClient(cli);
        } catch {
          /* destroy is best-effort during connection teardown */
        }
      }
      state.clients.clear();
    };

    conn.on('close', cleanup);
    conn.on('message', (frame) => this._dispatchFrame(conn, frame, state));
  }

  _dispatchFrame(conn, frame, state) {
    if (!frame || typeof frame !== 'object') {
      conn.send({
        ok: false,
        error: 'invalid frame: expected JSON object',
        code: 'invalid_frame',
      });
      return;
    }
    const { id, kind, capability } = frame;
    try {
      switch (kind) {
        case 'call':
          return this._handleCall(conn, id, capability, frame.payload, state);
        case 'publish':
          return this._handlePublish(
            conn,
            id,
            capability,
            frame.payload,
            state
          );
        case 'subscribe':
          return this._handleSubscribe(conn, id, capability, state);
        case 'unsubscribe':
          return this._handleUnsubscribe(conn, id, frame.subId, state);
        case 'action':
          // Reserved for future Action capability dispatch (planned for
          // 2.2.x). Returning a stable code now keeps the wire protocol
          // forward-compatible for clients that probe for support.
          conn.send({
            id,
            ok: false,
            error: 'action capabilities are not yet supported',
            code: 'not_implemented',
          });
          return;
        default:
          conn.send({
            id,
            ok: false,
            error: `unknown kind: ${kind}`,
            code: 'unknown_kind',
          });
      }
    } catch (e) {
      debug('dispatch error: %s', e.stack || e.message);
      conn.send({
        id,
        ok: false,
        error: e.message,
        code: 'internal_error',
      });
    }
  }

  _handleCall(conn, id, name, payload, { clients }) {
    const cap = this.registry.resolve('call', name);
    if (!cap) return this._notExposed(conn, id, 'call', name);
    let client = clients.get(name);
    if (!client) {
      client = this.node.createClient(cap.type, name);
      clients.set(name, client);
    }
    const request = reviveBigInts(payload);
    try {
      client.sendRequest(request, (response) => {
        conn.send({ id, ok: true, payload: toJSONSafe(response) });
      });
    } catch (e) {
      conn.send({
        id,
        ok: false,
        error: `call failed: ${e.message}`,
        code: 'call_failed',
      });
    }
  }

  _handlePublish(conn, id, name, payload, { publishers }) {
    const cap = this.registry.resolve('publish', name);
    if (!cap) return this._notExposed(conn, id, 'publish', name);
    let pub = publishers.get(name);
    if (!pub) {
      pub = this.node.createPublisher(cap.type, name);
      publishers.set(name, pub);
    }
    try {
      pub.publish(reviveBigInts(payload));
      conn.send({ id, ok: true });
    } catch (e) {
      conn.send({
        id,
        ok: false,
        error: `publish failed: ${e.message}`,
        code: 'publish_failed',
      });
    }
  }

  _handleSubscribe(conn, id, name, { subscriptions }) {
    const cap = this.registry.resolve('subscribe', name);
    if (!cap) return this._notExposed(conn, id, 'subscribe', name);
    if (id === undefined || id === null) {
      conn.send({
        ok: false,
        error: 'subscribe requires an id',
        code: 'missing_id',
      });
      return;
    }
    if (subscriptions.has(id)) {
      conn.send({
        id,
        ok: false,
        error: `id already in use: ${id}`,
        code: 'duplicate_id',
      });
      return;
    }
    const subscription = this.node.createSubscription(cap.type, name, (msg) => {
      conn.send({ event: 'message', subId: id, payload: toJSONSafe(msg) });
    });
    subscriptions.set(id, { name, subscription });
    conn.send({ id, ok: true });
  }

  _handleUnsubscribe(conn, id, subId, { subscriptions }) {
    if (subId === undefined || subId === null) {
      conn.send({
        id,
        ok: false,
        error: 'unsubscribe requires subId',
        code: 'missing_sub_id',
      });
      return;
    }
    const entry = subscriptions.get(subId);
    if (!entry) {
      conn.send({
        id,
        ok: false,
        error: `unknown subId: ${subId}`,
        code: 'unknown_sub_id',
      });
      return;
    }
    try {
      this.node.destroySubscription(entry.subscription);
    } catch {
      /* destroy is best-effort */
    }
    subscriptions.delete(subId);
    conn.send({ id, ok: true });
  }

  _notExposed(conn, id, kind, name) {
    conn.send({
      id,
      ok: false,
      error: `capability not exposed: ${kind} ${name}`,
      code: 'not_exposed',
    });
  }
}

module.exports = { Dispatcher };
