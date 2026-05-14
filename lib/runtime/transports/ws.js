// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

const { WebSocketServer } = require('ws');
const debug = require('debug')('rclnodejs:runtime:ws');
const { Connection } = require('../connection.js');
const { TransportAdapter } = require('../transport_adapter.js');

/**
 * A {@link Connection} backed by a single `ws.WebSocket`. JSON-encodes every
 * outbound frame, JSON-decodes every inbound frame, and silently no-ops sends
 * on a closed socket.
 */
class WebSocketConnection extends Connection {
  constructor(ws, req) {
    super();
    this.ws = ws;
    this.req = req; // raw HTTP upgrade request (for auth hooks)
    ws.on('message', (data, isBinary) => {
      if (isBinary) {
        this.send({
          ok: false,
          error: 'binary frames are not supported',
          code: 'binary_unsupported',
        });
        return;
      }
      let frame;
      try {
        frame = JSON.parse(data.toString('utf8'));
      } catch (e) {
        this.send({
          ok: false,
          error: `invalid JSON: ${e.message}`,
          code: 'invalid_json',
        });
        return;
      }
      this.emit('message', frame);
    });
    ws.on('close', () => this.emit('close'));
    ws.on('error', (err) => debug('socket error: %s', err.message));
  }

  send(frame) {
    if (this.ws.readyState !== this.ws.OPEN) return;
    try {
      this.ws.send(JSON.stringify(frame));
    } catch (e) {
      debug('send failed: %s', e.message);
    }
  }

  close(code, reason) {
    try {
      this.ws.close(code, reason);
    } catch {
      /* underlying socket may already be closing */
    }
  }
}

/**
 * Layer-2 WebSocket adapter for the Web Runtime.
 *
 * Speaks the capability wire protocol on a single URL path (default
 * `/capability`). This is intentionally separate from the lower-level
 * {@link module:rosocket} gateway: rosocket exposes raw ROS-shaped URLs
 * (`/topic/*`, `/service/*`); this adapter exposes one duplex channel
 * for the runtime's capability frames.
 */
class WebSocketTransport extends TransportAdapter {
  /**
   * @param {object} [options]
   * @param {number} [options.port=9000]
   * @param {string} [options.host='0.0.0.0']
   * @param {string} [options.path='/capability']
   * @param {(req: import('http').IncomingMessage) => boolean} [options.verifyClient]
   *   Optional auth hook called with the raw HTTP upgrade request. Return
   *   `false` to reject the connection before any frames are exchanged.
   */
  constructor(options = {}) {
    super();
    this.port = options.port != null ? options.port : 9000;
    this.host = options.host != null ? options.host : '0.0.0.0';
    this.path = options.path != null ? options.path : '/capability';
    this.verifyClient = options.verifyClient;
    this._wss = null;
  }

  async start({ onConnection }) {
    if (typeof onConnection !== 'function') {
      throw new TypeError(
        'WebSocketTransport.start: onConnection must be a function'
      );
    }
    return new Promise((resolve, reject) => {
      const wsVerify = this.verifyClient
        ? (info) => this.verifyClient(info.req)
        : undefined;
      this._wss = new WebSocketServer({
        host: this.host,
        port: this.port,
        path: this.path,
        verifyClient: wsVerify,
      });
      this._wss.on('error', reject);
      this._wss.on('connection', (ws, req) => {
        const conn = new WebSocketConnection(ws, req);
        try {
          onConnection(conn);
        } catch (e) {
          debug('onConnection threw: %s', e.stack || e.message);
          conn.close(1011, 'runtime error');
        }
      });
      this._wss.on('listening', () => {
        const addr = this._wss.address();
        this.port = addr.port; // record bound port for ephemeral (port: 0)
        debug(
          'capability runtime listening on %s:%d%s',
          addr.address,
          addr.port,
          this.path
        );
        resolve(addr);
      });
    });
  }

  async stop() {
    if (!this._wss) return;
    const wss = this._wss;
    this._wss = null;
    return new Promise((resolve) => {
      for (const client of wss.clients) {
        try {
          client.close();
        } catch {
          /* client may already be closing */
        }
      }
      wss.close(() => resolve());
    });
  }
}

module.exports = { WebSocketTransport };
