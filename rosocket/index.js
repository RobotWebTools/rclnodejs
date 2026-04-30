// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

const { WebSocketServer } = require('ws');
const debug = require('debug')('rclnodejs:rosocket');
const { toJSONSafe } = require('../lib/message_serialization.js');

// Convert a JSON value coming from the browser into shapes rclnodejs understands.
// Mainly: integer fields declared as 64-bit need to be BigInt. We don't have schema
// information here, so we accept either Number or BigInt for fields and let the
// underlying serializer coerce. For convenience we convert string-encoded ints
// like "12n" produced by toJSONSafe back to BigInt.
function reviveBigInts(value) {
  if (value === null || typeof value !== 'object') {
    if (typeof value === 'string' && /^-?\d+n$/.test(value)) {
      return BigInt(value.slice(0, -1));
    }
    return value;
  }
  if (Array.isArray(value)) return value.map(reviveBigInts);
  const out = {};
  for (const k of Object.keys(value)) out[k] = reviveBigInts(value[k]);
  return out;
}

function safeSend(ws, payload) {
  if (ws.readyState !== ws.OPEN) return;
  try {
    ws.send(typeof payload === 'string' ? payload : JSON.stringify(payload));
  } catch (e) {
    debug('send failed: %s', e.message);
  }
}

function sendError(ws, message, extra) {
  safeSend(ws, { error: message, ...(extra || {}) });
}

// Decode a URL path into { kind, name } where kind is 'topic' or 'service'.
// Examples:
//   /topic/chatter            -> { kind:'topic',   name:'/chatter' }
//   /topic/ns/sub/chatter     -> { kind:'topic',   name:'/ns/sub/chatter' }
//   /service/add_two_ints     -> { kind:'service', name:'/add_two_ints' }
function parseResourcePath(pathname) {
  const m = /^\/(topic|service)\/(.+)$/.exec(pathname);
  if (!m) return null;
  const name = '/' + m[2].replace(/^\/+/, '');
  if (name.length < 2) return null;
  return { kind: m[1], name };
}

/**
 * Start a resource-style WebSocket bridge that exposes ROS 2 topics and
 * services as plain WebSocket URLs carrying ROS messages as JSON.
 *
 * URL scheme:
 *   ws://host:port/topic/<topic_name>?type=<pkg>/msg/<Type>
 *   ws://host:port/service/<service_name>?type=<pkg>/srv/<Type>
 *
 * The browser only needs the built-in `WebSocket` and `JSON` APIs.
 *
 * @param {Object} options
 * @param {import('../lib/node.js')} options.node - rclnodejs Node to host pubs/subs/clients on.
 * @param {number} [options.port=9000] - Port to listen on.
 * @param {string} [options.host='0.0.0.0'] - Host to bind to.
 * @param {Object<string,string>} [options.topicTypes] - Optional default type per topic name (e.g. {"/chatter":"std_msgs/msg/String"}).
 * @param {Object<string,string>} [options.serviceTypes] - Optional default type per service name.
 * @param {(req: import('http').IncomingMessage) => boolean} [options.verifyClient] - Optional auth hook called with the raw HTTP upgrade request; return `false` to reject the connection.
 * @returns {Promise<{wss: WebSocketServer, close: () => Promise<void>, port: number}>}
 */
function startRosocket(options = {}) {
  const {
    node,
    port = 9000,
    host = '0.0.0.0',
    topicTypes = {},
    serviceTypes = {},
    verifyClient,
  } = options;

  if (!node) throw new TypeError('startRosocket: options.node is required');

  // ws's verifyClient is invoked with `info = { origin, secure, req }`,
  // not the raw IncomingMessage. Wrap it so users can write a simple
  // `(req) => boolean` hook as documented above.
  const wsVerifyClient = verifyClient
    ? (info) => verifyClient(info.req)
    : undefined;

  return new Promise((resolve, reject) => {
    const wss = new WebSocketServer({
      host,
      port,
      verifyClient: wsVerifyClient,
    });

    wss.on('error', reject);

    wss.on('connection', (ws, req) => {
      const url = new URL(req.url, 'http://localhost');
      const resource = parseResourcePath(url.pathname);
      if (!resource) {
        sendError(ws, `Unknown path: ${url.pathname}`);
        ws.close(1008, 'unknown path');
        return;
      }
      const typeFromQuery = url.searchParams.get('type');
      const typeFromMap =
        resource.kind === 'topic'
          ? topicTypes[resource.name]
          : serviceTypes[resource.name];
      const typeName = typeFromQuery || typeFromMap;
      if (!typeName) {
        sendError(
          ws,
          `Missing message type. Specify ?type=<pkg>/${resource.kind === 'topic' ? 'msg' : 'srv'}/<Type> or configure it server-side.`
        );
        ws.close(1008, 'missing type');
        return;
      }

      try {
        if (resource.kind === 'topic') {
          handleTopic(ws, node, resource.name, typeName);
        } else {
          handleService(ws, node, resource.name, typeName);
        }
      } catch (e) {
        debug('connection setup failed: %s', e.stack || e.message);
        sendError(ws, e.message);
        ws.close(1011, 'setup error');
      }
    });

    wss.on('listening', () => {
      const addr = wss.address();
      debug('rosocket listening on %s:%d', addr.address, addr.port);
      resolve({
        wss,
        port: addr.port,
        close: () =>
          new Promise((res) => {
            for (const client of wss.clients) {
              try {
                client.close();
              } catch (_) {}
            }
            wss.close(() => res());
          }),
      });
    });
  });
}

function handleTopic(ws, node, topicName, typeName) {
  // Lazily create both a publisher and a subscription on first need.
  // The publisher is only created if/when the client publishes; the
  // subscription is created immediately so the client receives messages.
  let publisher = null;
  const subscription = node.createSubscription(typeName, topicName, (msg) => {
    safeSend(ws, toJSONSafe(msg));
  });
  debug('subscribed %s [%s]', topicName, typeName);

  ws.on('message', (data, isBinary) => {
    if (isBinary) {
      sendError(ws, 'binary frames not supported on /topic/*');
      return;
    }
    let msg;
    try {
      msg = reviveBigInts(JSON.parse(data.toString('utf8')));
    } catch (e) {
      sendError(ws, `invalid JSON: ${e.message}`);
      return;
    }
    try {
      if (!publisher) {
        publisher = node.createPublisher(typeName, topicName);
      }
      publisher.publish(msg);
    } catch (e) {
      sendError(ws, `publish failed: ${e.message}`);
    }
  });

  ws.on('close', () => {
    try {
      node.destroySubscription(subscription);
    } catch (_) {}
    if (publisher) {
      try {
        node.destroyPublisher(publisher);
      } catch (_) {}
    }
  });
}

function handleService(ws, node, serviceName, typeName) {
  const client = node.createClient(typeName, serviceName);
  debug('service client created %s [%s]', serviceName, typeName);

  ws.on('message', (data, isBinary) => {
    if (isBinary) {
      sendError(ws, 'binary frames not supported on /service/*');
      return;
    }
    let parsed;
    try {
      parsed = JSON.parse(data.toString('utf8'));
    } catch (e) {
      sendError(ws, `invalid JSON: ${e.message}`);
      return;
    }
    // Allow either bare request {a:1,b:2} or wrapped {id, request}.
    const id = parsed && typeof parsed === 'object' ? parsed.id : undefined;
    const request = reviveBigInts(
      parsed && parsed.request !== undefined ? parsed.request : parsed
    );

    try {
      client.sendRequest(request, (response) => {
        const body = toJSONSafe(response);
        safeSend(ws, id !== undefined ? { id, response: body } : body);
      });
    } catch (e) {
      sendError(ws, `service call failed: ${e.message}`, { id });
    }
  });

  ws.on('close', () => {
    try {
      node.destroyClient(client);
    } catch (_) {}
  });
}

module.exports = { startRosocket };
