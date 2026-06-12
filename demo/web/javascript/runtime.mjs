// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// rclnodejs/web demo — runtime side (rclnodejs/web runtime + the demo's
// ROS 2 nodes; named `runtime.mjs` to avoid being confused with the
// page-side `static.mjs`).
//
//   1. Source ROS 2 (`source /opt/ros/<distro>/setup.bash`)
//   2. From this folder run `node runtime.mjs`
//   3. In another shell run `node static.mjs` to host
//      `index.html` on http://localhost:8080/ — same split as the
//      TypeScript demo's `tsx server.ts` + `vite`.

import rclnodejs from '../../../index.js';
// In a downstream project this is the public, supported import:
//   import { createRuntime, WebSocketTransport, HttpTransport } from
//     'rclnodejs/web/server';
// Inside this in-repo demo we use the relative path so the file runs
// straight out of a fresh git clone, no `npm install` required.
import {
  createRuntime,
  WebSocketTransport,
  HttpTransport,
} from '../../../lib/runtime/index.js';

const RUNTIME_PORT = Number(process.env.RUNTIME_PORT || 9000);
const HTTP_PORT = Number(process.env.HTTP_PORT || 9001);

function displayHost(host) {
  return host === '0.0.0.0' || host === '::' ? 'localhost' : host;
}

// Render the registry as a small human-readable table:
//   call       /add_two_ints       example_interfaces/srv/AddTwoInts
//   publish    /web_demo_chatter   std_msgs/msg/String
//   subscribe  /web_demo_tick      std_msgs/msg/String
function formatCapabilities(caps) {
  const rows = [];
  for (const verb of ['call', 'publish', 'subscribe']) {
    for (const [topic, type] of Object.entries(caps[verb] || {})) {
      rows.push([verb, topic, type]);
    }
  }
  if (rows.length === 0) return '  (none)';
  const w0 = Math.max(...rows.map((r) => r[0].length));
  const w1 = Math.max(...rows.map((r) => r[1].length));
  return rows
    .map(([v, t, ty]) => `  ${v.padEnd(w0)}  ${t.padEnd(w1)}  ${ty}`)
    .join('\n');
}

// ---- Layer 1: rclnodejs core ----------------------------------------
await rclnodejs.init();
const node = rclnodejs.createNode('rclnodejs_web_demo_node');

// A real ROS 2 service the browser can call.
node.createService(
  'example_interfaces/srv/AddTwoInts',
  '/add_two_ints',
  (request, response) => {
    const reply = response.template;
    reply.sum = request.a + request.b;
    response.send(reply);
  }
);

// A real ROS 2 publisher producing a tick once a second so the
// browser's subscribe() has something to receive without the user
// having to publish first.
const tickPub = node.createPublisher('std_msgs/msg/String', '/web_demo_tick');
let counter = 0;
setInterval(() => {
  tickPub.publish({
    data: `tick ${counter++} @ ${new Date().toISOString()}`,
  });
}, 1000);

rclnodejs.spin(node);

// ---- Layer 2 + 3: capability runtime over WebSocket *and* HTTP -------
// The same dispatcher / registry serves both transports — the L2 seam
// is what proves the runtime is transport-agnostic. Browser SDK picks
// a transport from the URL scheme; curl / Postman / AI agents use the
// HTTP one directly.
const runtime = createRuntime({
  node,
  transports: [
    new WebSocketTransport({
      port: RUNTIME_PORT,
      // '::' = dual-stack: accepts both IPv6 and IPv4-mapped
      // connections. Matches Node's http server default and avoids
      // the WSL2 / glibc "localhost → ::1" mismatch where
      // 0.0.0.0-only servers appear unreachable from the browser.
      host: '::',
    }),
    new HttpTransport({
      port: HTTP_PORT,
      host: '::',
      // Opt-in Server-Sent Events for `subscribe` over plain HTTP:
      //   GET /capability/subscribe/<name>   (text/event-stream)
      // Intended for clients that can't hold a WebSocket open (curl,
      // AI agents, serverless / edge functions). Browser apps should
      // still prefer the WebSocket transport, which multiplexes many
      // topics on one connection. Off by default in HttpTransport.
      sse: true,
      cors: true,
    }),
  ],
});
runtime.expose({
  call: { '/add_two_ints': 'example_interfaces/srv/AddTwoInts' },
  publish: { '/web_demo_chatter': 'std_msgs/msg/String' },
  subscribe: {
    '/web_demo_tick': 'std_msgs/msg/String',
    '/web_demo_chatter': 'std_msgs/msg/String',
    // Pairs with the stock publisher example so developers can feed the
    // demo from their own node instead of the built-in tick loop:
    //   node ../../../example/topics/publisher/publisher-example.mjs
    // then subscribe to `/topic` from the browser / curl / EventSource.
    '/topic': 'std_msgs/msg/String',
  },
});
await runtime.start();

const caps = runtime.registry.list();
const total =
  Object.keys(caps.call || {}).length +
  Object.keys(caps.publish || {}).length +
  Object.keys(caps.subscribe || {}).length;

console.log('rclnodejs/web demo running (JavaScript)');
console.log(
  `  WebSocket : ws://${displayHost('::')}:${RUNTIME_PORT}/capability`
);
console.log(
  `  HTTP      : http://${displayHost('::')}:${HTTP_PORT}/capability  (call / publish, curl-able)`
);
console.log(
  `  HTTP SSE  : http://${displayHost('::')}:${HTTP_PORT}/capability/subscribe/<name>  (subscribe via text/event-stream)`
);
console.log();
console.log(`Exposed capabilities (${total}):`);
console.log(formatCapabilities(caps));
console.log();
console.log(
  'Static page: run `node static.mjs` in another shell, then open http://localhost:8080/'
);

// ---- Graceful shutdown ----------------------------------------------
const stop = async () => {
  console.log('\nstopping…');
  await runtime.stop();
  rclnodejs.shutdown();
  process.exit(0);
};
process.once('SIGINT', stop);
process.once('SIGTERM', stop);
