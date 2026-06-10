// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

// rosocket demo server.
//
// Bridges two ROS 2 endpoints to plain WebSocket URLs so a browser can
// drive them with built-in `WebSocket` + `JSON`, no client library:
//
//   ws://<host>:9000/topic/chatter        std_msgs/msg/String
//   ws://<host>:9000/service/add_two_ints example_interfaces/srv/AddTwoInts
//
// Run inside WSL (where ROS 2 is sourced):
//   node demo/rosocket/server.cjs
//
// To exercise the service, start the existing AddTwoInts example in a
// second terminal (it implements `/add_two_ints`):
//   node example/services/service/service-example.cjs
//
// Then open demo/rosocket/index.html on the Windows host browser. WSL2
// forwards localhost so `ws://localhost:9000` works as-is. See README.md
// for fallback instructions.

const rclnodejs = require('../../index.js').default;
const { startRosocket } = require('../../rosocket');

const PORT = Number(process.env.PORT) || 9000;
const HOST = process.env.HOST || '0.0.0.0';

async function main() {
  await rclnodejs.init();
  const node = rclnodejs.createNode('rosocket_demo_node');
  rclnodejs.spin(node);

  const bridge = await startRosocket({
    node,
    port: PORT,
    host: HOST,
    topicTypes: {
      '/chatter': 'std_msgs/msg/String',
    },
    serviceTypes: {
      '/add_two_ints': 'example_interfaces/srv/AddTwoInts',
    },
  });

  const displayHost = HOST === '0.0.0.0' || HOST === '::' ? 'localhost' : HOST;
  const baseUrl = `ws://${displayHost}:${bridge.port}`;
  console.log(
    `[rosocket-demo] listening on ${baseUrl} (bind=${HOST}:${bridge.port})`
  );
  console.log('[rosocket-demo] endpoints:');
  console.log(`  ${baseUrl}/topic/chatter`);
  console.log(`  ${baseUrl}/service/add_two_ints`);
  console.log('Open demo/rosocket/index.html in the host browser to try it.');

  const shutdown = () => {
    bridge.close().finally(() => {
      try {
        rclnodejs.shutdown();
      } catch (_) {}
      process.exit(0);
    });
  };
  process.on('SIGINT', shutdown);
  process.on('SIGTERM', shutdown);
}

main().catch((e) => {
  console.error(e.stack || e.message);
  process.exit(1);
});
