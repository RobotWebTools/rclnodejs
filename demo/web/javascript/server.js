// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// rclnodejs/web demo — server side.
//
//   1. Source ROS 2 (`source /opt/ros/<distro>/setup.bash`)
//   2. From this folder run `node server.js`
//   3. Open index.html in any browser (see README for the deploy options)

'use strict';

const path = require('path');
const http = require('http');
const fs = require('fs');

const rclnodejs = require('../../../index.js');
const {
  createRuntime,
  WebSocketTransport,
  HttpTransport,
} = require('../../../lib/runtime');

const RUNTIME_PORT = Number(process.env.RUNTIME_PORT || 9000);
const HTTP_PORT = Number(process.env.HTTP_PORT || 9001);
const STATIC_PORT = Number(process.env.STATIC_PORT || 8080);

function displayHost(host) {
  return host === '0.0.0.0' || host === '::' ? 'localhost' : host;
}

async function main() {
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
      }),
    ],
  });
  runtime.expose({
    call: { '/add_two_ints': 'example_interfaces/srv/AddTwoInts' },
    publish: { '/web_demo_chatter': 'std_msgs/msg/String' },
    subscribe: {
      '/web_demo_tick': 'std_msgs/msg/String',
      '/web_demo_chatter': 'std_msgs/msg/String',
    },
  });
  await runtime.start();

  console.log(
    `rclnodejs/web : ws://${displayHost('::')}:${RUNTIME_PORT}/capability`
  );
  console.log(
    `              also http://${displayHost('::')}:${HTTP_PORT}/capability  (call/publish, curl-able)`
  );
  console.log('Capabilities  :', JSON.stringify(runtime.registry.list()));

  // ---- Optional: serve index.html so the user does not need any
  //                bundler or python -m http.server. Static asset
  //                server only — production deployments use nginx,
  //                a CDN, or any static host.
  //
  // The static server also maps `/sdk/*` to the in-repo `web/`
  // folder so `index.html` can `import { connect } from '/sdk/index.js'`
  // without bundling. In a downstream project you'd `npm install
  // rclnodejs` and import from `node_modules/rclnodejs/web/index.js`
  // (or your CDN of choice).
  const demoDir = __dirname;
  const sdkDir = path.resolve(__dirname, '..', '..', '..', 'web');
  const staticServer = http.createServer((req, res) => {
    let urlPath = (req.url || '/').split('?')[0];
    if (urlPath === '/') urlPath = '/index.html';

    let baseDir;
    let relPath;
    if (urlPath.startsWith('/sdk/')) {
      baseDir = sdkDir;
      relPath = urlPath.slice('/sdk/'.length);
    } else {
      baseDir = demoDir;
      relPath = urlPath.replace(/^\/+/, '');
    }
    const filePath = path.resolve(baseDir, relPath);
    if (!filePath.startsWith(baseDir)) {
      res.writeHead(403).end('forbidden');
      return;
    }
    fs.readFile(filePath, (err, data) => {
      if (err) {
        res.writeHead(404).end('not found');
        return;
      }
      const ext = path.extname(filePath).toLowerCase();
      const ctype =
        {
          '.html': 'text/html; charset=utf-8',
          '.js': 'application/javascript; charset=utf-8',
          '.mjs': 'application/javascript; charset=utf-8',
          '.css': 'text/css; charset=utf-8',
          '.json': 'application/json; charset=utf-8',
        }[ext] || 'application/octet-stream';
      res.writeHead(200, { 'content-type': ctype }).end(data);
    });
  });
  staticServer.listen(STATIC_PORT, () => {
    console.log(`Static files : http://localhost:${STATIC_PORT}/`);
  });

  // ---- Graceful shutdown ----------------------------------------------
  const stop = async () => {
    console.log('\nstopping…');
    staticServer.close();
    await runtime.stop();
    rclnodejs.shutdown();
    process.exit(0);
  };
  process.once('SIGINT', stop);
  process.once('SIGTERM', stop);
}

main().catch((err) => {
  console.error(err);
  process.exit(1);
});
