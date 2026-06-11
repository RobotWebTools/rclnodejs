// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// rclnodejs/web demo — static-file server (page side; named `static.mjs`
// to avoid being confused with the runtime-side `runtime.mjs`).
//
// Serves index.html on port 8080 and maps `/sdk/*` to the in-repo
// `web/` folder so the page can `import { connect } from '/sdk/index.js'`
// without bundling. In a downstream project you'd `npm install rclnodejs`
// and `import { connect } from 'rclnodejs/web'` instead.
//
// Pair with `node runtime.mjs` (the rclnodejs/web runtime + the demo's
// ROS 2 nodes) in another shell — the same split as the TypeScript
// demo's `tsx server.ts` + `vite`. Production deployments use nginx,
// a CDN, or any other static host.

import path from 'node:path';
import http from 'node:http';
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const STATIC_PORT = Number(process.env.STATIC_PORT || 8080);

const demoDir = __dirname;
const sdkDir = path.resolve(__dirname, '..', '..', '..', 'web');

const server = http.createServer((req, res) => {
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
  // Confine reads to baseDir. `path.relative` collapses `..` segments,
  // so any escape attempt either yields a result that starts with `..`
  // or is absolute — reject both. (`startsWith(baseDir)` alone would
  // false-positive on a sibling like `${baseDir}-other/...`.)
  const rel = path.relative(baseDir, filePath);
  if (rel.startsWith('..') || path.isAbsolute(rel)) {
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

server.on('error', (err) => {
  console.error(err);
  process.exit(1);
});

server.listen(STATIC_PORT, () => {
  console.log(`Static files : http://localhost:${STATIC_PORT}/`);
});

const stop = () => {
  console.log('\nstopping…');
  server.close();
  process.exit(0);
};
process.once('SIGINT', stop);
process.once('SIGTERM', stop);
