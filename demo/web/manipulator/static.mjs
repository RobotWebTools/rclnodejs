// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
// Licensed under the Apache License, Version 2.0.

import fs from 'node:fs';
import http from 'node:http';
import { fileURLToPath } from 'node:url';
import { TOPIC } from './joint-state.js';

export function parsePort(value, name) {
  if (!/^\d+$/.test(String(value))) {
    throw new TypeError(`${name} must be an integer from 0 to 65535`);
  }
  const port = Number(value);
  if (!Number.isSafeInteger(port) || port < 0 || port > 65535) {
    throw new RangeError(`${name} must be an integer from 0 to 65535`);
  }
  return port;
}

export function createStaticServer(runtimePort) {
  const files = new Map(
    [
      ['/', './index.html', 'text/html'],
      ['/index.html', './index.html', 'text/html'],
      ['/style.css', './style.css', 'text/css'],
      ['/app.js', './app.js', 'application/javascript'],
      ['/scene.js', './scene.js', 'application/javascript'],
      ['/joint-state.js', './joint-state.js', 'application/javascript'],
      ['/sdk/index.js', '../../../web/index.js', 'application/javascript'],
      ['/sdk/client.js', '../../../web/client.js', 'application/javascript'],
      [
        '/vendor/three.module.js',
        './node_modules/three/build/three.module.js',
        'application/javascript',
      ],
      [
        '/vendor/three.core.js',
        './node_modules/three/build/three.core.js',
        'application/javascript',
      ],
      [
        '/vendor/OrbitControls.js',
        './node_modules/three/examples/jsm/controls/OrbitControls.js',
        'application/javascript',
      ],
    ].map(([route, filename, type]) => [
      route,
      { filename: fileURLToPath(new URL(filename, import.meta.url)), type },
    ])
  );
  for (const [route, file] of files) {
    if (!fs.existsSync(file.filename)) {
      throw new Error(
        `Missing demo asset ${route}; run npm install in demo/web/manipulator`
      );
    }
  }
  return http.createServer((request, response) => {
    if (!['GET', 'HEAD'].includes(request.method)) {
      response.writeHead(405, { Allow: 'GET, HEAD' }).end('Method not allowed');
      return;
    }
    if (!URL.canParse(request.url, 'http://localhost')) {
      response.writeHead(400).end('Invalid request URL');
      return;
    }
    const pathname = new URL(request.url, 'http://localhost').pathname;
    if (pathname === '/config.json') {
      response.writeHead(200, {
        'content-type': 'application/json; charset=utf-8',
        'cache-control': 'no-store',
      });
      response.end(
        request.method === 'HEAD'
          ? undefined
          : JSON.stringify({
              runtimePort,
              topic: TOPIC,
            })
      );
      return;
    }
    // Serve only these demo assets, never arbitrary repository or dependency files.
    const file = files.get(pathname);
    if (!file) {
      response.writeHead(404).end('Not found');
      return;
    }
    fs.readFile(file.filename, (error, data) => {
      if (error) {
        console.error(`Unable to serve ${pathname}:`, error);
        response.writeHead(500).end('Unable to read demo asset');
        return;
      }
      response.writeHead(200, {
        'content-type': `${file.type}; charset=utf-8`,
        'cache-control': 'no-store',
        'x-content-type-options': 'nosniff',
      });
      response.end(request.method === 'HEAD' ? undefined : data);
    });
  });
}
