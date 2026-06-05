#!/usr/bin/env node
// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// `rclnodejs-web` — the rclnodejs Web launcher.
//
// For frontend developers who don't want to write a Node.js server.
// Run `rclnodejs-web --help` or `npx rclnodejs-web web.json` to start
// demo. See demo/web/javascript/README.md for the full demo.

import path from 'node:path';
import fs from 'node:fs';
import { fileURLToPath } from 'node:url';

import {
  parseArgv,
  loadConfigFile,
  mergeConfig,
  HELP,
} from '../lib/runtime/cli-config.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

const argv = process.argv.slice(2);

(async function main() {
  let parsed;
  try {
    parsed = parseArgv(argv);
  } catch (e) {
    fail(e);
  }

  if (parsed.help) {
    process.stdout.write(HELP);
    process.exit(0);
  }
  if (parsed.version) {
    const pkg = JSON.parse(
      fs.readFileSync(path.join(__dirname, '..', 'package.json'), 'utf8')
    );
    process.stdout.write(`rclnodejs ${pkg.version}\n`);
    process.exit(0);
  }

  let cfg;
  try {
    cfg = mergeConfig(loadConfigFile(parsed.configPath), parsed.partial);
  } catch (e) {
    fail(e);
  }

  // Defer the rclnodejs import until after argv parsing so --help / --version
  // never pay the (slow, native) load cost.
  const rclnodejs = (await import('../index.js')).default;
  const { createRuntime, WebSocketTransport, HttpTransport } =
    await import('../lib/runtime/index.js');

  // Track partial init so the catch block can clean up native handles before
  // process.exit() — without this, a startup failure (e.g. EADDRINUSE on the
  // WS port) leaves rclnodejs's native spin loop running and segfaults on exit.
  let rclInitialized = false;
  let runtime = null;

  try {
    await rclnodejs.init();
    rclInitialized = true;

    const node = rclnodejs.createNode(cfg.node);
    rclnodejs.spin(node);

    // Always start the WebSocket transport. Add HTTP only when the user
    // configured an http.port (via --http-port or in the config file).
    const transports = [
      new WebSocketTransport({
        port: cfg.port,
        host: cfg.host,
        path: cfg.path,
      }),
    ];
    const httpEnabled =
      cfg.http && cfg.http.port !== null && cfg.http.port !== undefined;
    if (httpEnabled) {
      transports.push(
        new HttpTransport({
          port: cfg.http.port,
          host: cfg.http.host || cfg.host,
          basePath: cfg.http.basePath || cfg.path,
        })
      );
    }
    runtime = createRuntime({ node, transports });
    runtime.expose(cfg.expose);
    await runtime.start();

    // After start(), each transport reports the actual bound port
    // (matters for `--port 0` / `--http-port 0` ephemeral modes).
    const wsTransport = runtime.transports[0];
    const httpTransport = httpEnabled ? runtime.transports[1] : null;

    if (!parsed.quiet) {
      const displayHost = (h) =>
        ['0.0.0.0', '::'].includes(h) ? 'localhost' : h;
      const list = runtime.registry.list();
      const totals =
        Object.keys(list.call).length +
        Object.keys(list.publish).length +
        Object.keys(list.subscribe).length;
      const noun = totals === 1 ? 'capability' : 'capabilities';
      process.stdout.write(
        `rclnodejs/web listening on ws://${displayHost(cfg.host)}:${wsTransport.port}${cfg.path} (${totals} ${noun})\n`
      );
      if (httpTransport) {
        const httpHost = displayHost(cfg.http.host || cfg.host);
        const httpBase = cfg.http.basePath || cfg.path;
        process.stdout.write(
          `                  also http://${httpHost}:${httpTransport.port}${httpBase} (call/publish only)\n`
        );
      }
    }

    const stop = async () => {
      if (!parsed.quiet) process.stdout.write('\nstopping…\n');
      await runtime.stop();
      rclnodejs.shutdown();
      process.exit(0);
    };
    process.once('SIGINT', stop);
    process.once('SIGTERM', stop);
  } catch (err) {
    // Best-effort native cleanup so rclnodejs doesn't segfault on dirty exit.
    if (runtime) {
      try {
        await runtime.stop();
      } catch (_) {
        /* ignore — we're already failing */
      }
    }
    if (rclInitialized) {
      try {
        rclnodejs.shutdown();
      } catch (_) {
        /* ignore — we're already failing */
      }
    }
    fail(err);
  }
})();

function fail(err) {
  if (err && err.cli) {
    process.stderr.write(`error: ${err.message}\n\n${HELP}`);
    process.exit(2);
  }
  process.stderr.write((err && err.stack) || String(err));
  process.stderr.write('\n');
  process.exit(1);
}
