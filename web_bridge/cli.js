#!/usr/bin/env node
// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

const rclnodejs = require('../index.js');
const { startWebBridge } = require('./bridge.js');

const USAGE = `Usage: rclnodejs-web-bridge [options]

Expose ROS 2 topics and services as resource-style WebSocket URLs.

Options:
  -p, --port <port>             Port to listen on (default: 9000)
  -H, --host <host>             Host/interface to bind (default: 0.0.0.0)
  -n, --node-name <name>        ROS 2 node name (default: web_bridge)
  -t, --topic <name>:<type>     Pre-declare a topic type (repeatable)
                                e.g. --topic /chatter:std_msgs/msg/String
  -s, --service <name>:<type>   Pre-declare a service type (repeatable)
                                e.g. --service /add:example_interfaces/srv/AddTwoInts
  -h, --help                    Show this help

URL scheme:
  ws://host:port/topic/<name>?type=<pkg>/msg/<Type>
  ws://host:port/service/<name>?type=<pkg>/srv/<Type>

Pre-declared types via --topic/--service let clients omit the ?type= query.
`;

function parseArgs(argv) {
  const opts = {
    port: 9000,
    host: '0.0.0.0',
    nodeName: 'web_bridge',
    topicTypes: {},
    serviceTypes: {},
  };
  const need = (i, flag) => {
    if (i + 1 >= argv.length) {
      console.error(`error: ${flag} requires a value`);
      process.exit(2);
    }
    return argv[i + 1];
  };
  const addPair = (target, raw, flag) => {
    const idx = raw.indexOf(':');
    if (idx <= 0) {
      console.error(`error: ${flag} expects <name>:<type>, got "${raw}"`);
      process.exit(2);
    }
    let name = raw.slice(0, idx);
    const type = raw.slice(idx + 1);
    if (!name.startsWith('/')) name = '/' + name;
    target[name] = type;
  };

  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    switch (a) {
      case '-h':
      case '--help':
        console.log(USAGE);
        process.exit(0);
        break;
      case '-p':
      case '--port': {
        const raw = need(i, a);
        const p = Number(raw);
        if (!Number.isInteger(p) || p < 0 || p > 65535) {
          console.error(
            `error: ${a} expects an integer in 0–65535, got "${raw}"`
          );
          process.exit(2);
        }
        opts.port = p;
        i++;
        break;
      }
      case '-H':
      case '--host':
        opts.host = need(i, a);
        i++;
        break;
      case '-n':
      case '--node-name':
        opts.nodeName = need(i, a);
        i++;
        break;
      case '-t':
      case '--topic':
        addPair(opts.topicTypes, need(i, a), a);
        i++;
        break;
      case '-s':
      case '--service':
        addPair(opts.serviceTypes, need(i, a), a);
        i++;
        break;
      default:
        console.error(`error: unknown argument: ${a}`);
        console.error(USAGE);
        process.exit(2);
    }
  }
  return opts;
}

async function main() {
  const opts = parseArgs(process.argv.slice(2));

  await rclnodejs.init();
  const node = rclnodejs.createNode(opts.nodeName);
  rclnodejs.spin(node);

  const bridge = await startWebBridge({
    node,
    port: opts.port,
    host: opts.host,
    topicTypes: opts.topicTypes,
    serviceTypes: opts.serviceTypes,
  });

  // 0.0.0.0 / :: are bind wildcards, not reachable URLs. Show a usable
  // hostname in the log so users can paste the URL directly into a browser.
  const displayHost =
    opts.host === '0.0.0.0' || opts.host === '::' || opts.host === ''
      ? '127.0.0.1'
      : opts.host;
  console.log(
    `[rclnodejs-web-bridge] node="${opts.nodeName}" listening on ws://${displayHost}:${bridge.port} (bind=${opts.host})`
  );
  for (const [name, type] of Object.entries(opts.topicTypes)) {
    console.log(`  topic   ${name}\t-> ${type}`);
  }
  for (const [name, type] of Object.entries(opts.serviceTypes)) {
    console.log(`  service ${name}\t-> ${type}`);
  }

  const shutdown = (sig) => {
    console.log(`[rclnodejs-web-bridge] received ${sig}, shutting down`);
    // Hard-exit fallback in case ws/rcl close callbacks don't fire
    // (e.g. due to in-flight rclnodejs.spin loop keeping the event loop busy).
    const hard = setTimeout(() => process.exit(0), 1500);
    hard.unref();
    Promise.resolve()
      .then(() => bridge.close())
      .catch(() => {})
      .then(() => {
        try {
          rclnodejs.shutdown();
        } catch (_) {}
        process.exit(0);
      });
  };
  process.on('SIGINT', () => shutdown('SIGINT'));
  process.on('SIGTERM', () => shutdown('SIGTERM'));
}

main().catch((e) => {
  console.error(e.stack || e.message);
  process.exit(1);
});
