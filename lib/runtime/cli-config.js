// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Programmatic config loader for the `rclnodejs-web` CLI.
//
// Pure-data; does not import rclnodejs. Kept separate from bin/ so it
// can be unit-tested without spinning up a node.

'use strict';

const fs = require('node:fs');
const path = require('node:path');

const DEFAULTS = Object.freeze({
  port: 9000,
  // '::' = dual-stack: accepts both IPv6 and IPv4-mapped connections.
  // Matches Node's http server default and avoids browser "localhost"
  // mismatches on WSL2 / glibc systems where localhost resolves to
  // ::1 first. Pass --host 0.0.0.0 to bind IPv4 only.
  host: '::',
  path: '/capability',
  node: 'rclnodejs_web',
  // Optional HTTP transport for `call`/`publish`. Disabled by default;
  // set `port` (or pass `--http-port` on the CLI) to turn it on.
  // `host` defaults to the same dual-stack address as the WS listener.
  // `basePath` defaults to `/capability`.
  http: { port: null, host: null, basePath: null },
  expose: { call: {}, publish: {}, subscribe: {} },
});

const KINDS = ['call', 'publish', 'subscribe'];

/**
 * Parse command-line argv (without the leading `node`/script entries)
 * into a partial config object. Flags here override anything from a
 * config file later via {@link mergeConfig}.
 *
 * @param {string[]} argv
 * @returns {{configPath?: string, partial: object, help?: boolean, version?: boolean, quiet?: boolean}}
 */
function parseArgv(argv) {
  const partial = {
    expose: { call: {}, publish: {}, subscribe: {} },
    http: {},
  };
  const out = { partial };
  let i = 0;
  while (i < argv.length) {
    const a = argv[i];
    const eat = (name) => {
      const v = argv[i + 1];
      if (v === undefined || v.startsWith('-')) {
        throw new CliError(`flag ${name} requires a value`);
      }
      i += 2;
      return v;
    };
    if (a === '--help' || a === '-h') {
      out.help = true;
      i++;
    } else if (a === '--version' || a === '-V') {
      out.version = true;
      i++;
    } else if (a === '--quiet' || a === '-q') {
      out.quiet = true;
      i++;
    } else if (a === '--port' || a === '-p') {
      partial.port = Number(eat(a));
    } else if (a === '--host') {
      partial.host = eat(a);
    } else if (a === '--path') {
      partial.path = eat(a);
    } else if (a === '--node-name' || a === '--node') {
      partial.node = eat(a);
    } else if (a === '--http-port') {
      partial.http.port = Number(eat(a));
    } else if (a === '--http-host') {
      partial.http.host = eat(a);
    } else if (a === '--http-base-path') {
      partial.http.basePath = eat(a);
    } else if (a === '--call' || a === '--publish' || a === '--subscribe') {
      const kind = a.slice(2);
      const pair = eat(a);
      const eq = pair.indexOf('=');
      if (eq <= 0) {
        throw new CliError(
          `${a} expects <name>=<type> (got ${JSON.stringify(pair)})`
        );
      }
      partial.expose[kind][pair.slice(0, eq)] = pair.slice(eq + 1);
    } else if (a.startsWith('-')) {
      throw new CliError(`unknown flag: ${a}`);
    } else if (!out.configPath) {
      out.configPath = a;
      i++;
    } else {
      throw new CliError(`unexpected positional argument: ${a}`);
    }
  }
  return out;
}

/**
 * Load and validate a JSON config file. Returns an empty object if no
 * path is given.
 *
 * @param {string|undefined} configPath
 * @returns {object}
 */
function loadConfigFile(configPath) {
  if (!configPath) return {};
  const abs = path.resolve(configPath);
  let raw;
  try {
    raw = fs.readFileSync(abs, 'utf8');
  } catch (e) {
    throw new CliError(`cannot read config: ${abs}: ${e.message}`);
  }
  let cfg;
  try {
    cfg = JSON.parse(raw);
  } catch (e) {
    throw new CliError(`invalid JSON in ${abs}: ${e.message}`);
  }
  validateConfig(cfg, abs);
  return cfg;
}

/**
 * Shape-check a parsed config. Throws on bad shape with a precise
 * message; returns nothing on success.
 *
 * @param {*} cfg
 * @param {string} [origin]
 */
function validateConfig(cfg, origin = 'config') {
  if (cfg === null || typeof cfg !== 'object' || Array.isArray(cfg)) {
    throw new CliError(`${origin}: top-level value must be a JSON object`);
  }
  if (cfg.port !== undefined && !Number.isFinite(cfg.port)) {
    throw new CliError(`${origin}: "port" must be a number`);
  }
  if (cfg.host !== undefined && typeof cfg.host !== 'string') {
    throw new CliError(`${origin}: "host" must be a string`);
  }
  if (cfg.path !== undefined && typeof cfg.path !== 'string') {
    throw new CliError(`${origin}: "path" must be a string`);
  }
  if (cfg.node !== undefined && typeof cfg.node !== 'string') {
    throw new CliError(`${origin}: "node" must be a string`);
  }
  if (cfg.http !== undefined) {
    if (
      cfg.http === null ||
      typeof cfg.http !== 'object' ||
      Array.isArray(cfg.http)
    ) {
      throw new CliError(`${origin}: "http" must be an object`);
    }
    if (
      cfg.http.port !== undefined &&
      cfg.http.port !== null &&
      !Number.isFinite(cfg.http.port)
    ) {
      throw new CliError(`${origin}: "http.port" must be a number`);
    }
    if (
      cfg.http.host !== undefined &&
      cfg.http.host !== null &&
      typeof cfg.http.host !== 'string'
    ) {
      throw new CliError(`${origin}: "http.host" must be a string`);
    }
    if (
      cfg.http.basePath !== undefined &&
      cfg.http.basePath !== null &&
      typeof cfg.http.basePath !== 'string'
    ) {
      throw new CliError(`${origin}: "http.basePath" must be a string`);
    }
  }
  if (cfg.expose !== undefined) {
    if (
      cfg.expose === null ||
      typeof cfg.expose !== 'object' ||
      Array.isArray(cfg.expose)
    ) {
      throw new CliError(`${origin}: "expose" must be an object`);
    }
    for (const kind of Object.keys(cfg.expose)) {
      if (!KINDS.includes(kind)) {
        throw new CliError(
          `${origin}: "expose.${kind}" — unknown kind; allowed: ${KINDS.join(', ')}`
        );
      }
      const m = cfg.expose[kind];
      if (m === null || typeof m !== 'object' || Array.isArray(m)) {
        throw new CliError(`${origin}: "expose.${kind}" must be an object`);
      }
      for (const [name, value] of Object.entries(m)) {
        // Accept either the shorthand string form or the rich
        // `{ type: string, ... }` form. The rich form's extra keys are
        // reserved for forward-compatibility (e.g. QoS metadata in
        // 2.4.0); the validator only insists on a non-empty type.
        if (typeof value === 'string') {
          if (!value) {
            throw new CliError(
              `${origin}: expose.${kind}["${name}"] must be a non-empty string`
            );
          }
        } else if (
          value &&
          typeof value === 'object' &&
          !Array.isArray(value) &&
          typeof value.type === 'string' &&
          value.type
        ) {
          // ok — rich form with at least { type }
        } else {
          throw new CliError(
            `${origin}: expose.${kind}["${name}"] must be a non-empty string or { "type": "<pkg>/<kind>/<Name>" }`
          );
        }
      }
    }
  }
}

/**
 * Merge defaults ⊕ file config ⊕ CLI partial into a fully-resolved
 * config. Later sources win.
 */
function mergeConfig(...sources) {
  const out = JSON.parse(JSON.stringify(DEFAULTS));
  for (const src of sources) {
    if (!src) continue;
    for (const k of ['port', 'host', 'path', 'node']) {
      if (src[k] !== undefined) out[k] = src[k];
    }
    if (src.http) {
      for (const k of ['port', 'host', 'basePath']) {
        if (src.http[k] !== undefined && src.http[k] !== null) {
          out.http[k] = src.http[k];
        }
      }
    }
    if (src.expose) {
      for (const kind of KINDS) {
        if (src.expose[kind]) {
          Object.assign(out.expose[kind], src.expose[kind]);
        }
      }
    }
  }
  return out;
}

class CliError extends Error {
  constructor(message) {
    super(message);
    this.name = 'CliError';
    this.cli = true;
  }
}

const HELP = `Usage: rclnodejs-web [config.json] [options]

  rclnodejs/web — typed Web SDK and capability runtime for ROS 2.

  Start a runtime that exposes a declarative allow-list of ROS 2
  topics and services to browsers. Speaks both WebSocket (always
  on) and HTTP (opt-in via --http-port).

WebSocket transport (always on):
  -p, --port <n>              WS listen port           (default 9000)
      --host <addr>           WS bind host             (default ::, dual-stack)
      --path <url>            WS URL path              (default /capability)

HTTP transport (opt-in; for call/publish only):
      --http-port <n>         HTTP listen port         (disabled if omitted)
      --http-host <addr>      HTTP bind host           (default same as --host)
      --http-base-path <p>    HTTP base path           (default /capability)

Capabilities:
      --call <name>=<type>    expose a service capability   (repeatable)
      --publish <name>=<type> expose a topic publish        (repeatable)
      --subscribe <n>=<type>  expose a topic subscription   (repeatable)

Other:
      --node-name <name>      ROS 2 node name          (default rclnodejs_web)
  -q, --quiet                 don't print the startup banner
  -h, --help                  show this help
  -V, --version               show rclnodejs version

Examples:
  rclnodejs-web --port 9000 \\
    --call /add_two_ints=example_interfaces/srv/AddTwoInts \\
    --subscribe /scan=sensor_msgs/msg/LaserScan

  # Add HTTP for call/publish on :9001 (curl-able):
  rclnodejs-web --port 9000 --http-port 9001 \\
    --call /add_two_ints=example_interfaces/srv/AddTwoInts

  # Or all from a JSON config:
  rclnodejs-web web.json

Config-file shape (every field optional):
  {
    "port": 9000, "host": "::", "path": "/capability",
    "http": { "port": 9001 },
    "expose": {
      "call":      { "/add_two_ints": "example_interfaces/srv/AddTwoInts" },
      "publish":   { "/cmd_vel":      "geometry_msgs/msg/Twist" },
      "subscribe": { "/scan":         "sensor_msgs/msg/LaserScan" }
    }
  }

Notes:
  The startup banner ("rclnodejs/web listening on …") is human-readable
  only; its exact wording may change between minor versions. Scripts
  and CI jobs should run with --quiet and rely on the exit status or
  the explicit --port value rather than scraping the banner.
`;

module.exports = {
  DEFAULTS,
  KINDS,
  CliError,
  HELP,
  parseArgv,
  loadConfigFile,
  validateConfig,
  mergeConfig,
};
