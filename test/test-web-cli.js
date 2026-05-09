// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

'use strict';

const assert = require('assert');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const { spawn } = require('node:child_process');
const WebSocket = require('ws');

const {
  parseArgv,
  loadConfigFile,
  mergeConfig,
  validateConfig,
  CliError,
  DEFAULTS,
} = require('../lib/runtime/cli-config.js');

describe('rclnodejs-web CLI', function () {
  this.timeout(60 * 1000);

  describe('parseArgv (unit)', function () {
    it('returns defaults for empty argv', function () {
      const out = parseArgv([]);
      assert.strictEqual(out.configPath, undefined);
      assert.deepStrictEqual(out.partial.expose, {
        call: {},
        publish: {},
        subscribe: {},
      });
    });

    it('parses scalar flags', function () {
      const { partial } = parseArgv([
        '--port',
        '9100',
        '--host',
        '127.0.0.1',
        '--path',
        '/cap',
        '--node-name',
        'demo',
      ]);
      assert.strictEqual(partial.port, 9100);
      assert.strictEqual(partial.host, '127.0.0.1');
      assert.strictEqual(partial.path, '/cap');
      assert.strictEqual(partial.node, 'demo');
    });

    it('collects repeated capability flags into expose maps', function () {
      const { partial } = parseArgv([
        '--call',
        '/add=example_interfaces/srv/AddTwoInts',
        '--publish',
        '/cmd_vel=geometry_msgs/msg/Twist',
        '--subscribe',
        '/scan=sensor_msgs/msg/LaserScan',
      ]);
      assert.deepStrictEqual(partial.expose.call, {
        '/add': 'example_interfaces/srv/AddTwoInts',
      });
      assert.deepStrictEqual(partial.expose.publish, {
        '/cmd_vel': 'geometry_msgs/msg/Twist',
      });
      assert.deepStrictEqual(partial.expose.subscribe, {
        '/scan': 'sensor_msgs/msg/LaserScan',
      });
    });

    it('captures the first positional as configPath', function () {
      const { configPath } = parseArgv(['my-config.json', '--port', '9000']);
      assert.strictEqual(configPath, 'my-config.json');
    });

    it('rejects bad capability syntax', function () {
      assert.throws(
        () => parseArgv(['--call', 'no-equals-sign']),
        (e) => e instanceof CliError
      );
    });

    it('rejects unknown flags', function () {
      assert.throws(
        () => parseArgv(['--bogus']),
        (e) => e instanceof CliError && /unknown flag/.test(e.message)
      );
    });

    it('surfaces --help / --version / --quiet', function () {
      assert.strictEqual(parseArgv(['--help']).help, true);
      assert.strictEqual(parseArgv(['-h']).help, true);
      assert.strictEqual(parseArgv(['--version']).version, true);
      assert.strictEqual(parseArgv(['--quiet']).quiet, true);
    });

    it('parses --http-port / --http-host / --http-base-path', function () {
      const { partial } = parseArgv([
        '--http-port',
        '9101',
        '--http-host',
        '127.0.0.1',
        '--http-base-path',
        '/cap',
      ]);
      assert.strictEqual(partial.http.port, 9101);
      assert.strictEqual(partial.http.host, '127.0.0.1');
      assert.strictEqual(partial.http.basePath, '/cap');
    });
  });

  describe('config file loading + validation', function () {
    let tmp;
    beforeEach(function () {
      tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'rcl-cli-'));
    });
    afterEach(function () {
      fs.rmSync(tmp, { recursive: true, force: true });
    });

    function writeJson(name, body) {
      const p = path.join(tmp, name);
      fs.writeFileSync(p, JSON.stringify(body));
      return p;
    }

    it('loads a well-formed config', function () {
      const p = writeJson('cfg.json', {
        port: 9123,
        expose: { call: { '/x': 'std_srvs/srv/Empty' } },
      });
      const cfg = loadConfigFile(p);
      assert.strictEqual(cfg.port, 9123);
      assert.strictEqual(cfg.expose.call['/x'], 'std_srvs/srv/Empty');
    });

    it('returns {} when path is undefined', function () {
      assert.deepStrictEqual(loadConfigFile(undefined), {});
    });

    it('rejects missing files', function () {
      assert.throws(
        () => loadConfigFile(path.join(tmp, 'nope.json')),
        (e) => e instanceof CliError && /cannot read/.test(e.message)
      );
    });

    it('rejects invalid JSON', function () {
      const p = path.join(tmp, 'bad.json');
      fs.writeFileSync(p, '{not json');
      assert.throws(
        () => loadConfigFile(p),
        (e) => e instanceof CliError && /invalid JSON/.test(e.message)
      );
    });

    it('rejects unknown expose kind', function () {
      assert.throws(
        () => validateConfig({ expose: { weird: {} } }),
        (e) => e instanceof CliError && /unknown kind/.test(e.message)
      );
    });

    it('rejects non-string capability type', function () {
      assert.throws(
        () => validateConfig({ expose: { call: { '/x': 42 } } }),
        (e) =>
          e instanceof CliError && /must be a non-empty string/.test(e.message)
      );
    });

    it('accepts a well-formed http block', function () {
      assert.doesNotThrow(() =>
        validateConfig({ http: { port: 9001, host: '::', basePath: '/cap' } })
      );
    });

    it('rejects http with non-number port', function () {
      assert.throws(
        () => validateConfig({ http: { port: 'nine' } }),
        (e) => e instanceof CliError && /http\.port/.test(e.message)
      );
    });

    it('rejects http as an array', function () {
      assert.throws(
        () => validateConfig({ http: [9001] }),
        (e) =>
          e instanceof CliError && /"http" must be an object/.test(e.message)
      );
    });
  });

  describe('mergeConfig', function () {
    it('CLI partial overrides file config', function () {
      const merged = mergeConfig(
        { port: 1, host: '1.1.1.1' },
        { port: 2 } // CLI partial
      );
      assert.strictEqual(merged.port, 2);
      assert.strictEqual(merged.host, '1.1.1.1');
    });

    it('expose maps are merged additively across sources', function () {
      const merged = mergeConfig(
        { expose: { call: { '/a': 'std_srvs/srv/Empty' } } },
        { expose: { call: { '/b': 'std_srvs/srv/Trigger' } } }
      );
      assert.deepStrictEqual(merged.expose.call, {
        '/a': 'std_srvs/srv/Empty',
        '/b': 'std_srvs/srv/Trigger',
      });
    });

    it('falls back to DEFAULTS when no source supplies a key', function () {
      const merged = mergeConfig({}, {});
      assert.strictEqual(merged.port, DEFAULTS.port);
      assert.strictEqual(merged.host, DEFAULTS.host);
      assert.strictEqual(merged.path, DEFAULTS.path);
      assert.strictEqual(merged.node, DEFAULTS.node);
    });

    it('http: CLI partial overrides file http block', function () {
      const merged = mergeConfig(
        { http: { port: 9101, host: '1.1.1.1' } },
        { http: { port: 9202 } }
      );
      assert.strictEqual(merged.http.port, 9202);
      assert.strictEqual(merged.http.host, '1.1.1.1');
    });

    it('http: omitting it leaves the transport disabled (port=null)', function () {
      const merged = mergeConfig({}, {});
      assert.strictEqual(merged.http.port, null);
    });
  });

  describe('end-to-end CLI launch', function () {
    let proc;
    afterEach(async function () {
      if (proc && !proc.killed) {
        proc.kill('SIGINT');
        await new Promise((res) => {
          proc.once('exit', res);
          setTimeout(res, 2000);
        });
      }
      proc = undefined;
    });

    it('--help exits 0 and prints usage without loading rclnodejs', function (done) {
      const p = spawn(
        process.execPath,
        [path.join(__dirname, '..', 'bin', 'rclnodejs-web.js'), '--help'],
        { stdio: ['ignore', 'pipe', 'pipe'] }
      );
      let out = '';
      p.stdout.on('data', (d) => (out += d.toString()));
      p.on('exit', (code) => {
        try {
          assert.strictEqual(code, 0);
          assert.match(out, /Usage: rclnodejs-web/);
          done();
        } catch (e) {
          done(e);
        }
      });
    });

    it('starts a runtime from CLI flags and accepts a service call', function (done) {
      let finished = false;
      const finish = (err) => {
        if (finished) return;
        finished = true;
        done(err);
      };

      proc = spawn(
        process.execPath,
        [
          path.join(__dirname, '..', 'bin', 'rclnodejs-web.js'),
          '--port',
          '0', // ephemeral
          '--host',
          '127.0.0.1',
          '--node-name',
          'cli_smoketest',
          '--call',
          '/cli_add=example_interfaces/srv/AddTwoInts',
        ],
        { stdio: ['ignore', 'pipe', 'pipe'] }
      );
      let stderr = '';
      proc.stderr.on('data', (d) => (stderr += d.toString()));

      let banner = '';
      let dialed = false;
      proc.stdout.on('data', (d) => {
        banner += d.toString();
        if (dialed) return;
        const m = /ws:\/\/[^:]+:(\d+)\/capability/.exec(banner);
        if (!m) return;
        dialed = true;
        const port = Number(m[1]);
        const ws = new WebSocket(`ws://127.0.0.1:${port}/capability`);
        ws.once('open', () => {
          ws.send(
            JSON.stringify({
              id: 'probe',
              kind: 'call',
              capability: '/cli_add',
              payload: { a: '1n', b: '2n' },
            })
          );
        });
        // Setting up a service client succeeds even with no service
        // backend; observing that the runtime accepted the frame (no
        // immediate `not_exposed` error) is enough to prove the CLI
        // booted with the right registry. Time-bounded so we don't
        // hang waiting for a reply that no one will send.
        const settle = setTimeout(() => {
          try {
            ws.close();
          } catch (_) {}
          finish();
        }, 1500);
        ws.on('message', (raw) => {
          let frame;
          try {
            frame = JSON.parse(raw.toString());
          } catch (e) {
            clearTimeout(settle);
            return finish(e);
          }
          if (frame.code === 'not_exposed') {
            clearTimeout(settle);
            return finish(
              new Error('runtime did not register --call /cli_add')
            );
          }
          // Any other frame is acceptable (success reply, or none at
          // all because nothing services /cli_add).
        });
        ws.once('error', (err) => {
          clearTimeout(settle);
          finish(new Error(`ws error: ${err.message}\nstderr=${stderr}`));
        });
      });

      proc.on('exit', (code) => {
        if (code !== null && code !== 0) {
          finish(
            new Error(`CLI exited ${code}\nstderr=${stderr}\nstdout=${banner}`)
          );
        }
      });
    });

    it('--http-port stands up the HTTP transport alongside WS', function (done) {
      let finished = false;
      const finish = (err) => {
        if (finished) return;
        finished = true;
        // Make sure the spawned process dies so mocha can exit.
        if (proc && !proc.killed) proc.kill('SIGKILL');
        done(err);
      };

      proc = spawn(
        process.execPath,
        [
          path.join(__dirname, '..', 'bin', 'rclnodejs-web.js'),
          '--port',
          '0',
          '--host',
          '127.0.0.1',
          '--http-port',
          '0',
          '--http-host',
          '127.0.0.1',
          '--node-name',
          'cli_http_smoketest',
          '--call',
          '/cli_http_add=example_interfaces/srv/AddTwoInts',
        ],
        { stdio: ['ignore', 'pipe', 'pipe'] }
      );
      let stderr = '';
      proc.stderr.on('data', (d) => (stderr += d.toString()));

      // Bound the wait so a hung process can't keep mocha running.
      const giveUp = setTimeout(() => {
        finish(
          new Error(
            `timed out waiting for HTTP banner. stderr=${stderr}\nstdout=${banner}`
          )
        );
      }, 5000);

      let banner = '';
      proc.stdout.on('data', async (d) => {
        banner += d.toString();
        // Banner format we set in bin/rclnodejs-web.js when http is enabled:
        //   rclnodejs/web listening on ws://localhost:<wsPort>/capability (...)
        //                  also http://localhost:<httpPort>/capability (call/publish only)
        const wsM = /rclnodejs\/web listening on ws:\/\/[^:]+:(\d+)/.exec(
          banner
        );
        const httpM = /also http:\/\/[^:]+:(\d+)/.exec(banner);
        if (!wsM || !httpM) return;
        clearTimeout(giveUp);

        // Ports should differ from each other and both be > 0
        // (proving --http-port 0 was honoured for ephemeral binding).
        try {
          const wsPort = Number(wsM[1]);
          const httpPort = Number(httpM[1]);
          assert.ok(wsPort > 0, `expected ws port > 0, got ${wsPort}`);
          assert.ok(httpPort > 0, `expected http port > 0, got ${httpPort}`);
          assert.notStrictEqual(wsPort, httpPort);

          // Hit the HTTP listener with a deliberately unexposed
          // capability so we get a fast 404 / not_exposed reply
          // (no waiting for a service backend).
          const res = await fetch(
            `http://127.0.0.1:${httpPort}/capability/call/never_exposed`,
            {
              method: 'POST',
              headers: { 'content-type': 'application/json' },
              body: '{}',
            }
          );
          assert.strictEqual(res.status, 404);
          const body = await res.json();
          assert.strictEqual(body.code, 'not_exposed');
          finish();
        } catch (e) {
          finish(e);
        }
      });

      proc.on('exit', (code) => {
        if (!finished && code !== null && code !== 0) {
          clearTimeout(giveUp);
          finish(
            new Error(`CLI exited ${code}\nstderr=${stderr}\nstdout=${banner}`)
          );
        }
      });
    });
  });
});
