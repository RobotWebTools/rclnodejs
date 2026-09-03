// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

import assert from 'assert';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawn } from 'node:child_process';
import WebSocket from 'ws';
import { fileURLToPath } from 'url';

import {
  parseArgv,
  loadConfigFile,
  mergeConfig,
  validateConfig,
  CliError,
  DEFAULTS,
} from '../lib/runtime/cli-config.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));

// Reap a spawned child deterministically: SIGINT → wait → SIGKILL → wait,
// resolving only after the process has actually exited. Prevents a hung
// `rclnodejs-web` from leaking past the test (which would hold the rcl
// context and flap subsequent runs).
function reapChild(child, { graceMs = 2000 } = {}) {
  if (child.exitCode !== null) return Promise.resolve();
  return new Promise((resolve) => {
    const onExit = () => resolve();
    child.once('exit', onExit);
    try {
      child.kill('SIGINT');
    } catch (_) {
      // already dead between the check and the kill — onExit will fire.
    }
    setTimeout(() => {
      if (child.exitCode === null) {
        try {
          child.kill('SIGKILL');
        } catch (_) {
          // already dead — onExit will still fire
        }
      }
    }, graceMs);
  });
}

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
        action: {},
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
        '--action',
        '/fibonacci=example_interfaces/action/Fibonacci',
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
      assert.deepStrictEqual(partial.expose.action, {
        '/fibonacci': 'example_interfaces/action/Fibonacci',
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

    it('parses --http-sse / --http-sse-keep-alive', function () {
      const { partial } = parseArgv([
        '--http-sse',
        '--http-sse-keep-alive',
        '30000',
      ]);
      assert.strictEqual(partial.http.sse, true);
      assert.strictEqual(partial.http.sseKeepAliveMs, 30000);
    });

    it('parses --http-cors * as "any origin"', function () {
      const { partial } = parseArgv(['--http-cors', '*']);
      assert.strictEqual(partial.http.cors, true);
    });

    it('accumulates repeated --http-cors into an allow-list', function () {
      const { partial } = parseArgv([
        '--http-cors',
        'https://a.example',
        '--http-cors',
        'https://b.example',
      ]);
      assert.deepStrictEqual(partial.http.cors, [
        'https://a.example',
        'https://b.example',
      ]);
    });

    it('--http-cors * wins over specific origins regardless of order', function () {
      const { partial } = parseArgv([
        '--http-cors',
        'https://a.example',
        '--http-cors',
        '*',
        '--http-cors',
        'https://b.example',
      ]);
      assert.strictEqual(partial.http.cors, true);
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

    it('accepts http.sse / http.sseKeepAliveMs / http.cors', function () {
      assert.doesNotThrow(() =>
        validateConfig({
          http: { port: 9001, sse: true, sseKeepAliveMs: 0, cors: '*' },
        })
      );
      assert.doesNotThrow(() =>
        validateConfig({ http: { cors: ['https://a.example'] } })
      );
    });

    it('rejects non-boolean http.sse', function () {
      assert.throws(
        () => validateConfig({ http: { sse: 'yes' } }),
        (e) => e instanceof CliError && /http\.sse/.test(e.message)
      );
    });

    it('rejects non-number http.sseKeepAliveMs', function () {
      assert.throws(
        () => validateConfig({ http: { sseKeepAliveMs: 'soon' } }),
        (e) => e instanceof CliError && /http\.sseKeepAliveMs/.test(e.message)
      );
    });

    it('rejects http.cors of the wrong shape', function () {
      assert.throws(
        () => validateConfig({ http: { cors: 42 } }),
        (e) => e instanceof CliError && /http\.cors/.test(e.message)
      );
      assert.throws(
        () => validateConfig({ http: { cors: [1, 2] } }),
        (e) => e instanceof CliError && /http\.cors/.test(e.message)
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

    it('http: sse/cors default off, and merge from either source', function () {
      const base = mergeConfig({}, {});
      assert.strictEqual(base.http.sse, false);
      assert.strictEqual(base.http.cors, false);
      assert.strictEqual(base.http.sseKeepAliveMs, null);

      const merged = mergeConfig(
        { http: { port: 9001, sse: true, cors: '*' } },
        { http: { sseKeepAliveMs: 5000 } }
      );
      assert.strictEqual(merged.http.sse, true);
      assert.strictEqual(merged.http.cors, '*');
      assert.strictEqual(merged.http.sseKeepAliveMs, 5000);
    });

    it('validateConfig rejects a flag-only NaN keep-alive (bin flow)', function () {
      // Mirrors bin/rclnodejs-web.js: parse → merge → validate. A
      // non-numeric `--http-sse-keep-alive` becomes NaN in the parsed
      // partial; the post-merge validateConfig is what rejects it, since
      // loadConfigFile only validates JSON config files.
      const { partial } = parseArgv(['--http-sse-keep-alive', 'foo']);
      assert.ok(Number.isNaN(partial.http.sseKeepAliveMs));
      const merged = mergeConfig({}, partial);
      assert.throws(
        () => validateConfig(merged, 'options'),
        (e) => e instanceof CliError && /http\.sseKeepAliveMs/.test(e.message)
      );
    });
  });

  describe('end-to-end CLI launch', function () {
    let proc;
    let tmp;
    beforeEach(function () {
      tmp = fs.mkdtempSync(path.join(os.tmpdir(), 'rcl-cli-e2e-'));
    });
    afterEach(async function () {
      if (proc && proc.exitCode === null) {
        // Try a polite shutdown first; escalate to SIGKILL after a grace
        // period so a hung child can never leave a zombie that flaps the
        // next test (or holds the rcl context). Only resolve once the
        // process has *actually* exited.
        await reapChild(proc);
      }
      proc = undefined;
      fs.rmSync(tmp, { recursive: true, force: true });
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

    it('boots from a JSON config file passed positionally', function (done) {
      let finished = false;
      const finish = (err) => {
        if (finished) return;
        finished = true;
        if (proc && !proc.killed) proc.kill('SIGKILL');
        done(err);
      };

      // Write a complete web.json: WS + HTTP + an exposed call.
      // `port: 0` / `http.port: 0` exercise the ephemeral-binding path
      // through the config file (not just CLI flags).
      const cfgPath = path.join(tmp, 'web.json');
      fs.writeFileSync(
        cfgPath,
        JSON.stringify({
          port: 0,
          host: '127.0.0.1',
          path: '/capability',
          node: 'cli_jsoncfg_smoketest',
          http: { port: 0, host: '127.0.0.1' },
          expose: {
            call: { '/cfg_add': 'example_interfaces/srv/AddTwoInts' },
          },
        })
      );

      proc = spawn(
        process.execPath,
        [path.join(__dirname, '..', 'bin', 'rclnodejs-web.js'), cfgPath],
        { stdio: ['ignore', 'pipe', 'pipe'] }
      );
      let stderr = '';
      proc.stderr.on('data', (d) => (stderr += d.toString()));

      const giveUp = setTimeout(() => {
        finish(
          new Error(
            `timed out waiting for banner. stderr=${stderr}\nstdout=${banner}`
          )
        );
      }, 5000);

      let banner = '';
      proc.stdout.on('data', async (d) => {
        banner += d.toString();
        const wsM = /rclnodejs\/web listening on ws:\/\/[^:]+:(\d+)/.exec(
          banner
        );
        const httpM = /also http:\/\/[^:]+:(\d+)/.exec(banner);
        if (!wsM || !httpM) return;
        clearTimeout(giveUp);

        try {
          const httpPort = Number(httpM[1]);
          // The exposed capability from web.json must be reachable;
          // an unexposed one must 404. Together these prove the JSON
          // expose block was honoured (not just the CLI defaults).
          const exposed = await fetch(
            `http://127.0.0.1:${httpPort}/capability/call/cfg_add`,
            {
              method: 'POST',
              headers: { 'content-type': 'application/json' },
              body: JSON.stringify({ a: '1n', b: '2n' }),
              signal: AbortSignal.timeout(800),
            }
          ).catch((e) => ({ _aborted: e.name === 'TimeoutError' }));
          if (!exposed._aborted) {
            // No backend services /cfg_add, so a real reply is unlikely;
            // but if one came back, it must NOT be a not_exposed 404.
            assert.notStrictEqual(exposed.status, 404);
          }

          const unexposed = await fetch(
            `http://127.0.0.1:${httpPort}/capability/call/never_in_cfg`,
            {
              method: 'POST',
              headers: { 'content-type': 'application/json' },
              body: '{}',
            }
          );
          assert.strictEqual(unexposed.status, 404);
          const body = await unexposed.json();
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
