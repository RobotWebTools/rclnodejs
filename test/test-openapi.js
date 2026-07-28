// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

// Tests for lib/openapi.js: the OpenAPI export built on top of the
// capability registry. These are pure metadata transforms — no ROS 2
// init, no transports, no network — so they run fast and need no
// `rclnodejs.init()`.

import assert from 'assert';
import path from 'node:path';
import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { buildOpenApiDocument, primitiveToJsonSchema } from '../lib/openapi.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const CLI_PATH = path.join(__dirname, '..', 'bin', 'rclnodejs-web.js');

function runCli(args) {
  return new Promise((resolve, reject) => {
    const p = spawn(process.execPath, [CLI_PATH, ...args], {
      stdio: ['ignore', 'pipe', 'pipe'],
    });
    let stdout = '';
    let stderr = '';
    p.stdout.on('data', (d) => (stdout += d.toString()));
    p.stderr.on('data', (d) => (stderr += d.toString()));
    p.on('close', (code) => resolve({ code, stdout, stderr }));
    p.on('error', reject);
  });
}

describe('lib/openapi.js', function () {
  describe('primitiveToJsonSchema', function () {
    it('maps bool to boolean', function () {
      assert.deepStrictEqual(primitiveToJsonSchema({ type: 'bool' }), {
        type: 'boolean',
      });
    });

    it('maps int32 to a plain integer', function () {
      assert.deepStrictEqual(primitiveToJsonSchema({ type: 'int32' }), {
        type: 'integer',
      });
    });

    it('maps float64 to number', function () {
      assert.deepStrictEqual(primitiveToJsonSchema({ type: 'float64' }), {
        type: 'number',
      });
    });

    it('maps int64/uint64 to a BigInt-string, not a bare integer', function () {
      const schema = primitiveToJsonSchema({ type: 'int64' });
      assert.strictEqual(schema.type, 'string');
      assert.strictEqual(schema.format, 'int64');
      const re = new RegExp(schema.pattern);
      assert.ok(re.test('42n'));
      assert.ok(re.test('-7n'));
      assert.ok(!re.test('42'));
      // Explicit example so API explorers (e.g. Swagger UI) show a sane
      // value instead of synthesizing an arbitrarily long digit run to
      // satisfy the unbounded `[0-9]+` pattern.
      assert.ok(re.test(schema.example));

      const unsignedSchema = primitiveToJsonSchema({ type: 'uint64' });
      assert.strictEqual(unsignedSchema.type, 'string');
      assert.strictEqual(unsignedSchema.format, 'uint64');
      const unsignedRe = new RegExp(unsignedSchema.pattern);
      assert.ok(unsignedRe.test('42n'));
      assert.ok(!unsignedRe.test('-7n'));
      assert.ok(unsignedRe.test(unsignedSchema.example));
    });

    it('maps bounded strings to maxLength', function () {
      const schema = primitiveToJsonSchema({
        type: 'string',
        stringUpperBound: 32,
      });
      assert.strictEqual(schema.type, 'string');
      assert.strictEqual(schema.maxLength, 32);
    });

    it('leaves unbounded strings without maxLength', function () {
      const schema = primitiveToJsonSchema({ type: 'string' });
      assert.strictEqual(schema.type, 'string');
      assert.strictEqual(schema.maxLength, undefined);
    });
  });

  describe('buildOpenApiDocument', function () {
    const capabilities = {
      call: { '/add_two_ints': 'example_interfaces/srv/AddTwoInts' },
      publish: { '/chatter': 'std_msgs/msg/String' },
      subscribe: { '/cmd_vel': 'geometry_msgs/msg/Twist' },
    };

    it('produces a valid-looking OpenAPI 3.1 document shell', function () {
      const doc = buildOpenApiDocument(capabilities, { title: 'test API' });
      assert.strictEqual(doc.openapi, '3.1.0');
      assert.strictEqual(doc.info.title, 'test API');
      assert.strictEqual(doc.info.version, '0.0.0');
    });

    it('documents a call capability as POST /capability/call/<name>', function () {
      const doc = buildOpenApiDocument(capabilities);
      const op = doc.paths['/capability/call/add_two_ints'].post;
      assert.strictEqual(op.operationId, 'call_add_two_ints');
      assert.strictEqual(
        op['x-ros-capability'].type,
        'example_interfaces/srv/AddTwoInts'
      );
      const reqSchema = op.requestBody.content['application/json'].schema;
      assert.deepStrictEqual(Object.keys(reqSchema.properties).sort(), [
        'a',
        'b',
      ]);
      const resSchema = op.responses['200'].content['application/json'].schema;
      assert.deepStrictEqual(Object.keys(resSchema.properties), ['sum']);
      // AddTwoInts fields are int64 — must use the BigInt-string schema.
      assert.strictEqual(reqSchema.properties.a.type, 'string');
      assert.strictEqual(reqSchema.properties.a.format, 'int64');
    });

    it('documents a publish capability as POST /capability/publish/<name> returning 204', function () {
      const doc = buildOpenApiDocument(capabilities);
      const op = doc.paths['/capability/publish/chatter'].post;
      assert.strictEqual(op.operationId, 'publish_chatter');
      assert.ok(op.responses['204']);
      const schema = op.requestBody.content['application/json'].schema;
      assert.deepStrictEqual(Object.keys(schema.properties), ['data']);
    });

    it('documents a subscribe capability as GET /capability/subscribe/<name> over SSE', function () {
      const doc = buildOpenApiDocument(capabilities);
      const op = doc.paths['/capability/subscribe/cmd_vel'].get;
      assert.strictEqual(op.operationId, 'subscribe_cmd_vel');
      const sseContent = op.responses['200'].content['text/event-stream'];
      assert.ok(sseContent, 'expected a text/event-stream response');
    });

    it("subscribe's 404 documents both unsupported_kind (sse off, the default) and not_exposed", function () {
      const doc = buildOpenApiDocument(capabilities);
      const op = doc.paths['/capability/subscribe/cmd_vel'].get;
      const codeSchema =
        op.responses['404'].content['application/json'].schema.properties.code;
      assert.deepStrictEqual(codeSchema.enum, [
        'not_exposed',
        'unsupported_kind',
      ]);
    });

    it('normalizes a trailing-slash basePath, matching HttpTransport, instead of emitting a double slash', function () {
      const doc = buildOpenApiDocument(capabilities, { basePath: '/api/' });
      assert.ok(doc.paths['/api/call/add_two_ints']);
      assert.ok(!('/api//call/add_two_ints' in doc.paths));
    });

    it('de-duplicates nested message types into one shared $ref component', function () {
      const doc = buildOpenApiDocument(capabilities);
      const op = doc.paths['/capability/subscribe/cmd_vel'].get;
      const schema = op.responses['200'].content['text/event-stream'].schema;
      // Twist has `linear` and `angular`, both Vector3 — both should point
      // at the same component instead of being inlined twice.
      assert.strictEqual(
        schema.properties.linear.$ref,
        schema.properties.angular.$ref
      );
      const refName = schema.properties.linear.$ref.split('/').pop();
      assert.ok(
        doc.components.schemas[refName],
        'Vector3 component should be registered'
      );
      assert.deepStrictEqual(
        Object.keys(doc.components.schemas[refName].properties).sort(),
        ['x', 'y', 'z']
      );
    });
  });

  describe('CLI subcommand (bin/rclnodejs-web.js openapi)', function () {
    this.timeout(10000);

    it('openapi prints a document from --call/--publish flags, no ROS init needed', async function () {
      const { code, stdout } = await runCli([
        'openapi',
        '--call',
        '/add_two_ints=example_interfaces/srv/AddTwoInts',
      ]);
      assert.strictEqual(code, 0);
      const doc = JSON.parse(stdout);
      assert.strictEqual(doc.openapi, '3.1.0');
      assert.ok(doc.paths['/capability/call/add_two_ints']);
    });

    it('openapi routes use --path when http.basePath is not set, matching the server transport', async function () {
      // Must match HttpTransport's own fallback (cfg.path, not a hardcoded
      // '/capability') or the document describes routes the server doesn't
      // actually serve.
      const { code, stdout } = await runCli([
        'openapi',
        '--path',
        '/api',
        '--call',
        '/add_two_ints=example_interfaces/srv/AddTwoInts',
      ]);
      assert.strictEqual(code, 0);
      const doc = JSON.parse(stdout);
      assert.ok(doc.paths['/api/call/add_two_ints']);
      assert.ok(!doc.paths['/capability/call/add_two_ints']);
    });

    it('openapi omits servers when the HTTP transport is off', async function () {
      const { code, stdout } = await runCli([
        'openapi',
        '--publish',
        '/chatter=std_msgs/msg/String',
      ]);
      assert.strictEqual(code, 0);
      assert.ok(!('servers' in JSON.parse(stdout)));
    });

    it('openapi points servers at the HTTP transport port when enabled', async function () {
      const { code, stdout } = await runCli([
        'openapi',
        '--publish',
        '/chatter=std_msgs/msg/String',
        '--http-port',
        '9001',
      ]);
      assert.strictEqual(code, 0);
      const doc = JSON.parse(stdout);
      assert.strictEqual(doc.servers.length, 1);
      assert.strictEqual(doc.servers[0].url, 'http://localhost:9001');
    });

    it('openapi uses a configured --http-host verbatim, not a hardcoded localhost', async function () {
      const { code, stdout } = await runCli([
        'openapi',
        '--publish',
        '/chatter=std_msgs/msg/String',
        '--http-port',
        '9001',
        '--http-host',
        'api.example.com',
      ]);
      assert.strictEqual(code, 0);
      const doc = JSON.parse(stdout);
      assert.strictEqual(doc.servers[0].url, 'http://api.example.com:9001');
    });

    it('openapi displays a wildcard --http-host as localhost, matching the startup banner', async function () {
      const { code, stdout } = await runCli([
        'openapi',
        '--publish',
        '/chatter=std_msgs/msg/String',
        '--http-port',
        '9001',
        '--http-host',
        '0.0.0.0',
      ]);
      assert.strictEqual(code, 0);
      const doc = JSON.parse(stdout);
      assert.strictEqual(doc.servers[0].url, 'http://localhost:9001');
    });

    it("openapi leaves info.version at the '0.0.0' placeholder", async function () {
      // info.version describes the user's API, not the tool that generated
      // it, and buildOpenApiDocument() has no option for it (see its
      // docstring) — so it's always this fixed placeholder.
      const { code, stdout } = await runCli([
        'openapi',
        '--publish',
        '/chatter=std_msgs/msg/String',
      ]);
      assert.strictEqual(code, 0);
      const doc = JSON.parse(stdout);
      assert.strictEqual(doc.info.version, '0.0.0');
    });

    it('rejects an unknown flag the same way the server-start path does', async function () {
      const { code, stderr } = await runCli(['openapi', '--nope']);
      assert.notStrictEqual(code, 0);
      assert.match(stderr, /unknown flag/);
    });

    it('--help documents the subcommand', async function () {
      const { code, stdout } = await runCli(['--help']);
      assert.strictEqual(code, 0);
      assert.match(stdout, /rclnodejs-web openapi/);
    });
  });
});
