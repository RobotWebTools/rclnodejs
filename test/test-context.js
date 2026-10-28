// Copyright (c) 2017 Intel Corporation. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import assert from 'assert';
import { execFile } from 'node:child_process';
import rclnodejs from '../index.js';

describe('context test suite', function () {
  this.timeout(60 * 1000);

  it('context constructor', function (done) {
    let context = new rclnodejs.Context();
    assert.ok(context.isUninitialized());
    assert.ok(!context.isInitialized());
    assert.ok(!context.isShutdown());
    done();
  });

  it('context defaultContext', function (done) {
    let context1 = rclnodejs.Context.defaultContext();
    assert.ok(context1.isDefaultContext());

    let context2 = rclnodejs.Context.defaultContext();
    assert.ok(context2.isDefaultContext());
    assert.strictEqual(context1, context2);

    let context3 = new rclnodejs.Context();
    assert.ok(!context3.isDefaultContext());

    context1.shutdown();
    context3.shutdown();

    let context4 = rclnodejs.Context.defaultContext();
    assert.ok(context4.isDefaultContext());

    context4.shutdown();
    done();
  });

  it('context initialized', async function () {
    let context = new rclnodejs.Context();
    await rclnodejs.init(context);
    assert.ok(!context.isUninitialized());
    assert.ok(context.isInitialized());
    assert.ok(!context.isShutdown());
  });

  it('context basic shutdown', async function () {
    let context = new rclnodejs.Context();
    await rclnodejs.init(context);
    context.shutdown();
    assert.ok(!context.isUninitialized());
    assert.ok(!context.isInitialized());
    assert.ok(context.isShutdown());
  });

  it('context nodes(), onNodeCreated() & onNodeDestroyed()', async function () {
    let context = new rclnodejs.Context();
    await rclnodejs.init(context);

    let node1 = new rclnodejs.Node('n1', undefined, context);
    assert.strictEqual(context.nodes.length, 1);
    assert.strictEqual(context.nodes[0].name(), 'n1');

    let node2 = new rclnodejs.Node('n2', undefined, context);
    assert.strictEqual(context.nodes.length, 2);

    context.shutdown();
    assert.strictEqual(context.nodes.length, 0);
  });

  it('context domain id', async function () {
    let context = new rclnodejs.Context(BigInt(123));
    await rclnodejs.init(context);
    assert.strictEqual(typeof context.domainId, 'bigint');
    assert.strictEqual(context.domainId, BigInt(123));
  });

  it('context with default domain id', async function () {
    let context = new rclnodejs.Context();
    await rclnodejs.init(context);
    assert.strictEqual(typeof context.domainId, 'bigint');
  });

  for (const [termination, operation, expectedCode] of [
    ['natural exit', 'for (const node of nodes) node.stop();', 0],
    ['explicit exit', 'process.exit(7);', 7],
    ['an unhandled callback', 'guard.trigger();', 1],
  ]) {
    it(`shuts down all contexts on ${termination}`, function (done) {
      const moduleUrl = new URL('../index.js', import.meta.url).href;
      const script = `
        import assert from 'node:assert/strict';
        import rclnodejs from ${JSON.stringify(moduleUrl)};
        const contexts = [
          rclnodejs.Context.defaultContext(),
          new rclnodejs.Context(),
        ];
        const nodes = [];
        for (const [index, context] of contexts.entries()) {
          await rclnodejs.init(context);
          nodes.push(rclnodejs.createNode(
            'exit_cleanup_' + index, '', context,
            new rclnodejs.NodeOptions(false, [], false, false, false)
          ));
        }
        const guard = nodes[0].createGuardCondition(() => {
          throw new Error('exit cleanup regression');
        });
        for (const node of nodes) node.spin(-1);
        process.once('exit', (code) => {
          assert.equal(code, ${expectedCode});
          assert.equal(rclnodejs.Context.instances.length, 0);
          for (const context of contexts) {
            assert.equal(context.isShutdown(), true);
            assert.equal(context.isValid(), false);
            assert.equal(context.nodes.length, 0);
          }
          for (const node of nodes) assert.equal(node.spinning, false);
          console.log('all contexts finalized');
        });
        ${operation}
      `;
      execFile(
        process.execPath,
        ['--input-type=module', '--eval', script],
        { timeout: 10000, killSignal: 'SIGKILL' },
        (error, stdout, stderr) => {
          try {
            assert.equal(error?.code ?? 0, expectedCode, stderr);
            assert.equal(error?.signal ?? null, null, stderr);
            assert.match(stdout, /all contexts finalized/);
            assert.doesNotMatch(stderr, /ERROR: (?:Address|Leak)Sanitizer/);
            if (expectedCode === 1) {
              assert.match(stderr, /exit cleanup regression/);
            }
            done();
          } catch (failure) {
            done(failure);
          }
        }
      );
    });
  }
});
