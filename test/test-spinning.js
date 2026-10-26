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

describe('Spin testing', function () {
  var node;
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(function () {
    node = rclnodejs.createNode('spin_node');
  });

  afterEach(function () {
    node.destroy();
  });

  it('rclnodejs.spin()', function () {
    rclnodejs.spin(node);
  });

  it('flushes promises from native callbacks without another JS event', function (done) {
    this.timeout(10000);
    const moduleUrl = new URL('../index.js', import.meta.url).href;
    const script = `
      import assert from 'node:assert/strict';
      import rclnodejs from ${JSON.stringify(moduleUrl)};
      await rclnodejs.init();
      const node = rclnodejs.createNode('promise_wakeup_regression');
      let timer;
      const completion = new Promise((resolve) => {
        timer = node.createTimer(10000000n, () => {
          timer.cancel();
          resolve();
        });
      });
      let watchdogFired = false;
      const watchdog = setTimeout(() => {
        watchdogFired = true;
      }, 1500);

      try {
        rclnodejs.spin(node);
        await completion;
        assert.equal(
          watchdogFired,
          false,
          'Native callbacks must flush promises without another JavaScript event'
        );
      } finally {
        clearTimeout(watchdog);
        rclnodejs.shutdown();
      }
    `;

    execFile(
      process.execPath,
      ['--input-type=module', '--eval', script],
      { timeout: 8000 },
      (error) => done(error)
    );
  });

  it('rclnodejs.spinOnce()', function () {
    rclnodejs.spinOnce(node);
  });

  it('rclnodejs.spinOnce() throws when already spinning', function () {
    rclnodejs.spin(node);
    assert.throws(function () {
      rclnodejs.spinOnce(node);
    });
  });

  it('preserves long spinOnce timeouts without nanosecond overflow', function (done) {
    this.timeout(15000);
    const moduleUrl = new URL('../index.js', import.meta.url).href;
    const script = `
      import assert from 'node:assert/strict';
      import { performance } from 'node:perf_hooks';
      import rclnodejs from ${JSON.stringify(moduleUrl)};
      await rclnodejs.init();
      const node = rclnodejs.createNode(
        'long_spin_once_regression', '', rclnodejs.Context.defaultContext(),
        new rclnodejs.NodeOptions(false, [], false, false, false)
      );
      try {
        let called = false;
        node.createTimer(1000000000n, () => { called = true; });
        const start = performance.now();
        node.spinOnce(5000);
        assert.equal(called, true, 'spinOnce returned before the timer was ready');
        assert.ok(performance.now() - start >= 800);
      } finally {
        rclnodejs.shutdown();
      }
    `;
    execFile(
      process.execPath,
      ['--input-type=module', '--eval', script],
      { timeout: 10000, killSignal: 'SIGKILL' },
      (error) => done(error)
    );
  });

  for (const spinMethod of ['spin', 'spinOnce']) {
    for (const [scenario, operation, expected] of [
      [
        'destroys another ready timer',
        'node.destroyTimer(second);',
        ['first', 'third'],
      ],
      ['cancels another ready timer', 'second.cancel();', ['first', 'third']],
      [
        'destroys itself',
        'node.destroyTimer(first);',
        ['first', 'second', 'third'],
      ],
      ['destroys the node', 'node.destroy();', ['first']],
      ['shuts down the context', 'context.shutdown();', ['first']],
      [
        'replaces another ready timer',
        `node.destroyTimer(second);
         node.createTimer(0n, () => calls.push('replacement'));`,
        ['first', 'third'],
      ],
    ]) {
      it(`${spinMethod} handles a timer callback that ${scenario}`, function (done) {
        this.timeout(15000);
        const moduleUrl = new URL('../index.js', import.meta.url).href;
        // Keep a native abort in a regression from terminating the test runner.
        const script = `
          import assert from 'node:assert/strict';
          import rclnodejs from ${JSON.stringify(moduleUrl)};
          await rclnodejs.init();
          const context = rclnodejs.Context.defaultContext();
          const node = rclnodejs.createNode('ready_timer_lifetime_regression');
          const calls = [];
          let first;
          let second;
          const completion = new Promise((resolve) => {
            first = node.createTimer(0n, () => {
              calls.push('first');
              first.cancel();
              ${operation}
              resolve();
            });
          });
          second = node.createTimer(0n, () => {
            calls.push('second');
            second.cancel();
          });
          const third = node.createTimer(0n, () => {
            calls.push('third');
            third.cancel();
          });

          try {
            assert.ok([first, second, third].every((timer) => timer.isReady()));
            node.${spinMethod}();
            await completion;
            assert.deepEqual(calls, ${JSON.stringify(expected)});
          } finally {
            context.shutdown();
          }
        `;
        execFile(
          process.execPath,
          ['--input-type=module', '--eval', script],
          { timeout: 10000, killSignal: 'SIGKILL' },
          (error) => done(error)
        );
      });
    }
  }

  describe('Interruptible spinning', function () {
    this.timeout(15000);

    function runInChild(body, done, timeout = 10000) {
      const moduleUrl = new URL('../index.js', import.meta.url).href;
      const script = `
        import assert from 'node:assert/strict';
        import { performance } from 'node:perf_hooks';
        import { setTimeout as delay } from 'node:timers/promises';
        import rclnodejs from ${JSON.stringify(moduleUrl)};
        await rclnodejs.init();
        const context = rclnodejs.Context.defaultContext();
        const node = rclnodejs.createNode(
          'interruptible_spin_regression', '', context,
          new rclnodejs.NodeOptions(false, [], false, false, false)
        );
        try {
          ${body}
        } finally {
          context.shutdown();
        }
      `;
      execFile(
        process.execPath,
        ['--input-type=module', '--eval', script],
        { timeout, killSignal: 'SIGKILL' },
        (error) => done(error)
      );
    }

    for (const timeout of [1500, -1]) {
      for (const operation of [
        'node.stop()',
        'node.destroy()',
        'context.shutdown()',
      ]) {
        it(`${operation} interrupts a ${timeout} ms wait`, function (done) {
          runInChild(
            `
              let callbacks = 0;
              node.createGuardCondition(() => callbacks++);
              node.spin(${timeout});
              await delay(200);
              const start = performance.now();
              ${operation};
              const elapsed = performance.now() - start;
              assert.ok(elapsed < 500, 'Shutdown took ' + elapsed + ' ms');
              assert.equal(node.spinning, false);
              assert.equal(callbacks, 0);
            `,
            done
          );
        });
      }
    }

    it('supports repeated start/stop and still dispatches user guards', function (done) {
      runInChild(
        `
          let callbacks = 0;
          let complete;
          const guard = node.createGuardCondition(() => {
            callbacks++;
            node.stop();
            complete();
          });
          for (let i = 0; i < 10; i++) {
            const dispatched = new Promise((resolve) => { complete = resolve; });
            node.spin(-1);
            guard.trigger();
            await dispatched;
            assert.equal(callbacks, i + 1);
            node.stop();

            node.spin(-1);
            await delay(20);
            const start = performance.now();
            node.stop();
            assert.ok(performance.now() - start < 500);
          }
        `,
        done
      );
    });

    it('stops an empty executor immediately and after waiting for handles', function (done) {
      runInChild(
        `
          for (let i = 0; i < 20; i++) {
            node.spin(-1);
            if (i % 2) await delay(10);
            const start = performance.now();
            node.stop();
            assert.ok(performance.now() - start < 500);
          }
          const start = performance.now();
          node.spinOnce(-1);
          assert.ok(performance.now() - start < 500);
        `,
        done
      );
    });

    for (const [scenario, setup] of [
      [
        'creating and destroying the only entity',
        `node.spinOnce(0);
         const oldTimer = node.createTimer(0n, () => {});
         node.destroyTimer(oldTimer);`,
      ],
      [
        'stopping and emptying a nonempty executor',
        `const oldGuard = node.createGuardCondition(() => {});
         node.spin(-1);
         node.stop();
         node.destroyGuardCondition(oldGuard);`,
      ],
    ]) {
      for (const entity of ['timer', 'subscription']) {
        it(`creates a ${entity} after a stale handle wake from ${scenario}`, function (done) {
          runInChild(
            `
              ${setup}
              node.spin(-1);
              await delay(200);
              let resolve;
              const completed = new Promise((done) => { resolve = done; });
              let callbacks = 0;
              const callback = () => {
                callbacks++;
                node.stop();
                resolve();
              };
              const start = performance.now();
              if (${JSON.stringify(entity)} === 'timer') {
                node.createTimer(1000000n, callback);
              } else {
                node.createSubscription(
                  'std_msgs/msg/String', 'stale_wake_topic', callback
                );
              }
              assert.ok(
                performance.now() - start < 500,
                'Creating an entity must not block behind an empty wait set'
              );
              let publishInterval;
              try {
                if (${JSON.stringify(entity)} === 'subscription') {
                  const publisher = node.createPublisher(
                    'std_msgs/msg/String', 'stale_wake_topic'
                  );
                  publishInterval = setInterval(() => publisher.publish('wake'), 20);
                }
                await completed;
                assert.equal(callbacks, 1);
                assert.equal(node.spinning, false);
              } finally {
                clearInterval(publishInterval);
              }
            `,
            done
          );
        });
      }
    }

    for (const entity of ['timer', 'subscription']) {
      it(`creates a ${entity} promptly after removing the last entity during spin`, function (done) {
        this.timeout(25000);
        runInChild(
          `
            const messageType = rclnodejs.require('std_msgs/msg/String');
            node.spinOnce(0);
            for (let i = 0; i < 100; i++) {
              const guard = node.createGuardCondition(() => {});
              node.spin(100);
              node.destroyGuardCondition(guard);
              await delay(1);
              const start = performance.now();
              const created = ${JSON.stringify(entity)} === 'timer'
                ? node.createTimer(1000000n, () => {})
                : node.createSubscription(messageType, 'empty_transition_topic', () => {});
              const elapsed = performance.now() - start;
              node.stop();
              if (${JSON.stringify(entity)} === 'timer') {
                node.destroyTimer(created);
              } else {
                node.destroySubscription(created);
              }
              assert.ok(
                elapsed < 75,
                'Entity creation waited for an empty wait set: ' + elapsed + ' ms (iteration ' + i + ')'
              );
            }
          `,
          done,
          20000
        );
      });
    }

    it('shuts down from a timer callback during an infinite spin', function (done) {
      runInChild(
        `
          const completed = new Promise((resolve) => {
            node.createTimer(10000000n, () => {
              context.shutdown();
              resolve();
            });
          });
          node.spin(-1);
          await completed;
          assert.equal(node.spinning, false);
          assert.equal(context.isShutdown(), true);
        `,
        done
      );
    });

    it('discards pending callbacks before restarting with different entities', function (done) {
      runInChild(
        `
          let timerCalls = 0;
          const timer = node.createTimer(0n, () => { timerCalls++; });
          node.spin(-1);
          Atomics.wait(new Int32Array(new SharedArrayBuffer(4)), 0, 0, 30);
          node.stop();
          node.destroyTimer(timer);
          const completed = new Promise((resolve) => {
            const guard = node.createGuardCondition(() => {
              node.stop();
              resolve();
            });
            guard.trigger();
          });
          node.spin(-1);
          await completed;
          assert.equal(timerCalls, 0);
        `,
        done
      );
    });

    it('reports guard initialization failure without marking the node as spinning', function (done) {
      runInChild(
        `
          context.shutdown();
          assert.throws(
            () => node.spin(-1),
            /Failed to initialize executor guard condition/
          );
          assert.equal(node.spinning, false);
          node.stop();
        `,
        done
      );
    });
  });
});
