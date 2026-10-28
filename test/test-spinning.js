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
import sinon from 'sinon';
import rclnodejs from '../index.js';
import DistroUtils from '../lib/distro.js';
import Entity from '../lib/entity.js';
import {
  PublisherEventCallbacks,
  SubscriptionEventCallbacks,
} from '../lib/event_handler.js';
import nativeLoader from '../lib/native_loader.js';

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

  for (const entityKind of ['publisher', 'subscription']) {
    for (const origin of ['timer', 'event']) {
      for (const throws of [false, true]) {
        it(`skips destroyed ${entityKind} events after ${origin} callbacks${throws ? ' throw' : ''}`, function () {
          if (DistroUtils.getDistroId() < DistroUtils.getDistroId('jazzy')) {
            this.skip();
          }
          const EventCallbacks =
            entityKind === 'publisher'
              ? PublisherEventCallbacks
              : SubscriptionEventCallbacks;
          const calls = [];
          const failure = new Error('event owner destroyed');
          const createEndpoint = (topic, callback) => {
            const callbacks = new EventCallbacks();
            callbacks.deadline = callback;
            return entityKind === 'publisher'
              ? node.createPublisher(
                  'std_msgs/msg/String',
                  topic,
                  undefined,
                  callbacks
                )
              : node.createSubscription(
                  'std_msgs/msg/String',
                  topic,
                  undefined,
                  () => {},
                  callbacks
                );
          };
          let releasedEndpoint;
          const destroyEndpoint = () => {
            calls.push('source');
            if (entityKind === 'publisher') {
              node.destroyPublisher(releasedEndpoint);
            } else {
              node.destroySubscription(releasedEndpoint);
            }
            if (throws) throw failure;
          };
          let sourceHandle;
          if (origin === 'timer') {
            const timer = node.createTimer(0n, () => {
              timer.cancel();
              destroyEndpoint();
            });
            sourceHandle = timer.handle;
          } else {
            const source = createEndpoint('event_source', destroyEndpoint);
            sourceHandle = source.events[0].handle;
          }
          releasedEndpoint = createEndpoint('event_released', () =>
            calls.push('released')
          );
          const liveEndpoint = createEndpoint('event_live', () =>
            calls.push('live')
          );
          const releasedEvent = releasedEndpoint.events[0];
          const takeData = sinon.spy(releasedEvent, 'takeData');
          const handles = [
            sourceHandle,
            releasedEvent.handle,
            liveEndpoint.events[0].handle,
          ];
          try {
            if (throws) {
              assert.throws(
                () => node.execute(handles),
                (error) => error === failure
              );
            } else {
              node.execute(handles);
            }
            assert.equal(releasedEvent.isDestroyed(), true);
            assert.equal(takeData.callCount, 0);
            assert.deepEqual(calls, throws ? ['source'] : ['source', 'live']);
          } finally {
            takeData.restore();
          }
        });
      }
    }
  }

  it('clears destroyed handle references when a callback throws', function () {
    const failure = new Error('guard callback failed');
    let released;
    const first = node.createGuardCondition(() => {
      node.destroyGuardCondition(released);
      throw failure;
    });
    released = node.createGuardCondition(() => assert.fail('must not run'));
    const clearHandles = sinon.spy(Entity, '_gcHandles');
    try {
      assert.throws(
        () => node.execute([first.handle, released.handle]),
        (error) => error === failure
      );
      assert.equal(released.isDestroyed(), true);
      assert.equal(clearHandles.callCount, 1);
    } finally {
      clearHandles.restore();
    }
  });

  it('releases raw messages when callbacks throw', function () {
    const failure = new Error('message callback failed');
    let destroyedMessage;
    class Message {
      toRawROS() {
        return {};
      }

      static destroyRawROS(message) {
        destroyedMessage = message;
      }
    }
    assert.throws(
      () =>
        node._runWithMessageType(Message, () => {
          throw failure;
        }),
      (error) => error === failure
    );
    assert.ok(destroyedMessage instanceof Message);
  });

  for (const actionKind of ['client', 'server']) {
    for (const [operation, throws] of [
      ['callback', false],
      ['callback', true],
      ['destruction', false],
      ['destruction', true],
      ['stop', false],
      ['stop', true],
    ]) {
      it(`handles ready action ${actionKind} events after ${operation === 'callback' ? 'a callback' : operation}${throws ? ' and an error' : ''}`, function () {
        const sandbox = sinon.createSandbox();
        const failure = new Error('action callback failed');
        const calls = [];
        const messageKind = actionKind === 'client' ? 'Response' : 'Request';
        const collection =
          actionKind === 'client' ? node._actionClients : node._actionServers;
        let destroyed = false;
        const action = {
          handle: {
            properties: {
              [`isGoal${messageKind}Ready`]: true,
              [`isCancel${messageKind}Ready`]: true,
            },
          },
          typeClass: {
            impl: {
              SendGoalService: { [messageKind]: Object },
              CancelGoal: { [messageKind]: Object },
            },
          },
          isDestroyed: () => destroyed,
          [`processGoal${messageKind}`]: () => {
            calls.push('goal');
            destroyed = operation === 'destruction';
            if (operation === 'stop') node.stop();
            if (throws) throw failure;
          },
          [`processCancel${messageKind}`]: () => calls.push('cancel'),
        };
        collection.push(action);
        try {
          sandbox.stub(nativeLoader, `actionTakeGoal${messageKind}`).returns(1);
          sandbox
            .stub(nativeLoader, `actionTakeCancel${messageKind}`)
            .returns(1);
          sandbox
            .stub(node, '_runWithMessageType')
            .callsFake((type, callback) => callback({}, () => ({})));
          if (throws) {
            assert.throws(
              () => node.execute([action.handle]),
              (error) => error === failure
            );
          } else {
            node.execute([action.handle]);
          }
          assert.deepEqual(
            calls,
            operation !== 'callback' || throws ? ['goal'] : ['goal', 'cancel']
          );
        } finally {
          collection.splice(collection.indexOf(action), 1);
          sandbox.restore();
        }
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

    it('publishes startup before the worker handles its first event', function (done) {
      runInChild(
        `
          let complete;
          let callbacks = 0;
          const guard = node.createGuardCondition(() => {
            callbacks++;
            node.stop();
            complete();
          });
          for (let i = 0; i < 50; i++) {
            const completed = new Promise((resolve) => { complete = resolve; });
            guard.trigger();
            node.spin(-1);
            await completed;
            assert.equal(callbacks, i + 1);
            assert.equal(node.spinning, false);
            await delay(0);
          }
        `,
        done
      );
    });

    it('does not acknowledge restarted work from an earlier callback', function (done) {
      runInChild(
        `
          let complete;
          let callbacks = 0;
          const completed = new Promise((resolve) => { complete = resolve; });
          const wait = new Int32Array(new SharedArrayBuffer(4));
          const guard = node.createGuardCondition(() => {
            callbacks++;
            if (callbacks === 1) {
              node.stop();
              node.spin(-1);
              guard.trigger();
              Atomics.wait(wait, 0, 0, 100);
              setImmediate(() => Atomics.wait(wait, 0, 0, 100));
            } else {
              node.destroyGuardCondition(guard);
              node.stop();
              complete();
            }
          });
          node.spin(-1);
          guard.trigger();
          await completed;
          assert.equal(callbacks, 2);
        `,
        done
      );
    });

    for (const restartFrom of ['callback', 'uncaughtException', 'spinOnce']) {
      it(`waits for the earlier callback before restarting from ${restartFrom}`, function (done) {
        runInChild(
          `
            let complete;
            const completed = new Promise((resolve) => { complete = resolve; });
            const wait = new Int32Array(new SharedArrayBuffer(4));
            const failure = new Error('restart from error handler');
            let first;
            function restartAndReplaceGuard() {
              node.stop();
              node.spin(-1);
              Atomics.wait(wait, 0, 0, 100);
              node.destroyGuardCondition(first);
              const second = node.createGuardCondition(() => {
                node.stop();
                complete();
              });
              second.trigger();
            }
            first = node.createGuardCondition(() => {
              if (${JSON.stringify(restartFrom)} !== 'uncaughtException') {
                restartAndReplaceGuard();
              } else {
                throw failure;
              }
            });
            if (${JSON.stringify(restartFrom)} === 'uncaughtException') {
              process.once('uncaughtException', (error) => {
                assert.equal(error, failure);
                restartAndReplaceGuard();
              });
            }
            if (${JSON.stringify(restartFrom)} === 'spinOnce') {
              first.trigger();
              node.spinOnce(0);
            } else {
              node.spin(-1);
              first.trigger();
            }
            await completed;
          `,
          done,
          4000
        );
      });
    }

    for (const restartFrom of ['callback', 'uncaughtException', 'spinOnce']) {
      it(`can stop again while restart waits for ${restartFrom}`, function (done) {
        runInChild(
          `
            let complete;
            const completed = new Promise((resolve) => { complete = resolve; });
            const failure = new Error('restart and stop');
            function restartAndStop() {
              node.stop();
              node.spin(-1);
              node.stop();
              complete();
            }
            const guard = node.createGuardCondition(() => {
              if (${JSON.stringify(restartFrom)} === 'uncaughtException') {
                throw failure;
              }
              restartAndStop();
            });
            if (${JSON.stringify(restartFrom)} === 'uncaughtException') {
              process.once('uncaughtException', (error) => {
                assert.equal(error, failure);
                restartAndStop();
              });
            }
            guard.trigger();
            if (${JSON.stringify(restartFrom)} === 'spinOnce') {
              node.spinOnce(0);
            } else {
              node.spin(-1);
            }
            await completed;
            assert.equal(node.spinning, false);
          `,
          done,
          4000
        );
      });
    }

    it('does not execute an earlier ready batch after an immediate restart', function (done) {
      runInChild(
        `
          const calls = [];
          let restarted;
          const afterRestart = new Promise((resolve) => { restarted = resolve; });
          let complete;
          const completed = new Promise((resolve) => { complete = resolve; });
          const first = node.createGuardCondition(() => {
            calls.push('first');
            node.stop();
            node.spin(-1);
            setImmediate(restarted);
          });
          const second = node.createGuardCondition(() => {
            calls.push('second');
            complete();
          });
          first.trigger();
          second.trigger();
          node.spin(-1);
          await afterRestart;
          assert.deepEqual(calls, ['first']);
          second.trigger();
          await completed;
          node.stop();
          assert.deepEqual(calls, ['first', 'second']);
        `,
        done
      );
    });

    it('stops and restarts without pumping unrelated JavaScript callbacks', function (done) {
      runInChild(
        `
          let callbacks = 0;
          const completed = new Promise((resolve) => {
            setImmediate(() => {
              callbacks++;
              resolve();
            });
          });
          for (let i = 0; i < 3; i++) {
            node.spin(-1);
            node.stop();
            assert.equal(node.spinning, false);
            assert.equal(callbacks, 0, 'stop() must not pump the event loop');
          }
          await completed;
          assert.equal(callbacks, 1);
        `,
        done
      );
    });

    it('reports callback errors through uncaughtException and can restart', function (done) {
      runInChild(
        `
          const failure = new Error('executor callback error');
          const completed = new Promise((resolve) => {
            process.once('uncaughtException', (error) => {
              assert.equal(error, failure);
              node.stop();
              resolve();
            });
          });
          const timer = node.createTimer(0n, () => {
            timer.cancel();
            throw failure;
          });
          node.spin(-1);
          await completed;
          assert.equal(node.spinning, false);
          await delay(0);
          node.spin(-1);
          node.stop();
        `,
        done
      );
    });

    it('continues spinning on new work after a handled callback error', function (done) {
      runInChild(
        `
          const failure = new Error('handled callback failure');
          const calls = [];
          const reported = new Promise((resolve) => {
            process.once('uncaughtException', resolve);
          });
          let complete;
          const completed = new Promise((resolve) => { complete = resolve; });
          const first = node.createGuardCondition(() => {
            calls.push('first');
            throw failure;
          });
          const later = node.createGuardCondition(() => {
            calls.push('later');
            node.stop();
            complete();
          });
          first.trigger();
          node.spin(-1);
          assert.equal(await reported, failure);
          assert.equal(node.spinning, true);
          assert.deepEqual(calls, ['first']);
          later.trigger();
          await completed;
          assert.deepEqual(calls, ['first', 'later']);
          assert.equal(node.spinning, false);
        `,
        done
      );
    });

    it('exits on an unhandled callback error without running later callbacks', function (done) {
      const moduleUrl = new URL('../index.js', import.meta.url).href;
      const script = `
        import rclnodejs from ${JSON.stringify(moduleUrl)};
        await rclnodejs.init();
        const node = rclnodejs.createNode(
          'unhandled_callback_error', '', rclnodejs.Context.defaultContext(),
          new rclnodejs.NodeOptions(false, [], false, false, false)
        );
        const first = node.createGuardCondition(() => {
          throw new Error('unhandled callback error');
        });
        const second = node.createGuardCondition(() => {
          console.log('unexpected second callback');
        });
        first.trigger();
        second.trigger();
        node.spin(-1);
      `;
      execFile(
        process.execPath,
        ['--input-type=module', '--eval', script],
        { timeout: 10000, killSignal: 'SIGKILL' },
        (error, stdout, stderr) => {
          try {
            assert.ok(error);
            assert.equal(error.code, 1);
            assert.equal(error.signal, null);
            assert.match(stderr, /unhandled callback error/);
            assert.doesNotMatch(stderr, /FATAL ERROR/);
            assert.doesNotMatch(stdout, /unexpected second callback/);
            done();
          } catch (failure) {
            done(failure);
          }
        }
      );
    });

    for (const spinMethod of ['spin', 'spinOnce']) {
      it(`${spinMethod} preserves non-Error thrown values`, function (done) {
        runInChild(
          `
            for (const value of [undefined, null, false, 0, 'failure', Symbol('failure'), {}]) {
              const guard = node.createGuardCondition(() => { throw value; });
              guard.trigger();
              let caught = false;
              let error;
              if (${JSON.stringify(spinMethod)} === 'spin') {
                const reported = new Promise((resolve) => {
                  process.once('uncaughtException', (thrown) => resolve({ thrown }));
                });
                node.spin(-1);
                error = (await reported).thrown;
                caught = true;
                node.stop();
              } else {
                try { node.spinOnce(0); } catch (thrown) {
                  caught = true;
                  error = thrown;
                }
              }
              assert.equal(caught, true);
              assert.equal(error, value);
              node.destroyGuardCondition(guard);
            }
          `,
          done
        );
      });

      it(`${spinMethod} stops the ready batch at the first guard error`, function (done) {
        runInChild(
          `
            const firstError = new Error('first guard failure');
            const calls = [];
            let complete;
            const completed = new Promise((resolve) => { complete = resolve; });
            const first = node.createGuardCondition(() => {
              calls.push('first');
              throw firstError;
            });
            const second = node.createGuardCondition(() => {
              calls.push('second');
              throw new Error('second guard failure');
            });
            const third = node.createGuardCondition(() => {
              calls.push('third');
              complete();
            });
            first.trigger();
            second.trigger();
            third.trigger();
            let error;
            if (${JSON.stringify(spinMethod)} === 'spin') {
              const reported = new Promise((resolve) => {
                process.once('uncaughtException', resolve);
              });
              node.spin(-1);
              error = await reported;
              node.stop();
            } else {
              try { node.spinOnce(0); } catch (failure) { error = failure; }
            }
            assert.equal(error, firstError);
            assert.deepEqual(calls, ['first']);
            node.destroyGuardCondition(first);
            node.destroyGuardCondition(second);
            third.trigger();
            if (${JSON.stringify(spinMethod)} === 'spin') {
              node.spin(-1);
              await completed;
              node.stop();
            } else {
              node.spinOnce(0);
              await completed;
            }
            assert.deepEqual(calls, ['first', 'third']);
          `,
          done
        );
      });

      it(`${spinMethod} stops before ready guards after a timer error`, function (done) {
        runInChild(
          `
            const failure = new Error('timer failure');
            const calls = [];
            const timer = node.createTimer(0n, () => {
              timer.cancel();
              calls.push('timer');
              throw failure;
            });
            const guard = node.createGuardCondition(() => calls.push('guard'));
            guard.trigger();
            let error;
            if (${JSON.stringify(spinMethod)} === 'spin') {
              const reported = new Promise((resolve) => {
                process.once('uncaughtException', resolve);
              });
              node.spin(-1);
              error = await reported;
              node.stop();
            } else {
              try { node.spinOnce(0); } catch (caught) { error = caught; }
            }
            assert.equal(error, failure);
            assert.deepEqual(calls, ['timer']);
          `,
          done
        );
      });

      it(`${spinMethod} discards remaining ready callbacks after stop and an error`, function (done) {
        runInChild(
          `
            const failure = new Error('stopped callback failed');
            const calls = [];
            const first = node.createGuardCondition(() => {
              calls.push('first');
              node.stop();
              throw failure;
            });
            let complete;
            const completed = new Promise((resolve) => { complete = resolve; });
            const second = node.createGuardCondition(() => {
              calls.push('second');
              complete();
            });
            first.trigger();
            second.trigger();
            let error;
            if (${JSON.stringify(spinMethod)} === 'spin') {
              const reported = new Promise((resolve) => {
                process.once('uncaughtException', resolve);
              });
              node.spin(-1);
              error = await reported;
            } else {
              try { node.spinOnce(0); } catch (caught) { error = caught; }
            }
            assert.equal(error, failure);
            assert.deepEqual(calls, ['first']);
            second.trigger();
            if (${JSON.stringify(spinMethod)} === 'spin') {
              node.spin(-1);
              await completed;
              node.stop();
            } else {
              node.spinOnce(0);
              await completed;
            }
            assert.deepEqual(calls, ['first', 'second']);
          `,
          done
        );
      });

      for (const operation of ['node.destroy()', 'context.shutdown()']) {
        for (const throws of [false, true]) {
          it(`${spinMethod} discards ready guards after ${operation}${throws ? ' and an error' : ''}`, function (done) {
            runInChild(
              `
                const failure = new Error('destroyed callback failed');
                const calls = [];
                let complete;
                const completed = new Promise((resolve) => { complete = resolve; });
                const timer = node.createTimer(0n, () => {
                  calls.push('timer');
                  timer.cancel();
                  ${operation};
                  if (${throws}) throw failure;
                  complete();
                });
                const guard = node.createGuardCondition(() => calls.push('guard'));
                guard.trigger();
                let error;
                if (${JSON.stringify(spinMethod)} === 'spin') {
                  const reported = ${throws} ? new Promise((resolve) => {
                    process.once('uncaughtException', (caught) => resolve({ caught }));
                  }) : completed;
                  node.spin(-1);
                  const result = await reported;
                  if (${throws}) error = result.caught;
                } else {
                  try { node.spinOnce(0); } catch (caught) { error = caught; }
                }
                assert.equal(error, ${throws} ? failure : undefined);
                assert.deepEqual(calls, ['timer']);
                assert.equal(node.spinning, false);
                assert.equal(context.nodes.includes(node), false);
                assert.equal(context.isShutdown(), ${JSON.stringify(operation)} === 'context.shutdown()');
              `,
              done
            );
          });
        }
      }
    }
  });
});
