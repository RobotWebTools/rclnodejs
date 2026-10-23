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
});
