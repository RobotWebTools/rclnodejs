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
});
