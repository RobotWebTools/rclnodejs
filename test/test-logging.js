// Copyright (c) 2018 Intel Corporation. All rights reserved.
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

'use strict';

const assert = require('assert');
const rclnodejs = require('../index.js');

describe('Rclnodejs - Test logging util', function() {

  it('Test setting severity level', function() {
    const logger = rclnodejs.logging.getLogger('severity_logger');
    logger.setLoggerLevel(logger.LoggingSeverity.DEBUG);
    assert.deepStrictEqual(logger.loggerEffectiveLevel, logger.LoggingSeverity.DEBUG);
  });

  it('Test logger name', function() {
    const logger = rclnodejs.logging.getLogger('logger');
    assert.strictEqual(logger.name, 'logger');
  });

  it('Test severity level threshold', function() {
    const logger = rclnodejs.logging.getLogger('threshold_logger');
    logger.setLoggerLevel(logger.LoggingSeverity.INFO);

    // Logging below threshold not expected to be logged
    assert.strictEqual(logger.debug('message debug'), false);

    // Logging at or above threshold expected to be logged
    assert.strictEqual(logger.info('message info'), true);
    assert.strictEqual(logger.warn('message warn'), true);
    assert.strictEqual(logger.fatal('message fatal'), true);
  });

  it('Test logger name', function() {
    const logger = rclnodejs.logging.getLogger('logger');
    assert.strictEqual(logger.name, 'logger');
  });

  async function testLoglevel(level) {
    await rclnodejs.init(['--ros-args', '--log-level', level]);
    const logger = rclnodejs.logging.getLogger(`test_logger_${level}`);
    const expected = logger.LoggingSeverity[level.toUpperCase()];
    assert.deepStrictEqual(logger.loggerEffectiveLevel, expected);
    rclnodejs.shutdown();
  }

  for (const level of ['debug', 'info', 'warn', 'error', 'fatal']) {
    it(`Test commandline parameter configuration of log level '${level}'`, async function() {
      // test the specific log level
      await testLoglevel(level);
    });
  }

  it ('Test commandline parameter configuration resets correctly', async  function () {
    // reset the default log level to 'info'
    await testLoglevel('info');

    // check that it was reset to 'info' when creating a node
    await rclnodejs.init();
    const node = rclnodejs.createNode('test_node');
    const logger = node.getLogger();
    assert.deepStrictEqual(logger.loggerEffectiveLevel, logger.LoggingSeverity.INFO);
    rclnodejs.shutdown();
  });

});
