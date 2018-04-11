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
    let logger = rclnodejs.logging.getLogger('severity_logger');
    logger.setLoggerLevel(logger.LoggingSeverity.DEBUG);
    assert.equal(logger.loggerEffectiveLevel, logger.LoggingSeverity.DEBUG);
  });

  it('Test logger name', function() {
    let logger = rclnodejs.logging.getLogger('logger');
    assert.equal(logger.name, 'logger');
  });

  it('Test severity level threshold', function() {
    let logger = rclnodejs.logging.getLogger('threshold_logger');
    logger.setLoggerLevel(logger.LoggingSeverity.INFO);

    // Logging below threshold not expected to be logged
    assert.equal(logger.debug('message debug'), false);

    // Logging at or above threshold expected to be logged
    assert.equal(logger.info('message info'), true);
    assert.equal(logger.warn('message warn'), true);
    assert.equal(logger.fatal('message fatal'), true);
  });
});
