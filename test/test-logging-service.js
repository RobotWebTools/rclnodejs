// Copyright (c) 2026, The Robot Web Tools Contributors
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
import sinon from 'sinon';
import Logging from '../lib/logging.js';
import LoggingService from '../lib/logging_service.js';

const LOGGING_SEVERITY = {
  UNSET: 0,
  DEBUG: 10,
  INFO: 20,
};

describe('LoggingService test suite', function () {
  let sandbox;

  beforeEach(function () {
    sandbox = sinon.createSandbox();
  });

  afterEach(function () {
    sandbox.restore();
  });

  it('starts get_logger_levels and set_logger_levels services', function () {
    const node = {
      name: () => 'logger_node',
      createService: sandbox.stub(),
    };
    const service = new LoggingService(node);

    service.start();
    service.start();

    assert.strictEqual(service.isStarted(), true);
    assert.strictEqual(node.createService.callCount, 2);
    assert.deepStrictEqual(node.createService.firstCall.args.slice(0, 2), [
      'rcl_interfaces/srv/GetLoggerLevels',
      'logger_node/get_logger_levels',
    ]);
    assert.deepStrictEqual(node.createService.secondCall.args.slice(0, 2), [
      'rcl_interfaces/srv/SetLoggerLevels',
      'logger_node/set_logger_levels',
    ]);
  });

  it('returns logger levels and maps lookup failures to UNSET', function () {
    sandbox.stub(Logging, 'getLogger').callsFake((name) => {
      if (name === 'missing_logger') {
        throw new Error('logger not found');
      }
      return { loggerEffectiveLevel: LOGGING_SEVERITY.INFO };
    });
    const service = new LoggingService({});
    const response = {
      template: { levels: [] },
      send: sandbox.spy(),
    };

    service._handleGetLoggerLevels(
      { names: ['existing_logger', 'missing_logger'] },
      response
    );

    assert.strictEqual(response.send.calledOnce, true);
    assert.deepStrictEqual(response.send.firstCall.args[0].levels, [
      { name: 'existing_logger', level: LOGGING_SEVERITY.INFO },
      { name: 'missing_logger', level: LOGGING_SEVERITY.UNSET },
    ]);
  });

  it('sets logger levels and reports per-level failures', function () {
    sandbox.stub(Logging, 'getLogger').callsFake((name) => ({
      setLoggerLevel(level) {
        if (name === 'bad_logger') {
          throw new Error(`failed to set ${level}`);
        }
      },
    }));
    const service = new LoggingService({});
    const response = {
      template: { results: [] },
      send: sandbox.spy(),
    };

    service._handleSetLoggerLevels(
      {
        levels: [
          { name: 'good_logger', level: LOGGING_SEVERITY.DEBUG },
          { name: 'bad_logger', level: 999 },
        ],
      },
      response
    );

    assert.strictEqual(response.send.calledOnce, true);
    assert.deepStrictEqual(response.send.firstCall.args[0].results, [
      { successful: true, reason: '' },
      { successful: false, reason: 'failed to set 999' },
    ]);
  });
});
