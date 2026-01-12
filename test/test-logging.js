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
const sinon = require('sinon');
const rclnodejsBinding = require('../lib/native_loader.js');
const Logging = require('../lib/logging.js');
const Context = require('../lib/context.js');

describe('Test logging util', function () {
  it('Test setting severity level', function () {
    const logger = rclnodejs.logging.getLogger('severity_logger');
    logger.setLoggerLevel(logger.LoggingSeverity.DEBUG);
    assert.deepStrictEqual(
      logger.loggerEffectiveLevel,
      logger.LoggingSeverity.DEBUG
    );
  });

  it('Test logger name', function () {
    const logger = rclnodejs.logging.getLogger('logger');
    assert.strictEqual(logger.name, 'logger');
  });

  it('Test severity level threshold', function () {
    const logger = rclnodejs.logging.getLogger('threshold_logger');
    logger.setLoggerLevel(logger.LoggingSeverity.INFO);

    // Logging below threshold not expected to be logged
    assert.strictEqual(logger.debug('message debug'), false);

    // Logging at or above threshold expected to be logged
    assert.strictEqual(logger.info('message info'), true);
    assert.strictEqual(logger.warn('message warn'), true);
    assert.strictEqual(logger.fatal('message fatal'), true);
  });

  it('Test logger name', function () {
    const logger = rclnodejs.logging.getLogger('logger');
    assert.strictEqual(logger.name, 'logger');
  });

  async function testLoglevel(level) {
    await rclnodejs.init(rclnodejs.Context.defaultContext(), [
      '--ros-args',
      '--log-level',
      level,
    ]);
    const logger = rclnodejs.logging.getLogger(`test_logger_${level}`);
    const expected = logger.LoggingSeverity[level.toUpperCase()];
    assert.deepStrictEqual(logger.loggerEffectiveLevel, expected);
    rclnodejs.shutdown();
  }

  for (const level of ['debug', 'info', 'warn', 'error', 'fatal']) {
    it(`Test commandline parameter configuration of log level '${level}'`, async function () {
      this.timeout(10000);
      // test the specific log level
      await testLoglevel(level);
    });
  }

  it('Test commandline parameter configuration resets correctly', async function () {
    // reset the default log level to 'info'
    await testLoglevel('info');

    // check that it was reset to 'info' when creating a node
    await rclnodejs.init();
    const node = rclnodejs.createNode('test_node');
    const logger = node.getLogger();
    assert.deepStrictEqual(
      logger.loggerEffectiveLevel,
      logger.LoggingSeverity.INFO
    );
    rclnodejs.shutdown();
  });

  it('Test getLoggingDirectory', function () {
    const logDir = rclnodejs.logging.getLoggingDirectory();
    assert.strictEqual(typeof logDir, 'string');
    assert.ok(logDir.length > 0);
  });

  it('Test enableRosout option true', async function () {
    if (
      rclnodejs.DistroUtils.getDistroId() <
      rclnodejs.DistroUtils.getDistroId('jazzy')
    ) {
      this.skip();
    }
    await rclnodejs.init();
    const options = new rclnodejs.NodeOptions();
    options.enableRosout = true;
    const node = rclnodejs.createNode(
      'node_with_rosout',
      '',
      rclnodejs.Context.defaultContext(),
      options
    );
    const publishers = node.getPublisherNamesAndTypesByNode(
      node.name(),
      node.namespace()
    );
    const rosoutPublisher = publishers.find((pub) =>
      pub.name.includes('rosout')
    );
    assert.notStrictEqual(rosoutPublisher, undefined);
    node.destroy();
    rclnodejs.shutdown();
  });

  it('Test enableRosout option false', async function () {
    if (
      rclnodejs.DistroUtils.getDistroId() <
      rclnodejs.DistroUtils.getDistroId('jazzy')
    ) {
      this.skip();
    }
    await rclnodejs.init();
    const options = new rclnodejs.NodeOptions();
    options.enableRosout = false;
    const node = rclnodejs.createNode(
      'node_without_rosout',
      '',
      rclnodejs.Context.defaultContext(),
      options
    );
    const publishers = node.getPublisherNamesAndTypesByNode(
      node.name(),
      node.namespace()
    );
    const rosoutPublisher = publishers.find((pub) =>
      pub.name.includes('rosout')
    );
    assert.strictEqual(rosoutPublisher, undefined);
    rclnodejs.shutdown();
  });

  it('Test rosoutQos option', async function () {
    if (
      rclnodejs.DistroUtils.getDistroId() <
      rclnodejs.DistroUtils.getDistroId('jazzy')
    ) {
      this.skip();
    }
    await rclnodejs.init();
    const options = new rclnodejs.NodeOptions();
    options.enableRosout = true;
    options.rosoutQos = rclnodejs.QoS.profileSensorData;
    const node = rclnodejs.createNode(
      'node_with_rosout_qos',
      '',
      rclnodejs.Context.defaultContext(),
      options
    );

    const publishers = node.getPublishersInfoByTopic('/rosout');
    const myPub = publishers.find(
      (p) => p.node_name === 'node_with_rosout_qos'
    );

    assert.notStrictEqual(myPub, undefined);
    // SensorData profile: Reliability = BEST_EFFORT (2), Durability = VOLATILE (2)
    // Default rosout profile: Reliability = RELIABLE (1), Durability = TRANSIENT_LOCAL (1)

    // Check reliability
    assert.strictEqual(
      myPub.qos_profile.reliability,
      rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
    );

    node.destroy();
    rclnodejs.shutdown();
  });

  it('Test child logger', function () {
    if (
      rclnodejs.DistroUtils.getDistroId() <=
      rclnodejs.DistroUtils.getDistroId('humble')
    ) {
      this.skip();
    }
    const logger = rclnodejs.logging.getLogger('parent_logger');
    const childLogger = logger.getChild('child_logger');
    assert.strictEqual(childLogger.name, 'parent_logger.child_logger');
    childLogger.destroy();
  });

  it('Test logging configure and shutdown', async function () {
    await rclnodejs.init();
    const logger = rclnodejs.logging.getLogger('config_logger');

    // Verify logging works initially
    assert.doesNotThrow(() => {
      logger.info('Initial message');
    });

    // Shutdown logging
    assert.doesNotThrow(() => {
      rclnodejs.logging.shutdown();
    });

    // Re-configure logging with thread-safe handler
    assert.doesNotThrow(() => {
      rclnodejs.logging.configure(rclnodejs.Context.defaultContext());
    });

    // Verify logging works after reconfiguration
    assert.doesNotThrow(() => {
      logger.info('Message after reconfiguration');
    });

    // Test invalid context
    assert.throws(() => {
      rclnodejs.logging.configure({});
    }, /context/);

    rclnodejs.shutdown();
  });
});

describe('Logging unit testing (Mocks)', function () {
  let sandbox;
  // Mock 'instanceof Context' is hard if we don't import real Context.
  // So we might need to import Context to use it in test or mock the instanceof check.

  beforeEach(function () {
    sandbox = sinon.createSandbox();
  });

  afterEach(function () {
    sandbox.restore();
  });

  describe('Validation', function () {
    const logger = new Logging('val-logger');

    it('setLoggerLevel throws on invalid input', function () {
      assert.throws(() => {
        logger.setLoggerLevel('high');
      }, /level.*number/);
    });

    it('getLogger throws on invalid name', function () {
      assert.throws(() => {
        Logging.getLogger(123);
      }, /name.*string/);
    });

    it('getChild throws on invalid name', function () {
      assert.throws(() => {
        logger.getChild('');
      }, /non-empty string/);
    });
  });

  describe('Logic and Binding Interaction', function () {
    it('setLoggerLevel calls binding', function () {
      const spy = sandbox.stub(rclnodejsBinding, 'setLoggerLevel');
      const logger = new Logging('test');
      logger.setLoggerLevel(10);
      assert.ok(spy.calledWith('test', 10));
    });

    it('loggerEffectiveLevel calls binding', function () {
      const stub = sandbox
        .stub(rclnodejsBinding, 'getLoggerEffectiveLevel')
        .returns(20);
      const logger = new Logging('test');
      assert.strictEqual(logger.loggerEffectiveLevel, 20);
    });

    it('logging methods call _log -> binding', function () {
      const logStub = sandbox.stub(rclnodejsBinding, 'log');
      const logger = new Logging('test');

      logger.debug('debug msg');
      assert.ok(logStub.calledWithMatch('test', 10, 'debug msg'));

      logger.info('info msg');
      assert.ok(logStub.calledWithMatch('test', 20, 'info msg'));

      logger.warn('warn msg');
      assert.ok(logStub.calledWithMatch('test', 30, 'warn msg'));

      logger.error('error msg');
      assert.ok(logStub.calledWithMatch('test', 40, 'error msg'));

      logger.fatal('fatal msg');
      assert.ok(logStub.calledWithMatch('test', 50, 'fatal msg'));
    });

    it('_log validates message type', function () {
      const logger = new Logging('test');
      assert.throws(() => {
        logger.debug(123);
      }, /message.*string/);
    });

    it('captures stack info (Caller class test)', function () {
      const logStub = sandbox.stub(rclnodejsBinding, 'log');
      const logger = new Logging('test');

      function internalFunction() {
        logger.info('msg');
      }
      internalFunction();

      assert.ok(logStub.calledOnce);
      const args = logStub.firstCall.args;
      // signature: (name, severity, message, functionName, lineNumber, fileName)
      const funcName = args[3];
      const lineNum = args[4];
      const fileName = args[5];

      // This might look different depending on how mocha runs it, but usually 'internalFunction'
      assert.strictEqual(funcName, 'internalFunction');
      assert.strictEqual(fileName, 'test-logging.js');
      assert.ok(typeof lineNum === 'number');
    });

    it('getChild logic (rosout sublogger)', function () {
      const logger = new Logging('parent');
      // rclnodejsBinding.addRosoutSublogger might be undefined in some envs unless we mock it
      if (!rclnodejsBinding.addRosoutSublogger) {
        rclnodejsBinding.addRosoutSublogger = () => {};
      }
      const stub = sandbox
        .stub(rclnodejsBinding, 'addRosoutSublogger')
        .returns(true);

      const child = logger.getChild('child');

      assert.strictEqual(child.name, 'parent.child');
      assert.ok(stub.calledWith('parent', 'child'));
    });

    it('destroy calls removeRosoutSublogger', function () {
      const logger = new Logging('parent');
      rclnodejsBinding.addRosoutSublogger = () => true;
      rclnodejsBinding.removeRosoutSublogger = () => {};

      const removeStub = sandbox.stub(
        rclnodejsBinding,
        'removeRosoutSublogger'
      );

      const child = logger.getChild('child');
      child.destroy();

      assert.ok(removeStub.calledWith('parent', 'child'));
    });

    it('destroy does nothing if no parent', function () {
      const logger = new Logging('solo');
      // ensure removeRosoutSublogger is spy
      if (!rclnodejsBinding.removeRosoutSublogger)
        rclnodejsBinding.removeRosoutSublogger = () => {};
      const removeStub = sandbox.stub(
        rclnodejsBinding,
        'removeRosoutSublogger'
      );

      logger.destroy(); // Should do nothing
      assert.strictEqual(removeStub.called, false);
    });

    it('configure calls binding', function () {
      const stub = sandbox.stub(rclnodejsBinding, 'loggingConfigure');
      // Create a fake Context instance
      // We can use Object.create(Context.prototype) to look like instance
      const fakeCtx = Object.create(Context.prototype);
      Object.defineProperty(fakeCtx, 'handle', { value: 'ctx-handle' });

      Logging.configure(fakeCtx);
      assert.ok(stub.calledWith('ctx-handle'));
    });

    it('configure throws on bad context', function () {
      assert.throws(() => {
        Logging.configure({});
      }, /context.*Context/);
    });

    it('shutdown calls binding', function () {
      const stub = sandbox.stub(rclnodejsBinding, 'loggingFini');
      Logging.shutdown();
      assert.ok(stub.calledOnce);
    });

    it('getLoggingDirectory calls binding', function () {
      const stub = sandbox
        .stub(rclnodejsBinding, 'getLoggingDirectory')
        .returns('/logs');
      assert.strictEqual(Logging.getLoggingDirectory(), '/logs');
    });
  });
});
