'use strict';

const assert = require('assert');
const sinon = require('sinon');
const rclnodejsBinding = require('../lib/native_loader.js');
const Logging = require('../lib/logging.js'); // Uses native binding relative require

describe('Logging unit testing', function () {
  let sandbox;
  const mockContext = {
    handle: 'mock-context-handle',
    constructor: { name: 'Context' },
  };
  // Mock 'instanceof Context' is hard if we don't import real Context.
  // So we might need to import Context to use it in test or mock the instanceof check.
  const Context = require('../lib/context.js');

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
      assert.strictEqual(fileName, 'test-logging-unit.js');
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
