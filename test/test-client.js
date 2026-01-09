'use strict';

const assert = require('assert');
const sinon = require('sinon');
const rclnodejsBinding = require('../lib/native_loader.js');
const Client = require('../lib/client.js');
const DistroUtils = require('../lib/distro.js');

describe('Client coverage testing', function () {
  let sandbox;
  let mockHandle = { _handle: 'mock-handle' };
  let mockNodeHandle = { _handle: 'mock-node-handle' };
  let originalAbortSignalAny;

  const MockTypeClass = {
    type: () => ({
      interfaceName: 'AddTwoInts',
      pkgName: 'example_interfaces',
    }),
    Request: class MockRequest {
      constructor(data) {
        Object.assign(this, data);
      }
      serialize() {
        return new Uint8Array([0]);
      }
    },
    Response: class MockResponse {
      constructor(data) {
        Object.assign(this, data);
      }
      toPlainObject() {
        return { sum: 3 };
      }
    },
  };

  beforeEach(function () {
    sandbox = sinon.createSandbox();

    // Stub native bindings
    sandbox.stub(rclnodejsBinding, 'sendRequest').returns(12345); // Sequence number
    sandbox.stub(rclnodejsBinding, 'serviceServerIsAvailable').returns(true);
    sandbox
      .stub(rclnodejsBinding, 'getClientServiceName')
      .returns('test_service');
    sandbox.stub(rclnodejsBinding, 'createClient').returns(mockHandle);
    sandbox.stub(rclnodejsBinding, 'getNodeLoggerName').returns('node_logger');
    sandbox
      .stub(rclnodejsBinding, 'configureClientIntrospection')
      .returns(undefined);
  });

  afterEach(function () {
    sandbox.restore();
  });

  it('constructor sets properties correctly', function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      'test_service',
      MockTypeClass,
      {
        validateRequests: true,
        validationOptions: { strict: false },
      }
    );

    assert.strictEqual(client.willValidateRequest, true);
    assert.strictEqual(client._serviceName, 'test_service');
  });

  it('createClient static method', function () {
    const client = Client.createClient(
      mockNodeHandle,
      'service',
      MockTypeClass,
      { qos: {} }
    );
    assert.ok(client instanceof Client);
    assert.ok(rclnodejsBinding.createClient.called);
  });

  it('willValidateRequest setter', function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    client.willValidateRequest = true;
    assert.strictEqual(client.willValidateRequest, true);
  });

  it('setValidation updates options', function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    client.setValidation({ strict: false });
    assert.strictEqual(client._validationOptions.strict, false);
  });

  it('sendRequest throws on invalid callback', function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    assert.throws(() => {
      client.sendRequest({}, null);
    }, /callback/);
  });

  it('sendRequest sends and stores callback', function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    const spyCallback = sinon.spy();

    client.sendRequest({ a: 1 }, spyCallback);

    assert.ok(rclnodejsBinding.sendRequest.called);
    assert.ok(client._sequenceNumberToCallbackMap.has(12345));
  });

  it('processResponse calls callback', function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    const spyCallback = sinon.spy();

    // Simulate sending
    client.sendRequest({ a: 1 }, spyCallback);

    // Simulate receiving
    const mockResponseWrapper = new MockTypeClass.Response();
    client.processResponse(12345, mockResponseWrapper);

    assert.ok(spyCallback.calledOnce);
    assert.deepStrictEqual(spyCallback.firstCall.args[0], { sum: 3 });
    assert.strictEqual(client._sequenceNumberToCallbackMap.has(12345), false);
  });

  it('processResponse fails gracefully for unknown sequence', function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    // Should verify it logs debug info, but hard to capture 'debug' module.
    // We assume it runs without error.
    client.processResponse(99999, {});
  });

  it('sendRequestAsync resolves on success', async function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    const promise = client.sendRequestAsync({ a: 1 });

    // Simulate response arrival manually since we don't have the event loop of rclnodejs running
    // We need to trigger the callback registered in the map
    const callback = client._sequenceNumberToCallbackMap.get(12345);
    assert.ok(callback);

    callback(new MockTypeClass.Response());

    const result = await promise;
    assert.deepStrictEqual(result, new MockTypeClass.Response());
  });

  it('sendRequestAsync handles timeout', async function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );

    // We need to avoid waiting actual time.
    // mocking AbortSignal.timeout is hard, usually implemented by node.
    // We'll use a short timeout.

    try {
      await client.sendRequestAsync({ a: 1 }, { timeout: 10 });
      assert.fail('Should have timed out');
    } catch (err) {
      assert.ok(
        err.name === 'TimeoutError' ||
          err.code === 'ETIMEDOUT' ||
          err.message.includes('Timeout'),
        'Error was: ' + err
      );
    }
  });

  it('sendRequestAsync handles manual abort', async function () {
    if (typeof AbortController === 'undefined') this.skip();

    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    const ac = new AbortController();

    const promise = client.sendRequestAsync({ a: 1 }, { signal: ac.signal });
    ac.abort();

    try {
      await promise;
      assert.fail('Should have aborted');
    } catch (err) {
      // Name might vary depending on polyfill or native
      assert.ok(err.name === 'AbortError', 'Error name ' + err.name);
    }
  });

  it('waitForService waits and returns true', async function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    rclnodejsBinding.serviceServerIsAvailable.returns(true);

    const available = await client.waitForService(100);
    assert.strictEqual(available, true);
  });

  it('waitForService handles timeout', async function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    rclnodejsBinding.serviceServerIsAvailable.returns(false);

    const start = Date.now();
    const available = await client.waitForService(50);
    const duration = Date.now() - start;

    assert.strictEqual(available, false);
    assert.ok(duration >= 40); // Allows some slack
  });

  it('loggerName getter', function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    assert.strictEqual(client.loggerName, 'node_logger');
  });

  it('configureIntrospection warns on old distro', function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    const clockStub = { handle: 'clock' };
    const qos = {};

    // Mock DistroUtils.getDistroId to return humble (enum numeric) vs old
    // We stub getDistroId on the class
    sandbox.stub(DistroUtils, 'getDistroId').callsFake((name) => {
      if (name === 'humble') return 2;
      return 1; // Current distro is 1 (older than 2)
    });

    const consoleSpy = sandbox.spy(console, 'warn');

    client.configureIntrospection(clockStub, qos, 'on');

    assert.ok(consoleSpy.calledWithMatch(/not supported/));
    assert.strictEqual(
      rclnodejsBinding.configureClientIntrospection.called,
      false
    );
  });

  it('configureIntrospection calls binding on new distro', function () {
    const client = new Client(
      mockHandle,
      mockNodeHandle,
      's',
      MockTypeClass,
      {}
    );
    const clockStub = { handle: 'clock' };
    const qos = {};

    sandbox.stub(DistroUtils, 'getDistroId').callsFake((name) => {
      if (name === 'humble') return 1;
      return 2; // Current distro is 2 (newer than 1)
    });

    client.configureIntrospection(clockStub, qos, 'on');

    assert.strictEqual(
      rclnodejsBinding.configureClientIntrospection.called,
      true
    );
  });
});
