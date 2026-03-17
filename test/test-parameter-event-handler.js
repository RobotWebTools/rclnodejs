// Copyright (c) 2026 Wayne Parrott. All rights reserved.
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

describe('ParameterEventHandler tests', function () {
  this.timeout(60 * 1000);

  let targetNode;
  let handlerNode;
  let handler;

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(function () {
    targetNode = rclnodejs.createNode('peh_target_node');
    handlerNode = rclnodejs.createNode('peh_handler_node');

    targetNode.declareParameter(
      new rclnodejs.Parameter(
        'test_int',
        rclnodejs.ParameterType.PARAMETER_INTEGER,
        BigInt(42)
      )
    );

    targetNode.declareParameter(
      new rclnodejs.Parameter(
        'test_string',
        rclnodejs.ParameterType.PARAMETER_STRING,
        'hello'
      )
    );

    rclnodejs.spin(targetNode);
    rclnodejs.spin(handlerNode);
  });

  afterEach(function () {
    if (handler && !handler.isDestroyed()) {
      handler.destroy();
    }
    handler = null;
    targetNode.destroy();
    handlerNode.destroy();
  });

  describe('Constructor', function () {
    it('should create a ParameterEventHandler via node method', function () {
      handler = handlerNode.createParameterEventHandler();
      assert.ok(handler);
      assert.strictEqual(handler.isDestroyed(), false);
    });

    it('should create a ParameterEventHandler directly', function () {
      handler = new rclnodejs.ParameterEventHandler(handlerNode);
      assert.ok(handler);
      assert.strictEqual(handler.isDestroyed(), false);
    });

    it('should throw for invalid node', function () {
      assert.throws(() => {
        new rclnodejs.ParameterEventHandler(null);
      });
      assert.throws(() => {
        new rclnodejs.ParameterEventHandler({});
      });
    });
  });

  describe('addParameterCallback', function () {
    it('should add a parameter callback and return a handle', function () {
      handler = handlerNode.createParameterEventHandler();
      const handle = handler.addParameterCallback(
        'test_int',
        '/peh_target_node',
        () => {}
      );
      assert.ok(handle);
      assert.strictEqual(handle.parameterName, 'test_int');
    });

    it('should throw for invalid arguments', function () {
      handler = handlerNode.createParameterEventHandler();

      assert.throws(() => {
        handler.addParameterCallback('', '/node', () => {});
      });
      assert.throws(() => {
        handler.addParameterCallback('param', '', () => {});
      });
      assert.throws(() => {
        handler.addParameterCallback('param', '/node', 'not a function');
      });
    });

    it('should receive callback when watched parameter changes', function (done) {
      handler = handlerNode.createParameterEventHandler();

      handler.addParameterCallback(
        'test_int',
        '/peh_target_node',
        (parameter) => {
          assert.strictEqual(parameter.name, 'test_int');
          done();
        }
      );

      // Wait briefly for subscription to be established, then change parameter
      setTimeout(() => {
        targetNode.setParameter(
          new rclnodejs.Parameter(
            'test_int',
            rclnodejs.ParameterType.PARAMETER_INTEGER,
            BigInt(99)
          )
        );
      }, 500);
    });

    it('should not fire callback for unrelated parameters', function (done) {
      handler = handlerNode.createParameterEventHandler();

      let callbackFired = false;
      handler.addParameterCallback('test_int', '/peh_target_node', () => {
        callbackFired = true;
      });

      // Change a different parameter
      setTimeout(() => {
        targetNode.setParameter(
          new rclnodejs.Parameter(
            'test_string',
            rclnodejs.ParameterType.PARAMETER_STRING,
            'world'
          )
        );
      }, 500);

      // Verify callback was NOT fired
      setTimeout(() => {
        assert.strictEqual(callbackFired, false);
        done();
      }, 2000);
    });

    it('should not fire callback for unrelated nodes', function (done) {
      handler = handlerNode.createParameterEventHandler();

      let callbackFired = false;
      handler.addParameterCallback('test_int', '/some_other_node', () => {
        callbackFired = true;
      });

      setTimeout(() => {
        targetNode.setParameter(
          new rclnodejs.Parameter(
            'test_int',
            rclnodejs.ParameterType.PARAMETER_INTEGER,
            BigInt(99)
          )
        );
      }, 500);

      setTimeout(() => {
        assert.strictEqual(callbackFired, false);
        done();
      }, 2000);
    });
  });

  describe('removeParameterCallback', function () {
    it('should remove a callback', function () {
      handler = handlerNode.createParameterEventHandler();
      const handle = handler.addParameterCallback(
        'test_int',
        '/peh_target_node',
        () => {}
      );
      handler.removeParameterCallback(handle);
    });

    it('should throw when removing non-existent callback', function () {
      handler = handlerNode.createParameterEventHandler();
      const handle = handler.addParameterCallback(
        'test_int',
        '/peh_target_node',
        () => {}
      );
      handler.removeParameterCallback(handle);

      assert.throws(() => {
        handler.removeParameterCallback(handle);
      });
    });

    it('should not fire removed callback', function (done) {
      handler = handlerNode.createParameterEventHandler();

      let callbackFired = false;
      const handle = handler.addParameterCallback(
        'test_int',
        '/peh_target_node',
        () => {
          callbackFired = true;
        }
      );

      handler.removeParameterCallback(handle);

      setTimeout(() => {
        targetNode.setParameter(
          new rclnodejs.Parameter(
            'test_int',
            rclnodejs.ParameterType.PARAMETER_INTEGER,
            BigInt(99)
          )
        );
      }, 500);

      setTimeout(() => {
        assert.strictEqual(callbackFired, false);
        done();
      }, 2000);
    });
  });

  describe('addParameterEventCallback', function () {
    it('should add an event callback and return a handle', function () {
      handler = handlerNode.createParameterEventHandler();
      const handle = handler.addParameterEventCallback(() => {});
      assert.ok(handle);
    });

    it('should throw for non-function callback', function () {
      handler = handlerNode.createParameterEventHandler();
      assert.throws(() => {
        handler.addParameterEventCallback('not a function');
      });
    });

    it('should receive all parameter events', function (done) {
      handler = handlerNode.createParameterEventHandler();

      handler.addParameterEventCallback((event) => {
        assert.ok(event.node);
        done();
      });

      setTimeout(() => {
        targetNode.setParameter(
          new rclnodejs.Parameter(
            'test_int',
            rclnodejs.ParameterType.PARAMETER_INTEGER,
            BigInt(99)
          )
        );
      }, 500);
    });
  });

  describe('removeParameterEventCallback', function () {
    it('should remove an event callback', function () {
      handler = handlerNode.createParameterEventHandler();
      const handle = handler.addParameterEventCallback(() => {});
      handler.removeParameterEventCallback(handle);
    });

    it('should throw when removing non-existent event callback', function () {
      handler = handlerNode.createParameterEventHandler();
      const handle = handler.addParameterEventCallback(() => {});
      handler.removeParameterEventCallback(handle);

      assert.throws(() => {
        handler.removeParameterEventCallback(handle);
      });
    });
  });

  describe('static methods', function () {
    it('getParameterFromEvent should find matching parameter', function () {
      const event = {
        node: '/peh_target_node',
        new_parameters: [{ name: 'param_a', value: {} }],
        changed_parameters: [{ name: 'param_b', value: {} }],
        deleted_parameters: [],
      };

      const found = rclnodejs.ParameterEventHandler.getParameterFromEvent(
        event,
        'param_b',
        '/peh_target_node'
      );
      assert.ok(found);
      assert.strictEqual(found.name, 'param_b');
    });

    it('getParameterFromEvent should return null for non-matching node', function () {
      const event = {
        node: '/other_node',
        new_parameters: [{ name: 'param_a', value: {} }],
        changed_parameters: [],
        deleted_parameters: [],
      };

      const found = rclnodejs.ParameterEventHandler.getParameterFromEvent(
        event,
        'param_a',
        '/peh_target_node'
      );
      assert.strictEqual(found, null);
    });

    it('getParametersFromEvent should return new + changed', function () {
      const event = {
        node: '/node',
        new_parameters: [{ name: 'a' }],
        changed_parameters: [{ name: 'b' }, { name: 'c' }],
        deleted_parameters: [{ name: 'd' }],
      };

      const params =
        rclnodejs.ParameterEventHandler.getParametersFromEvent(event);
      assert.strictEqual(params.length, 3);
    });
  });

  describe('destroy', function () {
    it('should destroy cleanly', function () {
      handler = handlerNode.createParameterEventHandler();
      handler.destroy();
      assert.strictEqual(handler.isDestroyed(), true);
    });

    it('should be idempotent', function () {
      handler = handlerNode.createParameterEventHandler();
      handler.destroy();
      handler.destroy(); // should not throw
      assert.strictEqual(handler.isDestroyed(), true);
    });

    it('should throw when adding callback after destroy', function () {
      handler = handlerNode.createParameterEventHandler();
      handler.destroy();

      assert.throws(() => {
        handler.addParameterCallback('param', '/node', () => {});
      });

      assert.throws(() => {
        handler.addParameterEventCallback(() => {});
      });
    });

    it('should be destroyed via node method', function () {
      handler = handlerNode.createParameterEventHandler();
      handlerNode.destroyParameterEventHandler(handler);
      assert.strictEqual(handler.isDestroyed(), true);
      handler = null; // prevent afterEach from double-destroying
    });
  });
});
