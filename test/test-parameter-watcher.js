// Copyright (c) 2025 Mahmoud Alghalayini. All rights reserved.
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
import rclnodejs from '../index.js';

describe('ParameterWatcher tests', function () {
  this.timeout(60 * 1000);

  let targetNode;
  let clientNode;
  let watcher;

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(async function () {
    targetNode = rclnodejs.createNode('target_node');
    clientNode = rclnodejs.createNode('client_node');

    targetNode.declareParameter(
      new rclnodejs.Parameter(
        'test_int',
        rclnodejs.ParameterType.PARAMETER_INTEGER,
        BigInt(42)
      )
    );
    targetNode.declareParameter(
      new rclnodejs.Parameter(
        'test_double',
        rclnodejs.ParameterType.PARAMETER_DOUBLE,
        3.14
      )
    );
    targetNode.declareParameter(
      new rclnodejs.Parameter(
        'test_string',
        rclnodejs.ParameterType.PARAMETER_STRING,
        'hello'
      )
    );

    watcher = clientNode.createParameterWatcher('target_node', [
      'test_int',
      'test_double',
    ]);

    rclnodejs.spin(targetNode);
    rclnodejs.spin(clientNode);

    await watcher.start(5000);
  });

  afterEach(function () {
    try {
      if (watcher && !watcher.isDestroyed()) {
        watcher.destroy();
      }
    } catch (e) {
      // Ignore cleanup errors
    }

    try {
      if (clientNode) {
        clientNode.stop();
        clientNode.destroy();
      }
    } catch (e) {
      // Ignore cleanup errors
    }

    try {
      if (targetNode) {
        targetNode.stop();
        targetNode.destroy();
      }
    } catch (e) {
      // Ignore cleanup errors
    }

    watcher = null;
    clientNode = null;
    targetNode = null;
  });

  describe('Constructor and properties', function () {
    it('should create ParameterWatcher with valid arguments', function () {
      const w = clientNode.createParameterWatcher('target_node', [
        'param1',
        'param2',
      ]);
      assert.strictEqual(w.remoteNodeName, 'target_node');
      assert.deepStrictEqual(w.watchedParameters, ['param1', 'param2']);
      w.destroy();
    });

    it('should throw error if remote node name is null', function () {
      assert.throws(() => {
        clientNode.createParameterWatcher(null, ['param1']);
      }, rclnodejs.TypeValidationError);
    });

    it('should throw error if remote node name is empty', function () {
      assert.throws(() => {
        clientNode.createParameterWatcher('', ['param1']);
      }, rclnodejs.TypeValidationError);
    });

    it('should throw error if parameter names is not an array', function () {
      assert.throws(() => {
        clientNode.createParameterWatcher('target_node', 'not_array');
      }, rclnodejs.TypeValidationError);
    });

    it('should throw error if parameter names is empty array', function () {
      assert.throws(() => {
        clientNode.createParameterWatcher('target_node', []);
      }, rclnodejs.TypeValidationError);
    });

    it('should normalize node name by removing leading slash', function () {
      const w = clientNode.createParameterWatcher('/target_node', ['param1']);
      assert.strictEqual(w.remoteNodeName, 'target_node');
      w.destroy();
    });
  });

  describe('start', function () {
    it('should start watching and return true when services available', async function () {
      const w = clientNode.createParameterWatcher('target_node', ['test_int']);
      const started = await w.start(5000);
      assert.strictEqual(started, true);
      w.destroy();
    });

    it('should return false when services not available', async function () {
      const w = clientNode.createParameterWatcher('nonexistent_node', [
        'param1',
      ]);
      const started = await w.start(1000);
      assert.strictEqual(started, false);
      w.destroy();
    });

    it('should throw error if watcher is destroyed', async function () {
      const w = clientNode.createParameterWatcher('target_node', ['test_int']);
      w.destroy();

      try {
        await w.start();
        assert.fail('Should have thrown error');
      } catch (error) {
        assert.ok(error.message.includes('destroyed'));
      }
    });
  });

  describe('change event', function () {
    it('should emit change event when watched parameter changes', function (done) {
      this.timeout(10000);

      watcher.on('change', (params) => {
        assert.ok(Array.isArray(params));
        assert.ok(params.length > 0);
        const changedParam = params.find((p) => p.name === 'test_int');
        assert.ok(changedParam);
        done();
      });

      setTimeout(() => {
        targetNode.setParameter(
          new rclnodejs.Parameter(
            'test_int',
            rclnodejs.ParameterType.PARAMETER_INTEGER,
            BigInt(100)
          )
        );
      }, 100);
    });

    it('should not emit change event for unwatched parameters', function (done) {
      this.timeout(10000);
      let changeCount = 0;

      watcher.on('change', (params) => {
        changeCount++;
        const hasUnwatched = params.some((p) => p.name === 'test_string');
        assert.strictEqual(hasUnwatched, false);
      });

      targetNode.setParameter(
        new rclnodejs.Parameter(
          'test_int',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          BigInt(200)
        )
      );

      setTimeout(() => {
        targetNode.setParameter(
          new rclnodejs.Parameter(
            'test_string',
            rclnodejs.ParameterType.PARAMETER_STRING,
            'changed'
          )
        );
      }, 100);

      setTimeout(() => {
        assert.strictEqual(changeCount, 1);
        done();
      }, 500);
    });

    it('should emit change event for multiple parameter changes', function (done) {
      this.timeout(10000);

      // Use once() to ensure done() is only called once
      watcher.once('change', (params) => {
        assert.ok(params.length >= 1);
        done();
      });

      setTimeout(() => {
        targetNode.setParameters([
          new rclnodejs.Parameter(
            'test_int',
            rclnodejs.ParameterType.PARAMETER_INTEGER,
            BigInt(111)
          ),
          new rclnodejs.Parameter(
            'test_double',
            rclnodejs.ParameterType.PARAMETER_DOUBLE,
            2.71
          ),
        ]);
      }, 100);
    });
  });

  describe('getCurrentValues', function () {
    it('should get current values of watched parameters', async function () {
      const values = await watcher.getCurrentValues();
      assert.ok(Array.isArray(values));
      assert.strictEqual(values.length, 2);

      const intParam = values.find((p) => p.name === 'test_int');
      const doubleParam = values.find((p) => p.name === 'test_double');

      assert.ok(intParam);
      assert.strictEqual(intParam.value, BigInt(42));
      assert.ok(doubleParam);
      assert.strictEqual(doubleParam.value, 3.14);
    });

    it('should throw error if watcher is destroyed', async function () {
      watcher.destroy();

      try {
        await watcher.getCurrentValues();
        assert.fail('Should have thrown error');
      } catch (error) {
        assert.ok(error.message.includes('destroyed'));
      }
    });

    it('should support timeout option', async function () {
      // Create watcher for non-existent node to ensure timeout occurs
      const slowWatcher = clientNode.createParameterWatcher(
        'nonexistent_slow_node',
        ['param1']
      );
      await slowWatcher.start(1000).catch(() => {}); // Services won't be available

      try {
        await slowWatcher.getCurrentValues({ timeout: 1 });
        assert.fail('Should have timed out');
      } catch (error) {
        assert.strictEqual(error.name, 'TimeoutError');
      } finally {
        slowWatcher.destroy();
      }
    });

    it('should support AbortSignal', async function () {
      // Create watcher for non-existent node to ensure abort can occur
      const slowWatcher = clientNode.createParameterWatcher(
        'nonexistent_slow_node',
        ['param1']
      );
      await slowWatcher.start(1000).catch(() => {}); // Services won't be available

      const controller = new AbortController();
      setTimeout(() => controller.abort(), 1);

      try {
        await slowWatcher.getCurrentValues({ signal: controller.signal });
        assert.fail('Should have been aborted');
      } catch (error) {
        assert.strictEqual(error.name, 'AbortError');
      } finally {
        slowWatcher.destroy();
      }
    });
  });

  describe('addParameter', function () {
    it('should add parameter to watch list', function () {
      const initialLength = watcher.watchedParameters.length;
      watcher.addParameter('test_string');

      assert.strictEqual(watcher.watchedParameters.length, initialLength + 1);
      assert.ok(watcher.watchedParameters.includes('test_string'));
    });

    it('should not duplicate parameters', function () {
      watcher.addParameter('test_int');
      const length = watcher.watchedParameters.length;
      watcher.addParameter('test_int');

      assert.strictEqual(watcher.watchedParameters.length, length);
    });

    it('should throw error if name is not a string', function () {
      assert.throws(() => {
        watcher.addParameter(123);
      }, rclnodejs.TypeValidationError);
    });

    it('should throw error if name is empty string', function () {
      assert.throws(() => {
        watcher.addParameter('');
      }, rclnodejs.TypeValidationError);
    });

    it('should throw error if watcher is destroyed', function () {
      watcher.destroy();

      assert.throws(() => {
        watcher.addParameter('test_string');
      }, /destroyed/);
    });

    it('should trigger change event after adding parameter', function (done) {
      this.timeout(10000);

      watcher.addParameter('test_string');

      watcher.on('change', (params) => {
        const hasNewParam = params.some((p) => p.name === 'test_string');
        if (hasNewParam) {
          done();
        }
      });

      setTimeout(() => {
        targetNode.setParameter(
          new rclnodejs.Parameter(
            'test_string',
            rclnodejs.ParameterType.PARAMETER_STRING,
            'updated'
          )
        );
      }, 100);
    });
  });

  describe('removeParameter', function () {
    it('should remove parameter from watch list', function () {
      const initialLength = watcher.watchedParameters.length;
      const removed = watcher.removeParameter('test_int');

      assert.strictEqual(removed, true);
      assert.strictEqual(watcher.watchedParameters.length, initialLength - 1);
      assert.ok(!watcher.watchedParameters.includes('test_int'));
    });

    it('should return false if parameter not in watch list', function () {
      const removed = watcher.removeParameter('nonexistent');
      assert.strictEqual(removed, false);
    });

    it('should throw error if watcher is destroyed', function () {
      watcher.destroy();

      assert.throws(() => {
        watcher.removeParameter('test_int');
      }, /destroyed/);
    });

    it('should not trigger change event after removing parameter', function (done) {
      this.timeout(10000);
      let changeCount = 0;

      watcher.removeParameter('test_int');

      watcher.on('change', (params) => {
        changeCount++;
        const hasRemoved = params.some((p) => p.name === 'test_int');
        assert.strictEqual(hasRemoved, false);
      });

      setTimeout(() => {
        targetNode.setParameter(
          new rclnodejs.Parameter(
            'test_int',
            rclnodejs.ParameterType.PARAMETER_INTEGER,
            BigInt(999)
          )
        );
      }, 100);

      setTimeout(() => {
        assert.strictEqual(changeCount, 0);
        done();
      }, 500);
    });
  });

  describe('Lifecycle management', function () {
    it('should destroy watcher', function () {
      const w = clientNode.createParameterWatcher('target_node', ['test_int']);
      w.destroy();
      assert.strictEqual(w.isDestroyed(), true);
    });

    it('should allow multiple destroy calls', function () {
      const w = clientNode.createParameterWatcher('target_node', ['test_int']);
      w.destroy();
      w.destroy();
      assert.strictEqual(w.isDestroyed(), true);
    });

    it('should auto-destroy when parent node is destroyed', function () {
      const node = rclnodejs.createNode('temp_node');
      const w = node.createParameterWatcher('target_node', ['test_int']);

      assert.strictEqual(w.isDestroyed(), false);
      node.destroy();
      assert.strictEqual(w.isDestroyed(), true);
    });

    it('should destroy via node.destroyParameterWatcher', function () {
      const node = rclnodejs.createNode('temp_node');
      const w = node.createParameterWatcher('target_node', ['test_int']);

      assert.strictEqual(w.isDestroyed(), false);
      node.destroyParameterWatcher(w);
      assert.strictEqual(w.isDestroyed(), true);

      node.destroy();
    });

    it('should throw error when calling methods on destroyed watcher', function () {
      watcher.destroy();

      assert.throws(() => {
        watcher.addParameter('param1');
      }, /destroyed/);

      assert.throws(() => {
        watcher.removeParameter('param1');
      }, /destroyed/);
    });

    it('should remove all event listeners on destroy', function () {
      const w = clientNode.createParameterWatcher('target_node', ['test_int']);
      w.on('change', () => {});
      w.on('change', () => {});

      assert.ok(w.listenerCount('change') > 0);
      w.destroy();
      assert.strictEqual(w.listenerCount('change'), 0);
    });
  });

  describe('watchedParameters getter', function () {
    it('should return array of watched parameter names', function () {
      const params = watcher.watchedParameters;
      assert.ok(Array.isArray(params));
      assert.strictEqual(params.length, 2);
      assert.ok(params.includes('test_int'));
      assert.ok(params.includes('test_double'));
    });

    it('should return a copy, not the original set', function () {
      const params1 = watcher.watchedParameters;
      const params2 = watcher.watchedParameters;

      assert.notStrictEqual(params1, params2);
      assert.deepStrictEqual(params1, params2);
    });

    it('should reflect changes after add/remove', function () {
      watcher.addParameter('new_param');
      assert.ok(watcher.watchedParameters.includes('new_param'));

      watcher.removeParameter('test_int');
      assert.ok(!watcher.watchedParameters.includes('test_int'));
    });
  });

  describe('remoteNodeName getter', function () {
    it('should return the remote node name', function () {
      assert.strictEqual(watcher.remoteNodeName, 'target_node');
    });
  });

  describe('Error cases', function () {
    it('should handle parameter events from other nodes', function (done) {
      this.timeout(10000);
      let changeCount = 0;

      watcher.on('change', () => {
        changeCount++;
      });

      const otherNode = rclnodejs.createNode('other_node');
      otherNode.declareParameter(
        new rclnodejs.Parameter(
          'test_int',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          BigInt(1)
        )
      );
      rclnodejs.spin(otherNode);

      setTimeout(() => {
        otherNode.setParameter(
          new rclnodejs.Parameter(
            'test_int',
            rclnodejs.ParameterType.PARAMETER_INTEGER,
            BigInt(2)
          )
        );
      }, 100);

      setTimeout(() => {
        assert.strictEqual(changeCount, 0);
        otherNode.destroy();
        done();
      }, 500);
    });

    it('should handle empty parameter event', function (done) {
      this.timeout(10000);
      let eventReceived = false;

      watcher.on('change', () => {
        eventReceived = true;
      });

      setTimeout(() => {
        assert.strictEqual(eventReceived, false);
        done();
      }, 500);
    });
  });

  describe('Integration with ParameterClient', function () {
    it('should use underlying ParameterClient correctly', async function () {
      const values = await watcher.getCurrentValues();
      assert.ok(Array.isArray(values));
      assert.strictEqual(values.length, 2);
    });

    it('should share timeout option with ParameterClient', async function () {
      // Use non-existent node to ensure timeout occurs
      const w = clientNode.createParameterWatcher(
        'nonexistent_timeout_node',
        ['test_int'],
        {
          timeout: 100,
        }
      );
      await w.start(1000).catch(() => {}); // Services won't be available

      try {
        await w.getCurrentValues({ timeout: 1 });
        assert.fail('Should have timed out');
      } catch (error) {
        assert.strictEqual(error.name, 'TimeoutError');
      } finally {
        w.destroy();
      }
    });
  });
});
