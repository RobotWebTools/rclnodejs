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

'use strict';

const assert = require('assert');
const rclnodejs = require('../index.js');

describe('Client async functionality', function () {
  this.timeout(60 * 1000);

  let node;
  let client;
  let service;
  let pendingTimeouts = [];

  before(async function () {
    rclnodejs.init();
    node = rclnodejs.createNode('test_async_client');

    service = node.createService(
      'example_interfaces/srv/AddTwoInts',
      'test_add_service',
      (request, response) => {
        if (
          (request.a === 1n && request.b === 1n) ||
          (request.a === 0n && request.b === 0n)
        ) {
          const timeoutId = setTimeout(() => {
            try {
              const result = response.template;
              result.sum = request.a + request.b;
              response.send(result);
            } catch (error) {
              // Service may have been destroyed
            }
          }, 100);
          pendingTimeouts.push(timeoutId);
        } else if (request.a === 50n && request.b === 60n) {
          const timeoutId = setTimeout(() => {
            try {
              const result = response.template;
              result.sum = request.a + request.b;
              response.send(result);
            } catch (error) {
              // Service may have been destroyed
            }
          }, 50);
          pendingTimeouts.push(timeoutId);
        } else {
          const result = response.template;
          result.sum = request.a + request.b;
          response.send(result);
        }
      }
    );

    client = node.createClient(
      'example_interfaces/srv/AddTwoInts',
      'test_add_service'
    );

    rclnodejs.spin(node);

    const available = await client.waitForService(5000);
    assert.ok(available, 'Service should be available');
  });

  after(function () {
    pendingTimeouts.forEach((timeoutId) => clearTimeout(timeoutId));
    pendingTimeouts = [];
    rclnodejs.shutdown();
  });

  describe('sendRequestAsync', function () {
    it('should exist as a method', function () {
      assert.strictEqual(typeof client.sendRequestAsync, 'function');
    });

    it('should return a Promise', function () {
      const request = { a: BigInt(1), b: BigInt(2) };
      const result = client.sendRequestAsync(request);
      assert.ok(result instanceof Promise);

      result.catch(() => {});
    });

    it('should resolve with correct response', async function () {
      const request = { a: BigInt(10), b: BigInt(20) };
      const response = await client.sendRequestAsync(request);

      assert.strictEqual(typeof response, 'object');
      assert.strictEqual(response.sum, BigInt(30));
    });

    it('should work without options', async function () {
      const request = { a: BigInt(5), b: BigInt(7) };
      const response = await client.sendRequestAsync(request);

      assert.strictEqual(response.sum, BigInt(12));
    });

    it('should work with empty options', async function () {
      const request = { a: BigInt(3), b: BigInt(4) };
      const response = await client.sendRequestAsync(request, {});

      assert.strictEqual(response.sum, BigInt(7));
    });

    it('should handle timeout option', async function () {
      const request = { a: BigInt(15), b: BigInt(25) };

      const response = await client.sendRequestAsync(request, {
        timeout: 5000,
      });
      assert.strictEqual(response.sum, BigInt(40));
    });

    it('should timeout with very short timeout', async function () {
      const request = { a: BigInt(1), b: BigInt(1) };

      try {
        await client.sendRequestAsync(request, { timeout: 1 });
        assert.fail('Should have timed out');
      } catch (error) {
        assert.strictEqual(error.name, 'TimeoutError');
        assert.ok(error.message.includes('timeout'));
        assert.strictEqual(error.code, 'TIMEOUT');
      }
    });

    it('should handle AbortController cancellation', async function () {
      const controller = new AbortController();
      const request = { a: BigInt(100), b: BigInt(200) };

      controller.abort();

      try {
        await client.sendRequestAsync(request, { signal: controller.signal });
        assert.fail('Should have been aborted');
      } catch (error) {
        assert.strictEqual(error.name, 'AbortError');
        assert.ok(error.message.includes('aborted'));
      }
    });

    it('should handle AbortController cancellation during request', async function () {
      const controller = new AbortController();
      const request = { a: BigInt(50), b: BigInt(60) };

      setTimeout(() => controller.abort(), 10);

      try {
        await client.sendRequestAsync(request, {
          signal: controller.signal,
          timeout: 5000,
        });
        assert.fail('Should have been aborted');
      } catch (error) {
        assert.strictEqual(error.name, 'AbortError');
      }
    });

    it('should handle multiple concurrent requests', async function () {
      const requests = [
        { a: BigInt(1), b: BigInt(1) },
        { a: BigInt(2), b: BigInt(2) },
        { a: BigInt(3), b: BigInt(3) },
      ];

      const promises = requests.map((request) =>
        client.sendRequestAsync(request, { timeout: 5000 })
      );

      const responses = await Promise.all(promises);

      assert.strictEqual(responses.length, 3);
      assert.strictEqual(responses[0].sum, BigInt(2));
      assert.strictEqual(responses[1].sum, BigInt(4));
      assert.strictEqual(responses[2].sum, BigInt(6));
    });

    it('should clean up properly on success', async function () {
      const initialMapSize = client._sequenceNumberToCallbackMap.size;

      const request = { a: BigInt(8), b: BigInt(9) };
      await client.sendRequestAsync(request);

      assert.strictEqual(
        client._sequenceNumberToCallbackMap.size,
        initialMapSize
      );
    });

    it('should clean up properly on error', async function () {
      const initialMapSize = client._sequenceNumberToCallbackMap.size;

      const request = { a: BigInt(1), b: BigInt(1) };

      try {
        await client.sendRequestAsync(request, { timeout: 1 });
      } catch (error) {}

      assert.strictEqual(
        client._sequenceNumberToCallbackMap.size,
        initialMapSize
      );
    });

    it('should not interfere with existing sendRequest method', function (done) {
      const request = { a: BigInt(6), b: BigInt(7) };

      client.sendRequest(request, (response) => {
        assert.strictEqual(response.sum, BigInt(13));
        done();
      });
    });

    it('should work alongside callback method', async function () {
      const request1 = { a: BigInt(10), b: BigInt(11) };
      const request2 = { a: BigInt(12), b: BigInt(13) };

      const asyncPromise = client.sendRequestAsync(request1);

      const callbackPromise = new Promise((resolve) => {
        client.sendRequest(request2, (response) => {
          resolve(response);
        });
      });

      const [asyncResponse, callbackResponse] = await Promise.all([
        asyncPromise,
        callbackPromise,
      ]);

      assert.strictEqual(asyncResponse.sum, BigInt(21));
      assert.strictEqual(callbackResponse.sum, BigInt(25));
    });
  });

  describe('Error handling edge cases', function () {
    it('should handle invalid request objects gracefully', async function () {
      try {
        await client.sendRequestAsync(null);
        assert.fail('Should have thrown an error');
      } catch (error) {
        assert.ok(error instanceof Error);
      }
    });

    it('should handle zero and negative timeouts', async function () {
      // Skip this test on Node.js < 18.20 where AbortSignal.timeout(0) throws RangeError
      const [major, minor] = process.versions.node.split('.').map(Number);
      if (major < 18 || (major === 18 && minor < 20)) {
        this.skip();
      }

      const request = { a: BigInt(1), b: BigInt(1) };

      try {
        await client.sendRequestAsync(request, { timeout: 0 });
        assert.fail('Should have timed out immediately');
      } catch (error) {
        assert.strictEqual(error.name, 'TimeoutError');
      }
    });

    it('should ignore negative timeout values', async function () {
      const request = { a: BigInt(2), b: BigInt(3) };

      const response = await client.sendRequestAsync(request, { timeout: -1 });
      assert.strictEqual(response.sum, BigInt(5));
    });

    it('should handle already aborted signal', async function () {
      const controller = new AbortController();
      controller.abort();

      const request = { a: BigInt(1), b: BigInt(1) };

      try {
        await client.sendRequestAsync(request, { signal: controller.signal });
        assert.fail('Should have been aborted immediately');
      } catch (error) {
        assert.ok(error.message.includes('aborted'));
      }
    });
  });
});
