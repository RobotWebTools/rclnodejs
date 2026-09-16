// Copyright (c) 2025, The Robot Web Tools Contributors
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
import DistroUtils from '../lib/distro.js';
import rclnodejs from '../index.js';
import TypeDescriptionService from '../lib/type_description_service.js';
import native from '../lib/native_loader.js';
import sinon from 'sinon';
import { exec } from 'child_process';

describe('type description service test suite', function () {
  this.timeout(60 * 1000);
  let node;
  let requestController;

  before(function () {
    if (DistroUtils.getDistroId() <= DistroUtils.getDistroId('humble')) {
      this.skip();
    }
  });

  beforeEach(async function () {
    requestController = new AbortController();
    await rclnodejs.init();
    const nodeName = 'test_type_description_service';
    node = rclnodejs.createNode(nodeName);
    rclnodejs.spin(node);
  });

  afterEach(function () {
    requestController.abort();
    rclnodejs.shutdown();
  });

  it('Test type description service', async function () {
    // Create a publisher
    const topic = 'test_get_type_description_publisher';
    const topicType = 'std_msgs/msg/String';
    node.createPublisher(topicType, topic);
    const infos = node.getPublishersInfoByTopic(
      '/test_get_type_description_publisher',
      false
    );
    assert.strictEqual(infos.length, 1);

    // Create a client to get the type description.
    const request = {
      type_name: topicType,
      type_hash: TypeDescriptionService.toTypeHash(infos[0].topic_type_hash),
      include_type_sources: true,
    };
    const serviceName = '/test_type_description_service/get_type_description';
    const GetTypeDescription =
      'type_description_interfaces/srv/GetTypeDescription';
    const client = node.createClient(GetTypeDescription, serviceName);
    const result = await client.waitForService(5000);
    if (!result) {
      throw new Error('Service not available');
    }

    const maxAttempts = 3;
    let response;
    for (let attempt = 1; attempt <= maxAttempts; attempt++) {
      try {
        response = await client.sendRequestAsync(request, {
          timeout: 10000,
          signal: requestController.signal,
        });
        break;
      } catch (error) {
        if (
          !(error instanceof rclnodejs.TimeoutError) ||
          attempt === maxAttempts
        ) {
          throw error;
        }
      }
    }

    assert.strictEqual(response.successful, true);
    assert.strictEqual(
      response.type_description.type_description.type_name,
      topicType
    );
    assert.notStrictEqual(response.type_sources.length, 0);
  });

  for (const sendFails of [false, true]) {
    it(`preserves and frees the native type-description response when sending ${sendFails ? 'fails' : 'succeeds'}`, function () {
      const topicType = 'std_msgs/msg/String';
      node.createPublisher(topicType, 'native_type_description_response');
      const [publisher] = node.getPublishersInfoByTopic(
        '/native_type_description_response'
      );
      const request = {
        type_name: topicType,
        type_hash: TypeDescriptionService.toTypeHash(publisher.topic_type_hash),
        include_type_sources: true,
      };
      const descriptionService = node._typeDescriptionService;
      const sandbox = sinon.createSandbox();
      const handleRequest = native.handleRequest;
      const sendError = new Error('response send failed');
      let nativeResponse;

      try {
        sandbox
          .stub(native, 'handleRequest')
          .callsFake((nodeHandle, rawRequest, rawResponse) => {
            handleRequest(nodeHandle, rawRequest, rawResponse);
            nativeResponse = Buffer.from(rawResponse);
          });
        const sendResponse = sandbox
          .stub(native, 'sendResponse')
          .callsFake((serviceHandle, rawResponse) => {
            assert.ok(
              rawResponse.equals(nativeResponse),
              'Send the native response without replacing its string buffers'
            );
            if (sendFails) {
              throw sendError;
            }
          });
        const destroyResponse = sandbox.spy(
          descriptionService._typeClass.Response,
          'destroyRawROS'
        );
        const invoke = () =>
          descriptionService._typeDescriptionService._callback(request, {
            _header: {},
          });

        if (sendFails) {
          assert.throws(invoke, (error) => error === sendError);
        } else {
          assert.strictEqual(invoke(), null);
        }
        assert.strictEqual(sendResponse.callCount, 1);
        assert.strictEqual(destroyResponse.callCount, 1);
      } finally {
        sandbox.restore();
      }
    });
  }

  it('Test type description service configured by parameter', function (done) {
    if (process.platform === 'win32') {
      this.skip();
    }

    // ROS 2 graph discovery is asynchronous: there is no guarantee
    // that an external `ros2` CLI process will see this node
    // immediately after rclnodejs.spin() returns. A fixed setTimeout
    // is racy on slower runners (notably the Rolling lane). Poll
    // instead, with a generous overall budget.
    waitForRos2Cli(
      'ros2 param list /test_type_description_service',
      (stdout) => stdout.includes('start_type_description_service'),
      done,
      "'start_type_description_service' not found in stdout."
    );
  });

  it('Test start_type_description_service parameter value', function (done) {
    if (process.platform === 'win32') {
      this.skip();
    }

    waitForRos2Cli(
      'ros2 param get /test_type_description_service start_type_description_service',
      (stdout) => stdout.includes('Boolean value is: True'),
      done,
      "'start_type_description_service param value' not found in stdout."
    );
  });
});

// Run a `ros2 ...` CLI command repeatedly until either `predicate(stdout)`
// returns true (success) or the overall budget runs out. Treats
// `Node not found` and other transient failures as retryable while the
// graph is still propagating; surfaces the last error otherwise.
function waitForRos2Cli(
  cmd,
  predicate,
  done,
  notFoundMessage,
  { timeoutMs = 15000, intervalMs = 500 } = {}
) {
  const deadline = Date.now() + timeoutMs;
  let lastErr = null;
  let lastStdout = '';
  let lastStderr = '';

  const tick = () => {
    exec(cmd, (error, stdout, stderr) => {
      lastErr = error;
      lastStdout = stdout || '';
      lastStderr = stderr || '';

      if (!error && !stderr && predicate(lastStdout)) {
        return done();
      }

      if (Date.now() < deadline) {
        return setTimeout(tick, intervalMs);
      }

      // Timed out. Prefer the most informative failure: the predicate
      // mismatch (we got a CLI response but the wrong content) wins
      // over a transient discovery error.
      if (!error && !stderr) {
        return done(new Error(`${notFoundMessage}\nstdout: ${lastStdout}`));
      }
      done(
        new Error(
          `\`${cmd}\` did not succeed within ${timeoutMs}ms. ` +
            `Last error: ${lastErr}, last stderr: ${lastStderr}`
        )
      );
    });
  };

  tick();
}
