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

'use strict';

const assert = require('assert');
const DistroUtils = require('../lib/distro.js');
const rclnodejs = require('../index.js');
const TypeDescriptionService = require('../lib/type_description_service.js');

describe('type description service test suite', function () {
  this.timeout(60 * 1000);

  beforeEach(function () {
    return rclnodejs.init();
  });

  afterEach(function () {
    rclnodejs.shutdown();
  });

  it('Test type description service', function (done) {
    if (DistroUtils.getDistroId() > DistroUtils.getDistroId('humble')) {
      this.skip();
      return;
    }
    // Create a node and a publisher
    const nodeName = 'test_type_description_service';
    const node = rclnodejs.createNode(nodeName);
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
    client.sendRequest(request, (response) => {
      assert.strictEqual(response.successful, true);
      assert.strictEqual(
        response.type_description.type_description.type_name,
        topicType
      );
      assert.notStrictEqual(response.type_sources.length, 0);
      done();
    });
    rclnodejs.spin(node);
  });
});
