// Copyright (c) 2017 Intel Corporation. All rights reserved.
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

describe('Test creating a service with an async callback', function(){

  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  it('Node.createService() with async callback', function (done) {
    // this should expose issue #944
    var clientNode = rclnodejs.createNode('single_ps_client_async');
    var serviceNode = rclnodejs.createNode('single_ps_service_async');
    const AddTwoInts = 'example_interfaces/srv/AddTwoInts';

    var service = serviceNode.createService(
      AddTwoInts,
      'single_ps_channel2',
      async (request, response) => {
        assert.deepStrictEqual(request.a, 1);
        assert.deepStrictEqual(request.b, 2);
        let result = response.template;
        result.sum = request.a + request.b;
        // to trigger the bug, two conditions must hold: 
        //   - the response is send assynchronously(!) by the callback
        //     via side effect
        //   - the callback returns something unrelated to the
        //     actual response. For an async function, if I do not
        //     explicitly return anything, it will return a Promise resolving to
        //     undefined.
        setTimeout(()=>response.send(result), 0);
      }
    );
    var client = clientNode.createClient(AddTwoInts, 'single_ps_channel2');
    const request = { a: 1, b: 2 };

    var timer = clientNode.createTimer(100, () => {
      client.sendRequest(request, (response) => {
        timer.cancel();
        assert.deepStrictEqual(response.sum, 3);
        serviceNode.destroy();
        clientNode.destroy();
        done();
      });
    });
    rclnodejs.spin(serviceNode);
    rclnodejs.spin(clientNode);
  });
});
