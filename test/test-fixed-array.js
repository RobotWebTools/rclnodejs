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

/* eslint-disable camelcase */
/* eslint-disable key-spacing */
/* eslint-disable comma-spacing */

describe('Test message which has a fixed array of 36', function() {
  this.timeout(60 * 1000);

  const mapData = {
    map: {
      header: {
        stamp: {
          sec: 123456,
          nanosec: 789,
        },
        frame_id: 'main_frame',
      },
      info: {
        map_load_time: {
          sec: 123456,
          nanosec: 789,
        },
        resolution: 1.0,
        width: 1024,
        height: 768,
        origin: {
          position: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
          },
        },
      },
      data: Int8Array.from([1, 2, 3]),
    },
    initial_pose: {
      header: {
        stamp: {
          sec: 123456,
          nanosec: 789,
        },
        frame_id: 'main frame',
      },
      pose: {
        pose: {
          position: { x: 11.5, y: 112.75, z: 9.0 },
          orientation: { x: 31.5, y: 21.5, z: 7.5, w: 1.5 },
        },
        covariance: Float64Array.from({ length: 36 }, (v, k) => k),
      },
    },
  };

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  it('Assigned with an array of 36', function(done) {
    const node = rclnodejs.createNode('set_map_client');
    node.createService(
      'nav_msgs/srv/SetMap',
      'set_map',
      (request, response) => {
        assert.deepStrictEqual(request, mapData);
        response.success = true;
        return response;
      }
    );

    rclnodejs.spin(node);
    const client = node.createClient('nav_msgs/srv/SetMap', 'set_map');
    client.sendRequest(mapData, response => {
      assert.deepStrictEqual(response.success, true);
      node.destroy();
      done();
    });
  });

  it('Assigned with a longer array', function(done) {
    mapData.initial_pose.pose.covariance = Float64Array.from(
      { length: 37 },
      (v, k) => k
    );
    const node = rclnodejs.createNode('set_map_client');
    const client = node.createClient('nav_msgs/srv/SetMap', 'set_map');
    assert.throws(() => {
      client.sendRequest(mapData, response => {});
    }, RangeError);
    node.destroy();
    done();
  });

  it('Assigned with a shorter array', function(done) {
    mapData.initial_pose.pose.covariance = Float64Array.from(
      { length: 35 },
      (v, k) => k
    );
    const node = rclnodejs.createNode('set_map_client');
    const client = node.createClient('nav_msgs/srv/SetMap', 'set_map');
    assert.throws(() => {
      client.sendRequest(mapData, response => {});
    }, RangeError);
    node.destroy();
    done();
  });
});
