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

describe('Spin testing', function() {
  var node;
  this.timeout(60 * 1000);

  before(function() {
    return rclnodejs.init();
  });

  after(function() {
    rclnodejs.shutdown();
  });

  beforeEach(function() {
    node = rclnodejs.createNode('spin_node');
  });

  afterEach(function() {
    node.destroy();
  });

  it('rclnodejs.spin()', function() {
    rclnodejs.spin(node);
  });

  it('rclnodejs.spinOnce()', function() {
    rclnodejs.spinOnce(node);
  });

  it('rclnodejs.spinOnce() throws when already spinning', function() {
    rclnodejs.spin(node);
    assert.throws(function() {
      rclnodejs.spinOnce(node);
    });
  });
});
