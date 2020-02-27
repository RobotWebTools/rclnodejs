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
const assertUtils = require('./utils.js');
const assertThrowsError = assertUtils.assertThrowsError;

describe('Node destroy testing', function() {
  this.timeout(60 * 1000);

  it('rclnodejs.init()', function(done) {
    rclnodejs
      .init()
      .then(function() {
        assert.ok(true);
        rclnodejs.shutdown();
        done();
      })
      .catch(function(err) {
        assert.ok(false);
        done(err);
      });
  });

  it('rclnodejs.init() & rclnodejs.shutdown()', function(done) {
    assert.deepStrictEqual(rclnodejs.isShutdown(), true);
    rclnodejs
      .init()
      .then(function() {
        assert.deepStrictEqual(rclnodejs.isShutdown(), false);
        rclnodejs.shutdown();
        assert.deepStrictEqual(rclnodejs.isShutdown(), true);
        done();
      })
      .catch(function(err) {
        assert.ok(false);
        done(err);
      });
  });

  it('rclnodejs init shutdown sequence', function(done) {
    rclnodejs
      .init()
      .then(function() {
        rclnodejs.shutdown();
        assert.ok(true);
      })
      .then(function() {
        assert.ok(true);
        return rclnodejs.init();
      })
      .then(function() {
        assert.doesNotThrow(function() {
          rclnodejs.shutdown();
        });
        assert.ok(true);
        done();
      })
      .catch(function(err) {
        assert.ok(false);
        done(err);
      });
  });

  it('rclnodejs.init(argv)', function(done) {
    rclnodejs
      .init(rclnodejs.Context.defaultContext(), ['a', 'b'])
      .then(function() {
        assert.ok(true);
        rclnodejs.shutdown();
        done();
      })
      .catch(function(err) {
        assert.ok(false);
        done(err);
      });
  });

  it('rclnodejs.init(argv) - invalid argv', function(done) {
    rclnodejs
      .init(rclnodejs.Context.defaultContext(), 'foobar')
      .then(function() {
        assert.ok(false);
        rclnodejs.shutdown();
        done();
      })
      // eslint-disable-next-line handle-callback-err
      .catch(function(_err) {
        assert.ok(true);
        done();
      });
  });

  it('rclnodejs.init(argv) with null argv elements', function(done) {
    rclnodejs
      .init(rclnodejs.Context.defaultContext(), ['a', null, 'b'])
      .then(function() {
        assert.ok(false);
        done();
      })
      // eslint-disable-next-line handle-callback-err
      .catch(function(err) {
        assert.ok(true);
        done();
      });
  });

  it('rclnodejs double init', function(done) {
    rclnodejs
      .init()
      .then(function() {
        assert.ok(true);
      })
      .then(function() {
        return rclnodejs.init();
      })
      .catch(function(err) {
        assert.notDeepStrictEqual(err, null);
        rclnodejs.shutdown();
        done();
      });
  });

  it('rclnodejs double shutdown', function(done) {
    rclnodejs
      .init()
      .then(function() {
        rclnodejs.shutdown();
        assert.ok(true);
      })
      .then(function() {
        assert.throws(function() {
          rclnodejs.shutdown();
        });
        assert.ok(true);
        done();
      })
      .catch(function(err) {
        assert.ok(true);
        done(err);
      });
  });

  it('rclnodejs create node without init', function() {
    assert.throws(function() {
      rclnodejs.createNode('my_node');
    });
  });
});
