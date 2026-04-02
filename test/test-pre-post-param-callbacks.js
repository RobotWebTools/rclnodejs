// Copyright (c) 2026 The Robot Web Tools Contributors. All rights reserved.
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

describe('Pre/Post set parameters callbacks', function () {
  let node;
  this.timeout(60 * 1000);

  before(function () {
    return rclnodejs.init();
  });

  after(function () {
    rclnodejs.shutdown();
  });

  beforeEach(function () {
    node = rclnodejs.createNode('pre_post_param_test_node');
    node.declareParameter(
      new rclnodejs.Parameter(
        'test_param',
        rclnodejs.ParameterType.PARAMETER_INTEGER,
        10
      )
    );
  });

  afterEach(function () {
    node.destroy();
  });

  describe('PreSetParametersCallback', function () {
    it('pre-set callback can modify parameter values', function () {
      // Coerce: double any integer value before it's set
      const preCallback = (params) =>
        params.map(
          (p) => new rclnodejs.Parameter(p.name, p.type, Number(p.value) * 2)
        );

      node.addPreSetParametersCallback(preCallback);
      node.setParameter(
        new rclnodejs.Parameter(
          'test_param',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          5
        )
      );

      const param = node.getParameter('test_param');
      assert.strictEqual(Number(param.value), 10); // 5 * 2 = 10
    });

    it('pre-set callback returning empty list rejects the set', function () {
      node.addPreSetParametersCallback(() => []);

      const result = node.setParameter(
        new rclnodejs.Parameter(
          'test_param',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          99
        )
      );

      assert.strictEqual(result.successful, false);
      assert.ok(result.reason.includes('empty'));

      // Value should remain unchanged
      const param = node.getParameter('test_param');
      assert.strictEqual(Number(param.value), 10);
    });

    it('multiple pre-set callbacks chain as a pipeline', function () {
      let callOrder = [];

      // First registered: multiply value by 3
      node.addPreSetParametersCallback((params) => {
        callOrder.push('first');
        return params.map(
          (p) => new rclnodejs.Parameter(p.name, p.type, Number(p.value) * 3)
        );
      });

      // Second registered (runs first due to LIFO): add 1 to value
      node.addPreSetParametersCallback((params) => {
        callOrder.push('second');
        return params.map(
          (p) => new rclnodejs.Parameter(p.name, p.type, Number(p.value) + 1)
        );
      });

      node.setParameter(
        new rclnodejs.Parameter(
          'test_param',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          5
        )
      );

      // LIFO order: second runs first, then first
      assert.deepStrictEqual(callOrder, ['second', 'first']);

      // Pipeline: input=5 → second: 5+1=6 → first: 6*3=18
      const param = node.getParameter('test_param');
      assert.strictEqual(Number(param.value), 18);
    });

    it('removePreSetParametersCallback removes the callback', function () {
      const preCallback = () => [];
      node.addPreSetParametersCallback(preCallback);
      node.removePreSetParametersCallback(preCallback);

      const result = node.setParameter(
        new rclnodejs.Parameter(
          'test_param',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          42
        )
      );

      assert.strictEqual(result.successful, true);
      assert.strictEqual(Number(node.getParameter('test_param').value), 42);
    });
  });

  describe('PostSetParametersCallback', function () {
    it('post-set callback is called after successful set', function () {
      let postCalled = false;
      let receivedParams = null;

      node.addPostSetParametersCallback((params) => {
        postCalled = true;
        receivedParams = params;
      });

      node.setParameter(
        new rclnodejs.Parameter(
          'test_param',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          42
        )
      );

      assert.ok(postCalled);
      assert.strictEqual(receivedParams.length, 1);
      assert.strictEqual(receivedParams[0].name, 'test_param');
      assert.strictEqual(Number(receivedParams[0].value), 42);
    });

    it('post-set callback is NOT called when on-set rejects', function () {
      let postCalled = false;

      node.addOnSetParametersCallback(() => ({
        successful: false,
        reason: 'rejected',
      }));
      node.addPostSetParametersCallback(() => {
        postCalled = true;
      });

      node.setParameter(
        new rclnodejs.Parameter(
          'test_param',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          99
        )
      );

      assert.strictEqual(postCalled, false);
    });

    it('multiple post-set callbacks all run', function () {
      let callOrder = [];
      node.addPostSetParametersCallback(() => callOrder.push('first'));
      node.addPostSetParametersCallback(() => callOrder.push('second'));

      node.setParameter(
        new rclnodejs.Parameter(
          'test_param',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          42
        )
      );

      // LIFO order
      assert.deepStrictEqual(callOrder, ['second', 'first']);
    });

    it('removePostSetParametersCallback removes the callback', function () {
      let postCalled = false;
      const postCallback = () => {
        postCalled = true;
      };

      node.addPostSetParametersCallback(postCallback);
      node.removePostSetParametersCallback(postCallback);

      node.setParameter(
        new rclnodejs.Parameter(
          'test_param',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          42
        )
      );

      assert.strictEqual(postCalled, false);
    });
  });

  describe('Full callback chain', function () {
    it('pre → on → post runs in order', function () {
      let callOrder = [];

      node.addPreSetParametersCallback((params) => {
        callOrder.push('pre');
        return params;
      });
      node.addOnSetParametersCallback((params) => {
        callOrder.push('on');
        return { successful: true };
      });
      node.addPostSetParametersCallback(() => {
        callOrder.push('post');
      });

      node.setParameter(
        new rclnodejs.Parameter(
          'test_param',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          42
        )
      );

      assert.deepStrictEqual(callOrder, ['pre', 'on', 'post']);
    });

    it('pre rejection stops on and post from running', function () {
      let callOrder = [];

      node.addPreSetParametersCallback(() => {
        callOrder.push('pre');
        return []; // reject
      });
      node.addOnSetParametersCallback(() => {
        callOrder.push('on');
        return { successful: true };
      });
      node.addPostSetParametersCallback(() => {
        callOrder.push('post');
      });

      node.setParameter(
        new rclnodejs.Parameter(
          'test_param',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          42
        )
      );

      assert.deepStrictEqual(callOrder, ['pre']);
    });

    it('on rejection stops post from running', function () {
      let callOrder = [];

      node.addPreSetParametersCallback((params) => {
        callOrder.push('pre');
        return params;
      });
      node.addOnSetParametersCallback(() => {
        callOrder.push('on');
        return { successful: false, reason: 'nope' };
      });
      node.addPostSetParametersCallback(() => {
        callOrder.push('post');
      });

      node.setParameter(
        new rclnodejs.Parameter(
          'test_param',
          rclnodejs.ParameterType.PARAMETER_INTEGER,
          42
        )
      );

      assert.deepStrictEqual(callOrder, ['pre', 'on']);
    });
  });
});
