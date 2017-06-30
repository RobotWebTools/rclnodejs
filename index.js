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

const rclnodejs = require('bindings')('rclnodejs');
const Node = require('./lib/node.js');
const generator = require('./rosidl_gen/generator.js');

function inherits(target, source) {
  let properties = Object.getOwnPropertyNames(source.prototype);
  properties.forEach((property) => {
    target.prototype[property] = source.prototype[property];
  });
}

inherits(rclnodejs.ShadowNode, Node);

let rcl = {
  _initialized: false,
  _messageGenerated: false,
  _nodes: [],
  _allMessageType: [],

  createNode(nodeName, namespace = '') {
    if (typeof (nodeName) !== 'string' || typeof (namespace) !== 'string') {
      throw new TypeError('Invalid argument.');
    }

    let handle = rclnodejs.createNode(nodeName, namespace);
    let node =  new rclnodejs.ShadowNode();

    node.init();
    node._handle = handle;
    this._nodes.push(node);
    return node;
  },

  getAllMessageTypes: function() {
    return this._allMessageType;
  },

  init(...args) {
    if (this._initialized) {
      console.log('Warning - rclnodejs is already initialized. These arguments will be ignored:', args);
    } else {
      rclnodejs.init(args);
      this._initialized = true;
    }

    // TODO(Kenny): introduce other policy to save the amout of time of doing message generation
    return new Promise(function(resolve, reject) {
      if (this._messageGenerated) {
        resolve();
        return;
      }

      this.message.generateAll().then((all) => {
        this._allMessageType = all;
        this._messageGenerated = true;
        resolve();
      }).catch((e) => {
        reject(e);
      });
    }.bind(this));
  },

  spin(node) {
    if (!(node instanceof rclnodejs.ShadowNode)) {
      throw new TypeError('Invalid argument.');
    }
    if (node.spinning) {
      throw new Error('The node is already spinning.');
    }

    rclnodejs.spin(node);
    node.spinning = true;
  },

  shutdown() {
    this._nodes.forEach((node) => {
      node.destory();
    });

    rclnodejs.shutdown();
    this._nodes = [];
    this._initialized = false;
  },

  message: generator,
};

module.exports = rcl;
