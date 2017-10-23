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
const packages = require('./rosidl_gen/packages.js');
const loader = require('./lib/interface_loader.js');
const QoS = require('./lib/qos.js');
const validator = require('./lib/validator.js');

function inherits(target, source) {
  let properties = Object.getOwnPropertyNames(source.prototype);
  properties.forEach((property) => {
    target.prototype[property] = source.prototype[property];
  });
}

inherits(rclnodejs.ShadowNode, Node);

/**
 * A module that exposes the rclnodejs interfaces.
 * @exports rclnodejs
 */
let rcl = {
  _initialized: false,
  _nodes: [],

  /** {@link QoS} class */
  QoS: QoS,

  /** {@link module:validator|validator} object */
  validator: validator,
  /**
   * Create a node.
   * @param {string} nodeName - The name used to register in ROS.
   * @param {string} namespace - The namespace used in ROS, default is an empty string.
   * @return {Node} The instance of Node.
   */
  createNode(nodeName, namespace = '') {
    if (typeof (nodeName) !== 'string' || typeof (namespace) !== 'string') {
      throw new TypeError('Invalid argument.');
    }

    let handle = rclnodejs.createNode(nodeName, namespace);
    let node =  new rclnodejs.ShadowNode();

    node.init(nodeName, namespace);
    node.handle = handle;
    this._nodes.push(node);
    return node;
  },

  /**
  * Init the module.
  * @param {array} args - The command line arguments to pass to rcl.
  * @return {Promise<undefined>} A Promise.
  */
  init(...args) {
    return new Promise((resolve, reject) => {
      if (!this._initialized) {
        // TODO(Kenny): introduce other policy to save the amout of time of doing message generation
        generator.generateAll(false).then(() => {
          rclnodejs.init(args);
          this._initialized = true;
          resolve();
        }).catch((e) => {
          reject(e);
        });
      } else {
        throw new Error('The module rclnodejs has been initialized.');
      }
    });
  },

  /**
   * Start to spin the node, which triggers the event loop to start to check the incoming events.
   * @param {Node} node - The node to be spun.
   * @return {undefined}
   */
  spin(node) {
    if (!(node instanceof rclnodejs.ShadowNode)) {
      throw new TypeError('Invalid argument.');
    }
    if (node.spinning) {
      throw new Error('The node is already spinning.');
    }
    node.startSpinning();
  },

  /**
   * Terminate the node, this will destory all the allocated resources and quit.
   * @return {undefined}
   */
  shutdown() {
    if (!this._initialized) {
      throw new Error('The module rclnodejs has been shut.');
    }

    this._nodes.forEach((node) => {
      node.stopSpinning();
      node.destroy();
    });

    rclnodejs.shutdown();
    this._nodes = [];
    this._initialized = false;
  },

  /**
   * Get the interface package, which is used by publisher/subscription or client/service.
   * @param {string} packageName - The package wanted to get.
   * @param {string} interfaceName - The interface in the package, if it's not assigned, then the whole package will be got.
   * @return {object} - the object of the designated package/interface.
   */
  require(packageName, interfaceName) {
    // TODO(minggang): Can require by a single interface name instead of the
    // whole package.
    let interfaceInfos = loader.loadInterfaceInfos(packageName);
    let pkg = {srv: {}, msg: {}};

    interfaceInfos.forEach((info) => {
      Object.defineProperty(pkg[info.type], info.name, {value: require(info.filePath)});
    });
    return pkg;
  },

  /**
   * Search packgaes which locate under path $AMENT_PREFIX_PATH, regenerate all JavaScript structs files from the IDL of
   * messages(.msg) and services(.srv) and put these files under folder 'generated'. Any existing files under
   * this folder will be overwritten after the execution.
   * @return {Promise<undefined>} A Promise.
   */
  regenerateAll() {
    // This will trigger to regererate all the JS structs used for messages and services,
    // to overwrite the existing ones although they have been created.
    return new Promise((resolve, reject) => {
      generator.generateAll(true).then(() => {
        resolve();
      }).catch((e) => {
        reject(e);
      });
    });
  },

  /**
   * Judge if the topic/service is hidden, see http://design.ros2.org/articles/topic_and_service_names.html#hidden-topic-or-service-names
   * @param {string} name - Name of topic/service.
   * @return {boolean} - True if a given topic or service name is hidden, otherwise False.
   */
  isTopicOrServiceHidden(name) {
    if (typeof (name) !== 'string') {
      throw new TypeError('Invalid argument');
    }

    let arr = name.split('/');
    for (let i= 0; i < arr.length; i++) {
      if (arr[i].startsWith('_')) return true;
    }
    return false;
  },

  /**
   * Expand a given topic name using given node name and namespace as well.
   * @param {string} topicName - Topic name to be expanded.
   * @param {string} nodeName - Name of the node that this topic is associated with.
   * @param {string} nodeNamespace - Namespace that the topic is within.
   * @return {string} Expanded topic name which is fully qualified.
   */
  expandTopicName(topicName, nodeName, nodeNamespace) {
    return rclnodejs.expandTopicName(topicName, nodeName, nodeNamespace);
  }
};

module.exports = rcl;
