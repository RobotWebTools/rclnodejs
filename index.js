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

const compareVersions = require('compare-versions');
const debug = require('debug')('rclnodejs');
const fs = require('mz/fs');
const generator = require('./rosidl_gen/generator.js');
const loader = require('./lib/interface_loader.js');
const logging = require('./lib/logging.js');
const Node = require('./lib/node.js');
const packages = require('./rosidl_gen/packages.js');
const path = require('path');
const QoS = require('./lib/qos.js');
const rclnodejs = require('bindings')('rclnodejs');
const validator = require('./lib/validator.js');

function inherits(target, source) {
  let properties = Object.getOwnPropertyNames(source.prototype);
  properties.forEach((property) => {
    target.prototype[property] = source.prototype[property];
  });
}

inherits(rclnodejs.ShadowNode, Node);

function getCurrentGeneratorVersion() {
  let jsonFilePath = path.join(generator.generatedRoot, 'generator.json');

  return new Promise((resolve, reject) => {
    fs.open(jsonFilePath, 'r', (err, fd) => {
      if (err) {
        if (err.code === 'ENOENT') {
          resolve(null);
          return;
        }
        reject(err);
      } else {
        fs.readFile(jsonFilePath, 'utf8', (err, data) => {
          if (err) {
            reject(err);
          } else {
            resolve(JSON.parse(data).version);
          }
        });
      }
    });
  });
}

/**
 * A module that exposes the rclnodejs interfaces.
 * @exports rclnodejs
 */
let rcl = {
  _initialized: false,
  _nodes: [],

  /** {@link QoS} class */
  QoS: QoS,

  /** {@link Logging} class */
  logging: logging,

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
    debug('Finish initializing node, name = %s and namespace = %s.', nodeName, namespace);
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
        getCurrentGeneratorVersion().then(version => {
          let forced = version === null || compareVersions(version, generator.version()) === -1
            ? true
            : false;
          if (forced) {
            debug('The generator will begin to create JavaScript code from ROS IDL files...');
          }

          generator.generateAll(forced).then(() => {
            rclnodejs.init(args);
            debug('Finish initializing rcl with args = %o.', args);
            this._initialized = true;
            resolve();
          }).catch(e => {
            reject(e);
          });
        }).catch(e => {
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
   * Return status that whether the module is shut down.
   * @return {boolean} Return true if the module is shut down, otherwise return false.
   */
  isShutdown() {
    return !this._initialized;
  },

  /**
   * Get the interface package, which is used by publisher/subscription or client/service.
   * @param {string} name - The name of interface to be required.
   * @return {object} - the object of the required package/interface.
   */
  require(name) {
    return loader.loadInterface(name);
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
    debug('Begin to regenerate JavaScript code from ROS IDL files.');
    return new Promise((resolve, reject) => {
      generator.generateAll(true).then(() => {
        debug('Finish regenerating.');
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
  },

  createMessage(type) {
    let typeClass = loader.loadInterface(type);

    if (typeClass) {
      return new typeClass();
    }

    return undefined;
  },

  /**
   * Create a plain JavaScript by specified type identifier
   * @param {string|Object} type -- the type identifier, acceptable formats could be 'std_msgs/std/String'
   *                                or {package: 'std_msgs', type: 'msg', name: 'String'}
   * @return {Object|undefined} A plain JavaScript of that type
   */
  createMessageObject(type) {
    return this.createMessage(type).toPlainObject();
  },
};

process.on('SIGINT', () => {
  debug('Catch ctrl+c event and will cleanup and terminate.');
  rcl.shutdown();
  process.exit(0);
});

module.exports = rcl;
