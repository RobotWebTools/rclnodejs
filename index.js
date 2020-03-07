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
const fs = require('fs');
const generator = require('./rosidl_gen/index.js');
const loader = require('./lib/interface_loader.js');
const logging = require('./lib/logging.js');
const Node = require('./lib/node.js');
const NodeOptions = require('./lib/node_options.js');
const Parameters = require('./lib/parameter.js');
const path = require('path');
const QoS = require('./lib/qos.js');
const rclnodejs = require('bindings')('rclnodejs');
const tsdGenerator = require('./rostsd_gen/index.js');
const validator = require('./lib/validator.js');
const Time = require('./lib/time.js');
const TimeSource = require('./lib/time_source.js');
const { Clock, ROSClock } = require('./lib/clock.js');
const ClockType = require('./lib/clock_type.js');
const Duration = require('./lib/duration.js');
const Context = require('./lib/context.js');

function inherits(target, source) {
  let properties = Object.getOwnPropertyNames(source.prototype);
  properties.forEach(property => {
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

  /** {@link Context} class */
  Context: Context,

  /** {@link QoS} class */
  QoS: QoS,

  /** {@link Logging} class */
  logging: logging,

  /** {@link module:validator|validator} object */
  validator: validator,

  /** {@link Time} class */
  Time: Time,

  /** {@link TimeSource} class */
  TimeSource: TimeSource,

  /** {@link Clock} class */
  Clock: Clock,

  /** {@link ROSClock} class */
  ROSClock: ROSClock,

  /** {@link ClockType} enum */
  ClockType: ClockType,

  /** {@link Duration} class */
  Duration: Duration,

  /** {@link NodeOptions} class */
  NodeOptions: NodeOptions,

  /** {@link Parameters} */
  Parameters: Parameters,

  /**
   * Create a node.
   * @param {string} nodeName - The name used to register in ROS.
   * @param {string} namespace - The namespace used in ROS, default is an empty string.
   * @param {Context} context - The context, default is Context.defaultContext().
   * @param {NodeOptions} options - The options to configure the new node behavior.
   * @return {Node} The instance of Node.
   */
  createNode(
    nodeName,
    namespace = '',
    context = Context.defaultContext(),
    options = NodeOptions.defaultOptions
  ) {
    if (typeof nodeName !== 'string' || typeof namespace !== 'string') {
      throw new TypeError('Invalid argument.');
    }

    let handle = rclnodejs.createNode(nodeName, namespace, context.handle());
    let node = new rclnodejs.ShadowNode();
    node.handle = handle;

    node.init(nodeName, namespace, context, options);
    debug(
      'Finish initializing node, name = %s and namespace = %s.',
      nodeName,
      namespace
    );

    node.context = context;
    this._nodes.push(node);
    return node;
  },

  /**
   * Init the module.
   * @param {Context} context - The context, default is Context.defaultContext().
   * @param {string[]} argv - Process commandline arguments.
   * @return {Promise<undefined>} A Promise.
   */
  init(context = Context.defaultContext(), argv = process.argv) {
    return new Promise((resolve, reject) => {
      // check argv for correct value and state
      if (!Array.isArray(argv)) {
        throw new TypeError('argv must be an array.');
      }
      if (argv.reduce((hasNull, arg) => typeof arg !== 'string', false)) {
        throw new TypeError('argv elements must not be null');
      }

      let that = this;
      if (!this._initialized) {
        getCurrentGeneratorVersion()
          .then(version => {
            let forced =
              version === null ||
              compareVersions(version, generator.version()) === -1
                ? true
                : false;
            if (forced) {
              debug(
                'The generator will begin to create JavaScript code from ROS IDL files...'
              );
            }

            generator
              .generateAll(forced)
              .then(() => {
                this._context = context;
                rclnodejs.init(context.handle(), argv);
                this._initialized = true;
                resolve();
              })
              .catch(e => {
                reject(e);
              });
          })
          .catch(e => {
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
   * @param {number} [timeout=10] - ms to wait, block forever if negative, don't wait if 0, default is 10.
   * @return {undefined}
   */
  spin(node, timeout = 10) {
    if (!(node instanceof rclnodejs.ShadowNode)) {
      throw new TypeError('Invalid argument.');
    }
    if (node.spinning) {
      throw new Error('The node is already spinning.');
    }
    node.startSpinning(this._context.handle(), timeout);
  },

  /**
   * Execute one item of work or wait until a timeout expires.
   * @param {Node} node - The node to be spun.
   * @param {number} [timeout=10] - ms to wait, block forever if negative, don't wait if 0, default is 10.
   * @return {undefined}
   */
  spinOnce(node, timeout = 10) {
    if (!(node instanceof rclnodejs.ShadowNode)) {
      throw new TypeError('Invalid argument.');
    }
    if (node.spinning) {
      throw new Error('The node is already spinning.');
    }
    node.spinOnce(this._context.handle(), timeout);
  },

  /**
   * @param {Context} context - The context to be shutdown.
   * @return {undefined}
   */
  shutdown(context) {
    if (!this._initialized) {
      throw new Error('The module rclnodejs has been shut.');
    }

    this._nodes.forEach(node => {
      node.stopSpinning();
      node.destroy();
    });
    if (!context) {
      Context.shutdownDefaultContext();
    } else {
      context.shutdown();
    }
    this._nodes = [];
    this._initialized = false;
    this._context = undefined;
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
    debug('Begin regeneration of JavaScript code from ROS IDL files.');
    return new Promise((resolve, reject) => {
      generator
        .generateAll(true)
        .then(() => {
          tsdGenerator.generateAll(); // create interfaces.d.ts
          debug('Finish regeneration.');
          resolve();
        })
        .catch(e => {
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
    if (typeof name !== 'string') {
      throw new TypeError('Invalid argument');
    }

    let arr = name.split('/');
    for (let i = 0; i < arr.length; i++) {
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
