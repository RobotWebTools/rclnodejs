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

const { Clock, ROSClock } = require('./lib/clock.js');
const ClockType = require('./lib/clock_type.js');
const compareVersions = require('compare-versions');
const Context = require('./lib/context.js');
const debug = require('debug')('rclnodejs');
const Duration = require('./lib/duration.js');
const fs = require('fs');
const generator = require('./rosidl_gen/index.js');
const loader = require('./lib/interface_loader.js');
const logging = require('./lib/logging.js');
const NodeOptions = require('./lib/node_options.js');
const {
  FloatingPointRange,
  IntegerRange,
  Parameter,
  ParameterDescriptor,
  ParameterType,
  DEFAULT_NUMERIC_RANGE_TOLERANCE,
} = require('./lib/parameter.js');
const path = require('path');
const QoS = require('./lib/qos.js');
const rclnodejs = require('bindings')('rclnodejs');
const tsdGenerator = require('./rostsd_gen/index.js');
const validator = require('./lib/validator.js');
const Time = require('./lib/time.js');
const ActionClient = require('./lib/action/client.js');
const ActionServer = require('./lib/action/server.js');
const ClientGoalHandle = require('./lib/action/client_goal_handle.js');
const { CancelResponse, GoalResponse } = require('./lib/action/response.js');
const ServerGoalHandle = require('./lib/action/server_goal_handle.js');
const {
  getActionClientNamesAndTypesByNode,
  getActionServerNamesAndTypesByNode,
  getActionNamesAndTypes,
} = require('./lib/action/graph.js');

/**
 * Get the version of the generator that was used for the currently present interfaces.
 * @return {Promise<string | null>} The current version or null if the *generator.json* file was not found
 * @throws {Error} if there was an error reading the *generator.json* file (except for it being absent)
 */
async function getCurrentGeneratorVersion() {
  const jsonFilePath = path.join(generator.generatedRoot, 'generator.json');

  return new Promise((resolve, reject) => {
    fs.open(jsonFilePath, 'r', (err) => {
      if (err) {
        if (err.code === 'ENOENT') {
          resolve(null);
        } else {
          reject(err);
        }
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
  _rosVersionChecked: false,

  /** {@link Clock} class */
  Clock: Clock,

  /** {@link ClockType} enum */
  ClockType: ClockType,

  /** {@link Context} class */
  Context: Context,

  /**
   * @constant {number}
   * The plus/minus tolerance for determining number equivalence.
   *
   *  @see [FloatingPointRange]{@link FloatingPointRange}
   *  @see [IntegerRange]{@link IntegerRange}
   */
  DEFAULT_NUMERIC_RANGE_TOLERANCE: DEFAULT_NUMERIC_RANGE_TOLERANCE,

  /** {@link Duration} class */
  Duration: Duration,

  /** {@link FloatingPointRange} class */
  FloatingPointRange: FloatingPointRange,

  /** {@link IntegerRange} class */
  IntegerRange: IntegerRange,

  /** {@link Logging} class */
  logging: logging,

  /** {@link NodeOptions} class */
  NodeOptions: NodeOptions,

  /** {@link Parameter} */
  Parameter: Parameter,

  /** {@link ParameterDescriptor} */
  ParameterDescriptor: ParameterDescriptor,

  /** {@link ParameterType} */
  ParameterType: ParameterType,

  /** {@link QoS} class */
  QoS: QoS,

  /** {@link ROSClock} class */
  ROSClock: ROSClock,

  /** {@link Time} class */
  Time: Time,

  /** {@link module:validator|validator} object */
  validator: validator,

  /** {@link ActionClient} class */
  ActionClient: ActionClient,

  /** {@link ActionServer} class */
  ActionServer: ActionServer,

  /** {@link ClientGoalHandle} class */
  ClientGoalHandle: ClientGoalHandle,

  /** {@link ServerGoalHandle} class */
  ServerGoalHandle: ServerGoalHandle,

  /** {@link ServerGoalHandle} enum */
  CancelResponse: CancelResponse,

  /** {@link GoalResponse} enum */
  GoalResponse: GoalResponse,

  /** {@link getActionClientNamesAndTypesByNode} function */
  getActionClientNamesAndTypesByNode: getActionClientNamesAndTypesByNode,

  /** {@link getActionServerNamesAndTypesByNode} function */
  getActionServerNamesAndTypesByNode: getActionServerNamesAndTypesByNode,

  /** {@link getActionNamesAndTypes} function */
  getActionNamesAndTypes: getActionNamesAndTypes,

  /**
   * Create and initialize a node.
   * @param {string} nodeName - The name used to register in ROS.
   * @param {string} [namespace=''] - The namespace used in ROS.
   * @param {Context} [context=Context.defaultContext()] - The context to create the node in.
   * @param {NodeOptions} [options=NodeOptions.defaultOptions] - The options to configure the new node behavior.
   * @return {Node} A new instance of the specified node.
   * @throws {Error} If the given context is not registered.
   * @deprecated since 0.18.0, Use new Node constructor.
   */
  createNode(
    nodeName,
    namespace = '',
    context = Context.defaultContext(),
    options = NodeOptions.defaultOptions
  ) {
    return new this.Node(nodeName, namespace, context, options);
  },

  /**
   * Create a managed Node that implements a well-defined life-cycle state
   * model using the {@link https://github.com/ros2/rcl/tree/master/rcl_lifecycle|ros2 client library (rcl) lifecyle api}.
   * @param {string} nodeName - The name used to register in ROS.
   * @param {string} [namespace=''] - The namespace used in ROS.
   * @param {Context} [context=Context.defaultContext()] - The context to create the node in.
   * @param {NodeOptions} [options=NodeOptions.defaultOptions] - The options to configure the new node behavior.
   * @return {LifecycleNode} A new instance of the specified node.
   * @throws {Error} If the given context is not registered.
   * @deprecated since 0.18.0, Use new LifecycleNode constructor.
   */
  createLifecycleNode(
    nodeName,
    namespace = '',
    context = Context.defaultContext(),
    options = NodeOptions.defaultOptions
  ) {
    return new this.lifecycle.LifecycleNode(
      nodeName,
      namespace,
      context,
      options
    );
  },

  /**
   * Initialize an RCL environment, i.e., a Context, and regenerate the javascript
   * message files if the output format of the message-generator tool has changed.
   * The context serves as a container for nodes, publishers, subscribers, etc. and
   * must be initialized before use.
   * @param {Context} [context=Context.defaultContext()] - The context to initialize.
   * @param {string[]} argv - Process command line arguments.
   * @return {Promise<undefined>} A Promise.
   * @throws {Error} If the given context has already been initialized or the command
   *                 line arguments argv could not be parsed.
   */
  async init(context = Context.defaultContext(), argv = process.argv) {
    // check if context has already been initialized
    if (!context.isUninitialized()) {
      throw new Error('The context has already been initialized.');
    }

    // check argv for correct value and state
    if (!Array.isArray(argv)) {
      throw new TypeError('argv must be an array.');
    }
    if (!argv.every((argument) => typeof argument === 'string')) {
      throw new TypeError('argv elements must be strings (and not null).');
    }

    rclnodejs.init(context.handle, argv);

    if (this._rosVersionChecked) {
      // no further processing required
      return;
    }

    const version = await getCurrentGeneratorVersion();
    const forced =
      version === null ||
      compareVersions.compare(version, generator.version(), '<');
    if (forced) {
      debug(
        'The generator will begin to create JavaScript code from ROS IDL files...'
      );
    }

    await generator.generateAll(forced);
    // TODO determine if tsd generateAll() should be here
    this._rosVersionChecked = true;
  },

  /**
   * Start detection and processing of units of work.
   * @param {Node} node - The node to be spun up.
   * @param {number} [timeout=10] - Timeout to wait in milliseconds. Block forever if negative. Don't wait if 0.
   * @throws {Error} If the node is already spinning.
   * @return {undefined}
   * @deprecated since 0.18.0, Use Node.spin(timeout)
   */
  spin(node, timeout = 10) {
    node.spin(timeout);
  },

  /**
   * Spin the node and trigger the event loop to check for one incoming event. Thereafter the node
   * will not received additional events until running additional calls to spin() or spinOnce().
   * @param {Node} node - The node to be spun.
   * @param {number} [timeout=10] - Timeout to wait in milliseconds. Block forever if negative. Don't wait if 0.
   * @throws {Error} If the node is already spinning.
   * @return {undefined}
   * @deprecated since 0.18.0, Use Node.spinOnce(timeout)
   */
  spinOnce(node, timeout = 10) {
    node.spinOnce(timeout);
  },

  /**
   * Shutdown an RCL environment identified by a context. The shutdown process will
   * destroy all nodes and related resources in the context. If no context is
   * explicitly given, the default context will be shut down.
   * This follows the semantics of
   * [rclpy.shutdown()]{@link http://docs.ros2.org/latest/api/rclpy/api/init_shutdown.html#rclpy.shutdown}.
   *
   * @param { Context } [context = Context.defaultContext()] - The context to be shutdown.
   * @return { undefined }
   * @throws { Error } If there is a problem shutting down the context or while destroying or shutting down a node within it.
   */
  shutdown(context = Context.defaultContext()) {
    context.shutdown();
  },

  /**
   * Shutdown all RCL environments via their contexts.
   * @return { undefined }
   * @throws { Error } If there is a problem shutting down the context or while destroying or shutting down a node within it.
   */
  shutdownAll() {
    for (const context of Context.instances) {
      this.shutdown(context);
    }
  },

  /**
   * Determine if an RCL environment identified by a context argument
   * has been shutdown.
   * @param {Context} [context=Context.defaultContext()] - The context to inspect.
   * @return {boolean} Return true if the module is shut down, otherwise return false.
   */
  isShutdown(context = Context.defaultContext()) {
    return !context.isInitialized();
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
   * Search packages which locate under path $AMENT_PREFIX_PATH, regenerate all JavaScript structs files from the IDL of
   * messages(.msg) and services(.srv) and put these files under folder 'generated'. Any existing files under
   * this folder will be overwritten after the execution.
   * @return {Promise<undefined>} A Promise.
   */
  async regenerateAll() {
    // This will trigger to regenerate all the JS structs used for messages and services,
    // to overwrite the existing ones although they have been created.
    debug('Begin regeneration of JavaScript code from ROS IDL files...');

    // generate the messages and type declarations, which must be done in sequence
    await generator.generateAll(true);
    await tsdGenerator.generateAll(); // create interfaces.d.ts

    debug('Finished regeneration.');
  },

  /**
   * Judge if the topic/service is hidden (see [the ROS2 design documentation]{@link http://design.ros2.org/articles/topic_and_service_names.html#hidden-topic-or-service-names}).
   * @param {string} name - Name of topic/service.
   * @return {boolean} - True if a given topic or service name is hidden, otherwise False.
   */
  isTopicOrServiceHidden(name) {
    if (typeof name !== 'string') {
      throw new TypeError('Invalid argument');
    }

    return name.split('/').some((slice) => slice.startsWith('_'));
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
    const typeClass = loader.loadInterface(type);

    if (typeClass) {
      return new typeClass();
    }

    return undefined;
  },

  /**
   * Create a plain JavaScript from the specified type identifier.
   * @param {string|Object} type -- the type identifier, acceptable formats could be 'std_msgs/std/String'
   *                                or {package: 'std_msgs', type: 'msg', name: 'String'}
   * @return {Object|undefined} A plain JavaScript of that type, or undefined if the object could not be created
   */
  createMessageObject(type) {
    return this.createMessage(type).toPlainObject();
  },

  /**
   * Removes the default signal handler installed by rclnodejs. After calling this, rclnodejs
   * will no longer clean itself up when a SIGINT is received, it is the application's
   * responsibility to properly shut down all nodes and contexts.
   *
   * Application which wishes to implement its own signal handler logic should call this.
   * @returns {undefined}
   */
  removeSignalHandlers() {
    // this will not throw even if the handler is already removed
    process.removeListener('SIGINT', _sigHandler);
  },
};

const _sigHandler = () => {
  // shuts down all live contexts. Applications that wishes to use their own signal handlers
  // should call `rclnodejs.removeSignalHandlers`.
  debug('Catch ctrl+c event and will cleanup and terminate.');
  rcl.shutdownAll();
};
process.on('SIGINT', _sigHandler);

module.exports = rcl;

// The following statements are located here to work around a
// circular dependency issue occurring in rate.js.
// Do not change the order of the following imports.
const Node = require('./lib/node.js');

/** {@link Node} class */
rcl.Node = Node;

const TimeSource = require('./lib/time_source.js');

/** {@link TimeSource} class */
rcl.TimeSource = TimeSource;

const Lifecycle = require('./lib/lifecycle.js');

/** Lifecycle namespace */
rcl.lifecycle = Lifecycle;
