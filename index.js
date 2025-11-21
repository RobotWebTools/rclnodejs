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

const DistroUtils = require('./lib/distro.js');
const RMWUtils = require('./lib/rmw.js');
const { Clock, ROSClock } = require('./lib/clock.js');
const ClockType = require('./lib/clock_type.js');
const { compareVersions } = require('./lib/utils.js');
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
const rclnodejs = require('./lib/native_loader.js');
const tsdGenerator = require('./rostsd_gen/index.js');
const validator = require('./lib/validator.js');
const Time = require('./lib/time.js');
const ActionClient = require('./lib/action/client.js');
const ActionServer = require('./lib/action/server.js');
const ActionUuid = require('./lib/action/uuid.js');
const ClientGoalHandle = require('./lib/action/client_goal_handle.js');
const { CancelResponse, GoalResponse } = require('./lib/action/response.js');
const ServerGoalHandle = require('./lib/action/server_goal_handle.js');
const { toJSONSafe, toJSONString } = require('./lib/message_serialization.js');
const {
  getActionClientNamesAndTypesByNode,
  getActionServerNamesAndTypesByNode,
  getActionNamesAndTypes,
} = require('./lib/action/graph.js');
const ServiceIntrospectionStates = require('./lib/service_introspection.js');
const {
  serializeMessage,
  deserializeMessage,
} = require('./lib/serialization.js');
const ParameterClient = require('./lib/parameter_client.js');
const errors = require('./lib/errors.js');
const ParameterWatcher = require('./lib/parameter_watcher.js');
const { spawn } = require('child_process');

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

let _rosVersionChecked = false;

/**
 * Run a ROS2 package executable using 'ros2 run' command.
 * @param {string} packageName - The name of the ROS2 package.
 * @param {string} executableName - The name of the executable to run.
 * @param {string[]} [args=[]] - Additional arguments to pass to the executable.
 * @return {Promise<{process: ChildProcess}>} A Promise that resolves with the process.
 */
function ros2Run(packageName, executableName, args = []) {
  return new Promise((resolve, reject) => {
    if (typeof packageName !== 'string' || !packageName.trim()) {
      reject(new Error('Package name must be a non-empty string'));
      return;
    }

    if (typeof executableName !== 'string' || !executableName.trim()) {
      reject(new Error('Executable name must be a non-empty string'));
      return;
    }

    if (!Array.isArray(args)) {
      reject(new Error('Arguments must be an array'));
      return;
    }

    const command = 'ros2';
    const cmdArgs = ['run', packageName, executableName, ...args];
    const childProcess = spawn(command, cmdArgs);

    childProcess.on('error', (error) => {
      reject(new Error(`Failed to start ros2 run: ${error.message}`));
    });
    childProcess.on('spawn', () => {
      resolve({
        process: childProcess,
      });
    });
  });
}

/**
 * Run a ROS2 launch file using 'ros2 launch' command.
 * @param {string} packageName - The name of the ROS2 package.
 * @param {string} launchFile - The name of the launch file to run.
 * @param {string[]} [args=[]] - Additional arguments to pass to the launch file.
 * @return {Promise<{process: ChildProcess}>} A Promise that resolves with the process.
 */
function ros2Launch(packageName, launchFile, args = []) {
  return new Promise((resolve, reject) => {
    if (typeof packageName !== 'string' || !packageName.trim()) {
      reject(new Error('Package name must be a non-empty string'));
      return;
    }
    if (typeof launchFile !== 'string' || !launchFile.trim()) {
      reject(new Error('Launch file name must be a non-empty string'));
      return;
    }
    if (!Array.isArray(args)) {
      reject(new Error('Arguments must be an array'));
      return;
    }
    const command = 'ros2';
    const cmdArgs = ['launch', packageName, launchFile, ...args];
    const childProcess = spawn(command, cmdArgs);

    childProcess.on('error', (error) => {
      reject(new Error(`Failed to start ros2 launch: ${error.message}`));
    });

    childProcess.on('spawn', () => {
      resolve({
        process: childProcess,
      });
    });
  });
}

/**
 * A module that exposes the rclnodejs interfaces.
 * @exports rclnodejs
 */
let rcl = {
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

  /** {@link DistroUtils} */
  DistroUtils: DistroUtils,

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

  /** {@link ParameterClient} class */
  ParameterClient: ParameterClient,

  /** {@link ParameterWatcher} class */
  ParameterWatcher: ParameterWatcher,

  /** {@link QoS} class */
  QoS: QoS,

  /** {@link RMWUtils} */
  RMWUtils: RMWUtils,

  /** {@link ROSClock} class */
  ROSClock: ROSClock,

  /** {@link ServiceIntrospectionStates} */
  ServiceIntrospectionStates: ServiceIntrospectionStates,

  /** {@link Time} class */
  Time: Time,

  /** {@link module:validator|validator} object */
  validator: validator,

  /** {@link ActionClient} class */
  ActionClient: ActionClient,

  /** {@link ActionServer} class */
  ActionServer: ActionServer,

  /** {@link ActionUuid} class */
  ActionUuid: ActionUuid,

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

  /** {@link serializeMessage} function */
  serializeMessage: serializeMessage,

  /** {@link deserializeMessage} function */
  deserializeMessage: deserializeMessage,

  /**
   * Create and initialize a node.
   * @param {string} nodeName - The name used to register in ROS.
   * @param {string} [namespace=''] - The namespace used in ROS.
   * @param {Context} [context=Context.defaultContext()] - The context to create the node in.
   * @param {NodeOptions} [options=NodeOptions.defaultOptions] - The options to configure the new node behavior.
   * @param {Array} [args=[]] - The arguments to pass to the node.
   * @param {boolean} [useGlobalArguments=true] - If true, the node will use the global arguments
   *                                             from the context, otherwise it will only use the arguments
   *                                             passed in the args parameter.
   * @return {Node} A new instance of the specified node.
   * @throws {Error} If the given context is not registered.
   * @deprecated since 0.18.0, Use new Node constructor.
   */
  createNode(
    nodeName,
    namespace = '',
    context = Context.defaultContext(),
    options = NodeOptions.defaultOptions,
    args = [],
    useGlobalArguments = true
  ) {
    return new this.Node(
      nodeName,
      namespace,
      context,
      options,
      args,
      useGlobalArguments
    );
  },

  /**
   * Create a LifecycleNode, a managed Node that implements a well-defined life-cycle state
   * model using the {@link https://github.com/ros2/rcl/tree/master/rcl_lifecycle|ros2 client library (rcl) lifecyle api}.
   * @param {string} nodeName - The name used to register in ROS.
   * @param {string} [namespace=''] - The namespace used in ROS.
   * @param {Context} [context=Context.defaultContext()] - The context to create the node in.
   * @param {NodeOptions} [options=NodeOptions.defaultOptions] - The options to configure the new node behavior.
   * @param {boolean} [enableCommunicationsInterface=true] - enable lifecycle service interfaces, e.g., GetState.
   * @return {LifecycleNode} A new instance of the specified node.
   * @throws {Error} If the given context is not registered.
   * @deprecated since 0.18.0, Use new LifecycleNode constructor.
   */
  createLifecycleNode(
    nodeName,
    namespace = '',
    context = Context.defaultContext(),
    options = NodeOptions.defaultOptions,
    enableCommunicationsInterface = true
  ) {
    return new this.lifecycle.LifecycleNode(
      nodeName,
      namespace,
      context,
      options,
      enableCommunicationsInterface
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

    rclnodejs.init(context.handle, argv, context._domainId);

    if (_rosVersionChecked) {
      // no further processing required
      return;
    }

    const version = await getCurrentGeneratorVersion();
    const forced =
      version === null || compareVersions(version, generator.version(), '<');
    if (forced) {
      debug(
        'The generator will begin to create JavaScript code from ROS IDL files...'
      );
    }

    await generator.generateAll(forced);
    // TODO determine if tsd generateAll() should be here
    _rosVersionChecked = true;
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

  /**
   * Run a ROS2 package executable using 'ros2 run' command.
   * @param {string} packageName - The name of the ROS2 package.
   * @param {string} executableName - The name of the executable to run.
   * @param {string[]} [args=[]] - Additional arguments to pass to the executable.
   * @return {Promise<{process: ChildProcess}>} A Promise that resolves with the process.
   */
  ros2Run: ros2Run,

  /**
   * Run a ROS2 launch file using 'ros2 launch' command.
   * @param {string} packageName - The name of the ROS2 package.
   * @param {string} launchFile - The name of the launch file to run.
   * @param {string[]} [args=[]] - Additional arguments to pass to the launch file.
   * @return {Promise<{process: ChildProcess}>} A Promise that resolves with the process.
   */
  ros2Launch: ros2Launch,

  /**
   * Convert a message object to be JSON-safe by converting TypedArrays to regular arrays
   * and handling BigInt, Infinity, NaN, etc. for JSON serialization.
   * @param {*} obj - The message object to convert
   * @returns {*} A JSON-safe version of the object
   */
  toJSONSafe: toJSONSafe,

  /**
   * Convert a message object to a JSON string with proper handling of TypedArrays,
   * BigInt, and other non-JSON-serializable values.
   * @param {*} obj - The message object to convert
   * @param {number} [space] - Space parameter for JSON.stringify formatting
   * @returns {string} The JSON string representation
   */
  toJSONString: toJSONString,

  // Error classes for structured error handling
  /** {@link RclNodeError} - Base error class for all rclnodejs errors */
  RclNodeError: errors.RclNodeError,

  /** {@link ValidationError} - Error thrown when validation fails */
  ValidationError: errors.ValidationError,
  /** {@link TypeValidationError} - Type validation error */
  TypeValidationError: errors.TypeValidationError,
  /** {@link RangeValidationError} - Range/value validation error */
  RangeValidationError: errors.RangeValidationError,
  /** {@link NameValidationError} - ROS name validation error */
  NameValidationError: errors.NameValidationError,

  /** {@link OperationError} - Base class for operation/runtime errors */
  OperationError: errors.OperationError,
  /** {@link TimeoutError} - Request timeout error */
  TimeoutError: errors.TimeoutError,
  /** {@link AbortError} - Request abortion error */
  AbortError: errors.AbortError,
  /** {@link ServiceNotFoundError} - Service not available error */
  ServiceNotFoundError: errors.ServiceNotFoundError,
  /** {@link NodeNotFoundError} - Remote node not found error */
  NodeNotFoundError: errors.NodeNotFoundError,

  /** {@link ParameterError} - Base error for parameter operations */
  ParameterError: errors.ParameterError,
  /** {@link ParameterNotFoundError} - Parameter not found error */
  ParameterNotFoundError: errors.ParameterNotFoundError,
  /** {@link ParameterTypeError} - Parameter type mismatch error */
  ParameterTypeError: errors.ParameterTypeError,
  /** {@link ReadOnlyParameterError} - Read-only parameter modification error */
  ReadOnlyParameterError: errors.ReadOnlyParameterError,

  /** {@link TopicError} - Base error for topic operations */
  TopicError: errors.TopicError,
  /** {@link PublisherError} - Publisher-specific error */
  PublisherError: errors.PublisherError,
  /** {@link SubscriptionError} - Subscription-specific error */
  SubscriptionError: errors.SubscriptionError,

  /** {@link ActionError} - Base error for action operations */
  ActionError: errors.ActionError,
  /** {@link GoalRejectedError} - Goal rejected by action server */
  GoalRejectedError: errors.GoalRejectedError,
  /** {@link ActionServerNotFoundError} - Action server not found */
  ActionServerNotFoundError: errors.ActionServerNotFoundError,

  /** {@link NativeError} - Wraps errors from native C++ layer */
  NativeError: errors.NativeError,
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
