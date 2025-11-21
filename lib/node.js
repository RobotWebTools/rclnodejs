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

const rclnodejs = require('./native_loader.js');

const ActionInterfaces = require('./action/interfaces.js');
const Client = require('./client.js');
const Clock = require('./clock.js');
const Context = require('./context.js');
const debug = require('debug')('rclnodejs:node');
const DistroUtils = require('./distro.js');
const GuardCondition = require('./guard_condition.js');
const loader = require('./interface_loader.js');
const Logging = require('./logging.js');
const NodeOptions = require('./node_options.js');
const {
  ParameterType,
  Parameter,
  ParameterDescriptor,
} = require('./parameter.js');
const { isValidSerializationMode } = require('./message_serialization.js');
const {
  TypeValidationError,
  RangeValidationError,
  ValidationError,
} = require('./errors.js');
const ParameterService = require('./parameter_service.js');
const ParameterClient = require('./parameter_client.js');
const ParameterWatcher = require('./parameter_watcher.js');
const Publisher = require('./publisher.js');
const QoS = require('./qos.js');
const Rates = require('./rate.js');
const Service = require('./service.js');
const Subscription = require('./subscription.js');
const TimeSource = require('./time_source.js');
const Timer = require('./timer.js');
const TypeDescriptionService = require('./type_description_service.js');
const Entity = require('./entity.js');
const { SubscriptionEventCallbacks } = require('../lib/event_handler.js');
const { PublisherEventCallbacks } = require('../lib/event_handler.js');
const { validateFullTopicName } = require('./validator.js');

// Parameter event publisher constants
const PARAMETER_EVENT_MSG_TYPE = 'rcl_interfaces/msg/ParameterEvent';
const PARAMETER_EVENT_TOPIC = 'parameter_events';

/**
 * @class - Class representing a Node in ROS
 */

class Node extends rclnodejs.ShadowNode {
  /**
   * Create a ROS2Node.
   * model using the {@link https://github.com/ros2/rcl/tree/master/rcl_lifecycle|ros2 client library (rcl) lifecyle api}.
   * @param {string} nodeName - The name used to register in ROS.
   * @param {string} [namespace=''] - The namespace used in ROS.
   * @param {Context} [context=Context.defaultContext()] - The context to create the node in.
   * @param {NodeOptions} [options=NodeOptions.defaultOptions] - The options to configure the new node behavior.
   * @throws {Error} If the given context is not registered.
   */
  constructor(
    nodeName,
    namespace = '',
    context = Context.defaultContext(),
    options = NodeOptions.defaultOptions,
    args = [],
    useGlobalArguments = true
  ) {
    super();

    if (typeof nodeName !== 'string') {
      throw new TypeValidationError('nodeName', nodeName, 'string');
    }
    if (typeof namespace !== 'string') {
      throw new TypeValidationError('namespace', namespace, 'string');
    }

    this._init(nodeName, namespace, options, context, args, useGlobalArguments);
    debug(
      'Finish initializing node, name = %s and namespace = %s.',
      nodeName,
      namespace
    );
  }

  _init(name, namespace, options, context, args, useGlobalArguments) {
    this.handle = rclnodejs.createNode(
      name,
      namespace,
      context.handle,
      args,
      useGlobalArguments
    );
    Object.defineProperty(this, 'handle', {
      configurable: false,
      writable: false,
    }); // make read-only

    this._context = context;
    this.context.onNodeCreated(this);

    this._publishers = [];
    this._subscriptions = [];
    this._clients = [];
    this._services = [];
    this._timers = [];
    this._guards = [];
    this._events = [];
    this._actionClients = [];
    this._actionServers = [];
    this._parameterClients = [];
    this._parameterWatchers = [];
    this._rateTimerServer = null;
    this._parameterDescriptors = new Map();
    this._parameters = new Map();
    this._parameterService = null;
    this._typeDescriptionService = null;
    this._parameterEventPublisher = null;
    this._setParametersCallbacks = [];
    this._logger = new Logging(rclnodejs.getNodeLoggerName(this.handle));
    this._spinning = false;

    this._parameterEventPublisher = this.createPublisher(
      PARAMETER_EVENT_MSG_TYPE,
      PARAMETER_EVENT_TOPIC
    );

    // initialize _parameterOverrides from parameters defined on the commandline
    this._parameterOverrides = this._getNativeParameterOverrides();

    // override cli parameterOverrides with those specified in options
    if (options.parameterOverrides.length > 0) {
      for (const parameter of options.parameterOverrides) {
        if ((!parameter) instanceof Parameter) {
          throw new TypeValidationError(
            'parameterOverride',
            parameter,
            'Parameter instance',
            {
              nodeName: name,
            }
          );
        }
        this._parameterOverrides.set(parameter.name, parameter);
      }
    }

    // initialize _parameters from parameterOverrides
    if (options.automaticallyDeclareParametersFromOverrides) {
      for (const parameter of this._parameterOverrides.values()) {
        parameter.validate();
        const descriptor = ParameterDescriptor.fromParameter(parameter);
        this._parameters.set(parameter.name, parameter);
        this._parameterDescriptors.set(parameter.name, descriptor);
      }
    }

    // Clock that has support for ROS time.
    // Note: parameter overrides and parameter event publisher need to be ready at this point
    // to be able to declare 'use_sim_time' if it was not declared yet.
    this._clock = new Clock.ROSClock();
    this._timeSource = new TimeSource(this);
    this._timeSource.attachClock(this._clock);

    if (options.startParameterServices) {
      this._parameterService = new ParameterService(this);
      this._parameterService.start();
    }

    if (
      DistroUtils.getDistroId() >= DistroUtils.getDistroId('jazzy') &&
      options.startTypeDescriptionService
    ) {
      this._typeDescriptionService = new TypeDescriptionService(this);
      this._typeDescriptionService.start();
    }
  }

  execute(handles) {
    let timersReady = this._timers.filter((timer) =>
      handles.includes(timer.handle)
    );
    let guardsReady = this._guards.filter((guard) =>
      handles.includes(guard.handle)
    );
    let subscriptionsReady = this._subscriptions.filter((subscription) =>
      handles.includes(subscription.handle)
    );
    let clientsReady = this._clients.filter((client) =>
      handles.includes(client.handle)
    );
    let servicesReady = this._services.filter((service) =>
      handles.includes(service.handle)
    );
    let actionClientsReady = this._actionClients.filter((actionClient) =>
      handles.includes(actionClient.handle)
    );
    let actionServersReady = this._actionServers.filter((actionServer) =>
      handles.includes(actionServer.handle)
    );
    let eventsReady = this._events.filter((event) =>
      handles.includes(event.handle)
    );

    timersReady.forEach((timer) => {
      if (timer.isReady()) {
        rclnodejs.callTimer(timer.handle);
        timer.callback();
      }
    });

    eventsReady.forEach((event) => {
      event.takeData();
    });

    for (const subscription of subscriptionsReady) {
      if (subscription.isDestroyed()) continue;
      if (subscription.isRaw) {
        let rawMessage = rclnodejs.rclTakeRaw(subscription.handle);
        if (rawMessage) {
          subscription.processResponse(rawMessage);
        }
        continue;
      }

      this._runWithMessageType(
        subscription.typeClass,
        (message, deserialize) => {
          let success = rclnodejs.rclTake(subscription.handle, message);
          if (success) {
            subscription.processResponse(deserialize());
          }
        }
      );
    }

    for (const guard of guardsReady) {
      if (guard.isDestroyed()) continue;

      guard.callback();
    }

    for (const client of clientsReady) {
      if (client.isDestroyed()) continue;
      this._runWithMessageType(
        client.typeClass.Response,
        (message, deserialize) => {
          let sequenceNumber = rclnodejs.rclTakeResponse(
            client.handle,
            message
          );
          if (sequenceNumber !== undefined) {
            client.processResponse(sequenceNumber, deserialize());
          }
        }
      );
    }

    for (const service of servicesReady) {
      if (service.isDestroyed()) continue;
      this._runWithMessageType(
        service.typeClass.Request,
        (message, deserialize) => {
          let header = rclnodejs.rclTakeRequest(
            service.handle,
            this.handle,
            message
          );
          if (header) {
            service.processRequest(header, deserialize());
          }
        }
      );
    }

    for (const actionClient of actionClientsReady) {
      if (actionClient.isDestroyed()) continue;

      const properties = actionClient.handle.properties;

      if (properties.isGoalResponseReady) {
        this._runWithMessageType(
          actionClient.typeClass.impl.SendGoalService.Response,
          (message, deserialize) => {
            let sequence = rclnodejs.actionTakeGoalResponse(
              actionClient.handle,
              message
            );
            if (sequence != undefined) {
              actionClient.processGoalResponse(sequence, deserialize());
            }
          }
        );
      }

      if (properties.isCancelResponseReady) {
        this._runWithMessageType(
          actionClient.typeClass.impl.CancelGoal.Response,
          (message, deserialize) => {
            let sequence = rclnodejs.actionTakeCancelResponse(
              actionClient.handle,
              message
            );
            if (sequence != undefined) {
              actionClient.processCancelResponse(sequence, deserialize());
            }
          }
        );
      }

      if (properties.isResultResponseReady) {
        this._runWithMessageType(
          actionClient.typeClass.impl.GetResultService.Response,
          (message, deserialize) => {
            let sequence = rclnodejs.actionTakeResultResponse(
              actionClient.handle,
              message
            );
            if (sequence != undefined) {
              actionClient.processResultResponse(sequence, deserialize());
            }
          }
        );
      }

      if (properties.isFeedbackReady) {
        this._runWithMessageType(
          actionClient.typeClass.impl.FeedbackMessage,
          (message, deserialize) => {
            let success = rclnodejs.actionTakeFeedback(
              actionClient.handle,
              message
            );
            if (success) {
              actionClient.processFeedbackMessage(deserialize());
            }
          }
        );
      }

      if (properties.isStatusReady) {
        this._runWithMessageType(
          actionClient.typeClass.impl.GoalStatusArray,
          (message, deserialize) => {
            let success = rclnodejs.actionTakeStatus(
              actionClient.handle,
              message
            );
            if (success) {
              actionClient.processStatusMessage(deserialize());
            }
          }
        );
      }
    }

    for (const actionServer of actionServersReady) {
      if (actionServer.isDestroyed()) continue;

      const properties = actionServer.handle.properties;

      if (properties.isGoalRequestReady) {
        this._runWithMessageType(
          actionServer.typeClass.impl.SendGoalService.Request,
          (message, deserialize) => {
            const result = rclnodejs.actionTakeGoalRequest(
              actionServer.handle,
              message
            );
            if (result) {
              actionServer.processGoalRequest(result, deserialize());
            }
          }
        );
      }

      if (properties.isCancelRequestReady) {
        this._runWithMessageType(
          actionServer.typeClass.impl.CancelGoal.Request,
          (message, deserialize) => {
            const result = rclnodejs.actionTakeCancelRequest(
              actionServer.handle,
              message
            );
            if (result) {
              actionServer.processCancelRequest(result, deserialize());
            }
          }
        );
      }

      if (properties.isResultRequestReady) {
        this._runWithMessageType(
          actionServer.typeClass.impl.GetResultService.Request,
          (message, deserialize) => {
            const result = rclnodejs.actionTakeResultRequest(
              actionServer.handle,
              message
            );
            if (result) {
              actionServer.processResultRequest(result, deserialize());
            }
          }
        );
      }

      if (properties.isGoalExpired) {
        let GoalInfoArray = ActionInterfaces.GoalInfo.ArrayType;
        let message = new GoalInfoArray(actionServer._goalHandles.size);
        let count = rclnodejs.actionExpireGoals(
          actionServer.handle,
          actionServer._goalHandles.size,
          message._refArray.buffer
        );
        if (count > 0) {
          actionServer.processGoalExpired(message, count);
        }
        GoalInfoArray.freeArray(message);
      }
    }

    // At this point it is safe to clear the cache of any
    // destroyed entity references
    Entity._gcHandles();
  }

  /**
   * Determine if this node is spinning.
   * @returns {boolean} - true when spinning; otherwise returns false.
   */
  get spinning() {
    return this._spinning;
  }

  /**
   * Trigger the event loop to continuously check for and route.
   * incoming events.
   * @param {Node} node - The node to be spun up.
   * @param {number} [timeout=10] - Timeout to wait in milliseconds. Block forever if negative. Don't wait if 0.
   * @throws {Error} If the node is already spinning.
   * @return {undefined}
   */
  spin(timeout = 10) {
    if (this.spinning) {
      throw new Error('The node is already spinning.');
    }
    this.start(this.context.handle, timeout);
    this._spinning = true;
  }

  /**
   * Use spin().
   * @deprecated, since 0.18.0
   */
  startSpinning(timeout) {
    this.spin(timeout);
  }

  /**
   * Terminate spinning - no further events will be received.
   * @returns {undefined}
   */
  stop() {
    super.stop();
    this._spinning = false;
  }

  /**
   * Terminate spinning - no further events will be received.
   * @returns {undefined}
   * @deprecated since 0.18.0, Use stop().
   */
  stopSpinning() {
    super.stop();
    this._spinning = false;
  }

  /**
   * Spin the node and trigger the event loop to check for one incoming event. Thereafter the node
   * will not received additional events until running additional calls to spin() or spinOnce().
   * @param {Node} node - The node to be spun.
   * @param {number} [timeout=10] - Timeout to wait in milliseconds. Block forever if negative. Don't wait if 0.
   * @throws {Error} If the node is already spinning.
   * @return {undefined}
   */
  spinOnce(timeout = 10) {
    if (this.spinning) {
      throw new Error('The node is already spinning.');
    }
    super.spinOnce(this.context.handle, timeout);
  }

  _removeEntityFromArray(entity, array) {
    let index = array.indexOf(entity);
    if (index > -1) {
      array.splice(index, 1);
    }
  }

  _destroyEntity(entity, array, syncHandles = true) {
    if (entity['isDestroyed'] && entity.isDestroyed()) return;

    this._removeEntityFromArray(entity, array);
    if (syncHandles) {
      this.syncHandles();
    }

    if (entity['_destroy']) {
      entity._destroy();
    } else {
      // guards and timers
      entity.handle.release();
    }
  }

  _validateOptions(options) {
    if (
      options !== undefined &&
      (options === null || typeof options !== 'object')
    ) {
      throw new TypeValidationError('options', options, 'object', {
        nodeName: this.name(),
      });
    }

    if (options === undefined) {
      return Node.getDefaultOptions();
    }

    if (options.enableTypedArray === undefined) {
      options = Object.assign(options, { enableTypedArray: true });
    }

    if (options.qos === undefined) {
      options = Object.assign(options, { qos: QoS.profileDefault });
    }

    if (options.isRaw === undefined) {
      options = Object.assign(options, { isRaw: false });
    }

    if (options.serializationMode === undefined) {
      options = Object.assign(options, { serializationMode: 'default' });
    } else if (!isValidSerializationMode(options.serializationMode)) {
      throw new ValidationError(
        `Invalid serializationMode: ${options.serializationMode}. Valid modes are: 'default', 'plain', 'json'`,
        {
          code: 'INVALID_SERIALIZATION_MODE',
          argumentName: 'serializationMode',
          providedValue: options.serializationMode,
          expectedType: "'default' | 'plain' | 'json'",
          nodeName: this.name(),
        }
      );
    }

    return options;
  }

  /**
   * Create a Timer.
   * @param {bigint} period - The number representing period in nanoseconds.
   * @param {function} callback - The callback to be called when timeout.
   * @param {Clock} [clock] - The clock which the timer gets time from.
   * @return {Timer} - An instance of Timer.
   */
  createTimer(period, callback, clock = null) {
    if (arguments.length === 3 && !(arguments[2] instanceof Clock)) {
      clock = null;
    } else if (arguments.length === 4) {
      clock = arguments[3];
    }

    if (typeof period !== 'bigint') {
      throw new TypeValidationError('period', period, 'bigint', {
        nodeName: this.name(),
      });
    }
    if (typeof callback !== 'function') {
      throw new TypeValidationError('callback', callback, 'function', {
        nodeName: this.name(),
      });
    }

    const timerClock = clock || this._clock;
    let timerHandle = rclnodejs.createTimer(
      timerClock.handle,
      this.context.handle,
      period
    );
    let timer = new Timer(timerHandle, period, callback);
    debug('Finish creating timer, period = %d.', period);
    this._timers.push(timer);
    this.syncHandles();

    return timer;
  }

  /**
   * Create a Rate.
   *
   * @param {number} hz - The frequency of the rate timer; default is 1 hz.
   * @returns {Promise<Rate>} - Promise resolving to new instance of Rate.
   */
  async createRate(hz = 1) {
    if (typeof hz !== 'number') {
      throw new TypeValidationError('hz', hz, 'number', {
        nodeName: this.name(),
      });
    }

    const MAX_RATE_HZ_IN_MILLISECOND = 1000.0;
    if (hz <= 0.0 || hz > MAX_RATE_HZ_IN_MILLISECOND) {
      throw new RangeValidationError(
        'hz',
        hz,
        `0.0 < hz <= ${MAX_RATE_HZ_IN_MILLISECOND}`,
        {
          nodeName: this.name(),
        }
      );
    }

    // lazy initialize rateTimerServer
    if (!this._rateTimerServer) {
      this._rateTimerServer = new Rates.RateTimerServer(this);
      await this._rateTimerServer.init();
    }

    const period = Math.round(1000 / hz);
    const timer = this._rateTimerServer.createTimer(BigInt(period) * 1000000n);
    const rate = new Rates.Rate(hz, timer);

    return rate;
  }

  /**
   * Create a Publisher.
   * @param {function|string|object} typeClass - The ROS message class,
        OR a string representing the message class, e.g. 'std_msgs/msg/String',
        OR an object representing the message class, e.g. {package: 'std_msgs', type: 'msg', name: 'String'}
   * @param {string} topic - The name of the topic.
   * @param {object} options - The options argument used to parameterize the publisher.
   * @param {boolean} options.enableTypedArray - The topic will use TypedArray if necessary, default: true.
   * @param {QoS} options.qos - ROS Middleware "quality of service" settings for the publisher, default: QoS.profileDefault.
   * @param {PublisherEventCallbacks} eventCallbacks - The event callbacks for the publisher.
   * @return {Publisher} - An instance of Publisher.
   */
  createPublisher(typeClass, topic, options, eventCallbacks) {
    return this._createPublisher(
      typeClass,
      topic,
      options,
      Publisher,
      eventCallbacks
    );
  }

  _createPublisher(typeClass, topic, options, publisherClass, eventCallbacks) {
    if (typeof typeClass === 'string' || typeof typeClass === 'object') {
      typeClass = loader.loadInterface(typeClass);
    }
    options = this._validateOptions(options);

    if (typeof typeClass !== 'function') {
      throw new TypeValidationError('typeClass', typeClass, 'function', {
        nodeName: this.name(),
        entityType: 'publisher',
      });
    }
    if (typeof topic !== 'string') {
      throw new TypeValidationError('topic', topic, 'string', {
        nodeName: this.name(),
        entityType: 'publisher',
      });
    }
    if (
      eventCallbacks &&
      !(eventCallbacks instanceof PublisherEventCallbacks)
    ) {
      throw new TypeValidationError(
        'eventCallbacks',
        eventCallbacks,
        'PublisherEventCallbacks',
        {
          nodeName: this.name(),
          entityType: 'publisher',
          entityName: topic,
        }
      );
    }

    let publisher = publisherClass.createPublisher(
      this,
      typeClass,
      topic,
      options,
      eventCallbacks
    );
    debug('Finish creating publisher, topic = %s.', topic);
    this._publishers.push(publisher);
    return publisher;
  }

  /**
   * This callback is called when a message is published
   * @callback SubscriptionCallback
   * @param {Object} message - The message published
   * @see [Node.createSubscription]{@link Node#createSubscription}
   * @see [Node.createPublisher]{@link Node#createPublisher}
   * @see {@link Publisher}
   * @see {@link Subscription}
   */

  /**
   * Create a Subscription with optional content-filtering.
   * @param {function|string|object} typeClass - The ROS message class,
        OR a string representing the message class, e.g. 'std_msgs/msg/String',
        OR an object representing the message class, e.g. {package: 'std_msgs', type: 'msg', name: 'String'}
   * @param {string} topic - The name of the topic.
   * @param {object} options - The options argument used to parameterize the subscription.
   * @param {boolean} options.enableTypedArray - The topic will use TypedArray if necessary, default: true.
   * @param {QoS} options.qos - ROS Middleware "quality of service" settings for the subscription, default: QoS.profileDefault.
   * @param {boolean} options.isRaw - The topic is serialized when true, default: false.
   * @param {string} [options.serializationMode='default'] - Controls message serialization format:
   *  'default': Use native rclnodejs behavior (respects enableTypedArray setting),
   *  'plain': Convert TypedArrays to regular arrays,
   *  'json': Fully JSON-safe (handles TypedArrays, BigInt, etc.).
   * @param {object} [options.contentFilter=undefined] - The content-filter, default: undefined.
   *  Confirm that your RMW supports content-filtered topics before use. 
   * @param {string} options.contentFilter.expression - Specifies the criteria to select the data samples of
   *  interest. It is similar to the WHERE part of an SQL clause.
   * @param {string[]} [options.contentFilter.parameters=undefined] - Array of strings that give values to
   *  the ‘parameters’ (i.e., "%n" tokens) in the filter_expression. The number of supplied parameters must
   *  fit with the requested values in the filter_expression (i.e., the number of %n tokens). default: undefined.
   * @param {SubscriptionCallback} callback - The callback to be call when receiving the topic subscribed. The topic will be an instance of null-terminated Buffer when options.isRaw is true.
   * @param {SubscriptionEventCallbacks} eventCallbacks - The event callbacks for the subscription.
   * @return {Subscription} - An instance of Subscription.
   * @throws {ERROR} - May throw an RMW error if content-filter is malformed. 
   * @see {@link SubscriptionCallback}
   * @see {@link https://www.omg.org/spec/DDS/1.4/PDF|Content-filter details at DDS 1.4 specification, Annex B}
   */
  createSubscription(typeClass, topic, options, callback, eventCallbacks) {
    if (typeof typeClass === 'string' || typeof typeClass === 'object') {
      typeClass = loader.loadInterface(typeClass);
    }

    if (typeof options === 'function') {
      callback = options;
      options = undefined;
    }
    options = this._validateOptions(options);

    if (typeof typeClass !== 'function') {
      throw new TypeValidationError('typeClass', typeClass, 'function', {
        nodeName: this.name(),
        entityType: 'subscription',
      });
    }
    if (typeof topic !== 'string') {
      throw new TypeValidationError('topic', topic, 'string', {
        nodeName: this.name(),
        entityType: 'subscription',
      });
    }
    if (typeof callback !== 'function') {
      throw new TypeValidationError('callback', callback, 'function', {
        nodeName: this.name(),
        entityType: 'subscription',
        entityName: topic,
      });
    }
    if (
      eventCallbacks &&
      !(eventCallbacks instanceof SubscriptionEventCallbacks)
    ) {
      throw new TypeValidationError(
        'eventCallbacks',
        eventCallbacks,
        'SubscriptionEventCallbacks',
        {
          nodeName: this.name(),
          entityType: 'subscription',
          entityName: topic,
        }
      );
    }

    let subscription = Subscription.createSubscription(
      this,
      typeClass,
      topic,
      options,
      callback,
      eventCallbacks
    );
    debug('Finish creating subscription, topic = %s.', topic);
    this._subscriptions.push(subscription);
    this.syncHandles();

    return subscription;
  }

  /**
   * Create a Client.
   * @param {function|string|object} typeClass - The ROS message class,
        OR a string representing the message class, e.g. 'std_msgs/msg/String',
        OR an object representing the message class, e.g. {package: 'std_msgs', type: 'msg', name: 'String'}
   * @param {string} serviceName - The service name to request.
   * @param {object} options - The options argument used to parameterize the client.
   * @param {boolean} options.enableTypedArray - The response will use TypedArray if necessary, default: true.
   * @param {QoS} options.qos - ROS Middleware "quality of service" settings for the client, default: QoS.profileDefault.
   * @return {Client} - An instance of Client.
   */
  createClient(typeClass, serviceName, options) {
    if (typeof typeClass === 'string' || typeof typeClass === 'object') {
      typeClass = loader.loadInterface(typeClass);
    }
    options = this._validateOptions(options);

    if (typeof typeClass !== 'function') {
      throw new TypeValidationError('typeClass', typeClass, 'function', {
        nodeName: this.name(),
        entityType: 'client',
      });
    }
    if (typeof serviceName !== 'string') {
      throw new TypeValidationError('serviceName', serviceName, 'string', {
        nodeName: this.name(),
        entityType: 'client',
      });
    }

    let client = Client.createClient(
      this.handle,
      serviceName,
      typeClass,
      options
    );
    debug('Finish creating client, service = %s.', serviceName);
    this._clients.push(client);
    this.syncHandles();

    return client;
  }

  /**
   * This callback is called when a request is sent to service
   * @callback RequestCallback
   * @param {Object} request - The request sent to the service
   * @param {Response} response - The response to client.
        Use [response.send()]{@link Response#send} to send response object to client
   * @return {undefined}
   * @see [Node.createService]{@link Node#createService}
   * @see [Client.sendRequest]{@link Client#sendRequest}
   * @see {@link Client}
   * @see {@link Service}
   * @see {@link Response#send}
   */

  /**
   * Create a Service.
   * @param {function|string|object} typeClass - The ROS message class,
        OR a string representing the message class, e.g. 'std_msgs/msg/String',
        OR an object representing the message class, e.g. {package: 'std_msgs', type: 'msg', name: 'String'}
   * @param {string} serviceName - The service name to offer.
   * @param {object} options - The options argument used to parameterize the service.
   * @param {boolean} options.enableTypedArray - The request will use TypedArray if necessary, default: true.
   * @param {QoS} options.qos - ROS Middleware "quality of service" settings for the service, default: QoS.profileDefault.
   * @param {RequestCallback} callback - The callback to be called when receiving request.
   * @return {Service} - An instance of Service.
   * @see {@link RequestCallback}
   */
  createService(typeClass, serviceName, options, callback) {
    if (typeof typeClass === 'string' || typeof typeClass === 'object') {
      typeClass = loader.loadInterface(typeClass);
    }

    if (typeof options === 'function') {
      callback = options;
      options = undefined;
    }
    options = this._validateOptions(options);

    if (typeof typeClass !== 'function') {
      throw new TypeValidationError('typeClass', typeClass, 'function', {
        nodeName: this.name(),
        entityType: 'service',
      });
    }
    if (typeof serviceName !== 'string') {
      throw new TypeValidationError('serviceName', serviceName, 'string', {
        nodeName: this.name(),
        entityType: 'service',
      });
    }
    if (typeof callback !== 'function') {
      throw new TypeValidationError('callback', callback, 'function', {
        nodeName: this.name(),
        entityType: 'service',
        entityName: serviceName,
      });
    }

    let service = Service.createService(
      this.handle,
      serviceName,
      typeClass,
      options,
      callback
    );
    debug('Finish creating service, service = %s.', serviceName);
    this._services.push(service);
    this.syncHandles();

    return service;
  }

  /**
   * Create a ParameterClient for accessing parameters on a remote node.
   * @param {string} remoteNodeName - The name of the remote node whose parameters to access.
   * @param {object} [options] - Options for parameter client.
   * @param {number} [options.timeout=5000] - Default timeout in milliseconds for service calls.
   * @return {ParameterClient} - An instance of ParameterClient.
   */
  createParameterClient(remoteNodeName, options = {}) {
    if (typeof remoteNodeName !== 'string' || remoteNodeName.trim() === '') {
      throw new TypeError('Remote node name must be a non-empty string');
    }

    const parameterClient = new ParameterClient(this, remoteNodeName, options);
    debug(
      'Finish creating parameter client for remote node = %s.',
      remoteNodeName
    );
    this._parameterClients.push(parameterClient);

    return parameterClient;
  }

  /**
   * Create a ParameterWatcher for watching parameter changes on a remote node.
   * @param {string} remoteNodeName - The name of the remote node whose parameters to watch.
   * @param {string[]} parameterNames - Array of parameter names to watch.
   * @param {object} [options] - Options for parameter watcher.
   * @param {number} [options.timeout=5000] - Default timeout in milliseconds for service calls.
   * @return {ParameterWatcher} - An instance of ParameterWatcher.
   */
  createParameterWatcher(remoteNodeName, parameterNames, options = {}) {
    const watcher = new ParameterWatcher(
      this,
      remoteNodeName,
      parameterNames,
      options
    );
    debug(
      'Finish creating parameter watcher for remote node = %s.',
      remoteNodeName
    );
    this._parameterWatchers.push(watcher);

    return watcher;
  }

  /**
   * Create a guard condition.
   * @param {Function} callback - The callback to be called when the guard condition is triggered.
   * @return {GuardCondition} - An instance of GuardCondition.
   */
  createGuardCondition(callback) {
    if (typeof callback !== 'function') {
      throw new TypeValidationError('callback', callback, 'function', {
        nodeName: this.name(),
        entityType: 'guard_condition',
      });
    }

    let guard = GuardCondition.createGuardCondition(callback, this.context);
    debug('Finish creating guard condition');
    this._guards.push(guard);
    this.syncHandles();

    return guard;
  }

  /**
   * Destroy all resource allocated by this node, including
   * <code>Timer</code>s/<code>Publisher</code>s/<code>Subscription</code>s
   * /<code>Client</code>s/<code>Service</code>s
   * @return {undefined}
   */
  destroy() {
    if (this.spinning) {
      this.stop();
    }

    // Action servers/clients require manual destruction due to circular reference with goal handles.
    this._actionClients.forEach((actionClient) => actionClient.destroy());
    this._actionServers.forEach((actionServer) => actionServer.destroy());

    this._parameterClients.forEach((paramClient) => paramClient.destroy());
    this._parameterWatchers.forEach((watcher) => watcher.destroy());

    this.context.onNodeDestroyed(this);

    this.handle.release();
    this._clock = null;
    this._timers = [];
    this._publishers = [];
    this._subscriptions = [];
    this._clients = [];
    this._services = [];
    this._guards = [];
    this._actionClients = [];
    this._actionServers = [];
    this._parameterClients = [];
    this._parameterWatchers = [];

    if (this._rateTimerServer) {
      this._rateTimerServer.shutdown();
      this._rateTimerServer = null;
    }
  }

  /**
   * Destroy a Publisher.
   * @param {Publisher} publisher - The Publisher to be destroyed.
   * @return {undefined}
   */
  destroyPublisher(publisher) {
    if (!(publisher instanceof Publisher)) {
      throw new TypeValidationError(
        'publisher',
        publisher,
        'Publisher instance',
        {
          nodeName: this.name(),
        }
      );
    }
    if (publisher.events) {
      publisher.events.forEach((event) => {
        this._destroyEntity(event, this._events);
      });
      publisher.events = [];
    }
    this._destroyEntity(publisher, this._publishers, false);
  }

  /**
   * Destroy a Subscription.
   * @param {Subscription} subscription - The Subscription to be destroyed.
   * @return {undefined}
   */
  destroySubscription(subscription) {
    if (!(subscription instanceof Subscription)) {
      throw new TypeValidationError(
        'subscription',
        subscription,
        'Subscription instance',
        {
          nodeName: this.name(),
        }
      );
    }
    if (subscription.events) {
      subscription.events.forEach((event) => {
        this._destroyEntity(event, this._events);
      });
      subscription.events = [];
    }

    this._destroyEntity(subscription, this._subscriptions);
  }

  /**
   * Destroy a Client.
   * @param {Client} client - The Client to be destroyed.
   * @return {undefined}
   */
  destroyClient(client) {
    if (!(client instanceof Client)) {
      throw new TypeValidationError('client', client, 'Client instance', {
        nodeName: this.name(),
      });
    }
    this._destroyEntity(client, this._clients);
  }

  /**
   * Destroy a Service.
   * @param {Service} service - The Service to be destroyed.
   * @return {undefined}
   */
  destroyService(service) {
    if (!(service instanceof Service)) {
      throw new TypeValidationError('service', service, 'Service instance', {
        nodeName: this.name(),
      });
    }
    this._destroyEntity(service, this._services);
  }

  /**
   * Destroy a ParameterClient.
   * @param {ParameterClient} parameterClient - The ParameterClient to be destroyed.
   * @return {undefined}
   */
  destroyParameterClient(parameterClient) {
    if (!(parameterClient instanceof ParameterClient)) {
      throw new TypeError('Invalid argument');
    }
    this._removeEntityFromArray(parameterClient, this._parameterClients);
    parameterClient.destroy();
  }

  /**
   * Destroy a ParameterWatcher.
   * @param {ParameterWatcher} watcher - The ParameterWatcher to be destroyed.
   * @return {undefined}
   */
  destroyParameterWatcher(watcher) {
    if (!(watcher instanceof ParameterWatcher)) {
      throw new TypeError('Invalid argument');
    }
    this._removeEntityFromArray(watcher, this._parameterWatchers);
    watcher.destroy();
  }

  /**
   * Destroy a Timer.
   * @param {Timer} timer - The Timer to be destroyed.
   * @return {undefined}
   */
  destroyTimer(timer) {
    if (!(timer instanceof Timer)) {
      throw new TypeValidationError('timer', timer, 'Timer instance', {
        nodeName: this.name(),
      });
    }
    this._destroyEntity(timer, this._timers);
  }

  /**
   * Destroy a guard condition.
   * @param {GuardCondition} guard - The guard condition to be destroyed.
   * @return {undefined}
   */
  destroyGuardCondition(guard) {
    if (!(guard instanceof GuardCondition)) {
      throw new TypeValidationError('guard', guard, 'GuardCondition instance', {
        nodeName: this.name(),
      });
    }
    this._destroyEntity(guard, this._guards);
  }

  /**
   * Get the name of the node.
   * @return {string}
   */
  name() {
    return rclnodejs.getNodeName(this.handle);
  }

  /**
   * Get the namespace of the node.
   * @return {string}
   */
  namespace() {
    return rclnodejs.getNamespace(this.handle);
  }

  /**
   * Get the context in which this node was created.
   * @return {Context}
   */
  get context() {
    return this._context;
  }

  /**
   * Get the nodes logger.
   * @returns {Logger} - The logger for the node.
   */
  getLogger() {
    return this._logger;
  }

  /**
   * Get the clock used by the node.
   * @returns {Clock} - The nodes clock.
   */
  getClock() {
    return this._clock;
  }

  /**
   * Get the current time using the node's clock.
   * @returns {Timer} - The current time.
   */
  now() {
    return this.getClock().now();
  }

  /**
   * Get the list of published topics discovered by the provided node for the remote node name.
   * @param {string} nodeName - The name of the node.
   * @param {string} namespace - The name of the namespace.
   * @param {boolean} noDemangle - If true topic names and types returned will not be demangled, default: false.
   * @return {Array<{name: string, types: Array<string>}>} - An array of the names and types.
   */
  getPublisherNamesAndTypesByNode(nodeName, namespace, noDemangle = false) {
    return rclnodejs.getPublisherNamesAndTypesByNode(
      this.handle,
      nodeName,
      namespace,
      noDemangle
    );
  }

  /**
   * Get the list of published topics discovered by the provided node for the remote node name.
   * @param {string} nodeName - The name of the node.
   * @param {string} namespace - The name of the namespace.
   * @param {boolean} noDemangle - If true topic names and types returned will not be demangled, default: false.
   * @return {Array<{name: string, types: Array<string>}>} - An array of the names and types.
   */
  getSubscriptionNamesAndTypesByNode(nodeName, namespace, noDemangle = false) {
    return rclnodejs.getSubscriptionNamesAndTypesByNode(
      this.handle,
      nodeName,
      namespace,
      noDemangle
    );
  }

  /**
   * Get service names and types for which a remote node has servers.
   * @param {string} nodeName - The name of the node.
   * @param {string} namespace - The name of the namespace.
   * @return {Array<{name: string, types: Array<string>}>} - An array of the names and types.
   */
  getServiceNamesAndTypesByNode(nodeName, namespace) {
    return rclnodejs.getServiceNamesAndTypesByNode(
      this.handle,
      nodeName,
      namespace
    );
  }

  /**
   * Get service names and types for which a remote node has clients.
   * @param {string} nodeName - The name of the node.
   * @param {string} namespace - The name of the namespace.
   * @return {Array<{name: string, types: Array<string>}>} - An array of the names and types.
   */
  getClientNamesAndTypesByNode(nodeName, namespace) {
    return rclnodejs.getClientNamesAndTypesByNode(
      this.handle,
      nodeName,
      namespace
    );
  }

  /**
   * Get the list of topics discovered by the provided node.
   * @param {boolean} noDemangle - If true topic names and types returned will not be demangled, default: false.
   * @return {Array<{name: string, types: Array<string>}>} - An array of the names and types.
   */
  getTopicNamesAndTypes(noDemangle = false) {
    return rclnodejs.getTopicNamesAndTypes(this.handle, noDemangle);
  }

  /**
   * Get the list of services discovered by the provided node.
   * @return {Array<{name: string, types: Array<string>}>} - An array of the names and types.
   */
  getServiceNamesAndTypes() {
    return rclnodejs.getServiceNamesAndTypes(this.handle);
  }

  /**
   * Return a list of publishers on a given topic.
   *
   * The returned parameter is a list of TopicEndpointInfo objects, where each will contain
   * the node name, node namespace, topic type, topic endpoint's GID, and its QoS profile.
   *
   * When the `no_mangle` parameter is `true`, the provided `topic` should be a valid
   * topic name for the middleware (useful when combining ROS with native middleware (e.g. DDS)
   * apps).  When the `no_mangle` parameter is `false`, the provided `topic` should
   * follow ROS topic name conventions.
   *
   * `topic` may be a relative, private, or fully qualified topic name.
   *  A relative or private topic will be expanded using this node's namespace and name.
   *  The queried `topic` is not remapped.
   *
   * @param {string} topic - The topic on which to find the publishers.
   * @param {boolean} [noDemangle=false] - If `true`, `topic` needs to be a valid middleware topic
   *                               name, otherwise it should be a valid ROS topic name. Defaults to `false`.
   * @returns {Array} - list of publishers
   */
  getPublishersInfoByTopic(topic, noDemangle = false) {
    return rclnodejs.getPublishersInfoByTopic(
      this.handle,
      this._getValidatedTopic(topic, noDemangle),
      noDemangle
    );
  }

  /**
   * Return a list of subscriptions on a given topic.
   *
   * The returned parameter is a list of TopicEndpointInfo objects, where each will contain
   * the node name, node namespace, topic type, topic endpoint's GID, and its QoS profile.
   *
   * When the `no_mangle` parameter is `true`, the provided `topic` should be a valid
   * topic name for the middleware (useful when combining ROS with native middleware (e.g. DDS)
   * apps).  When the `no_mangle` parameter is `false`, the provided `topic` should
   * follow ROS topic name conventions.
   *
   * `topic` may be a relative, private, or fully qualified topic name.
   *  A relative or private topic will be expanded using this node's namespace and name.
   *  The queried `topic` is not remapped.
   *
   * @param {string} topic - The topic on which to find the subscriptions.
   * @param {boolean} [noDemangle=false] -  If `true`, `topic` needs to be a valid middleware topic
                                    name, otherwise it should be a valid ROS topic name. Defaults to `false`.
   * @returns {Array} - list of subscriptions
   */
  getSubscriptionsInfoByTopic(topic, noDemangle = false) {
    return rclnodejs.getSubscriptionsInfoByTopic(
      this.handle,
      this._getValidatedTopic(topic, noDemangle),
      noDemangle
    );
  }

  /**
   * Get the list of nodes discovered by the provided node.
   * @return {Array<string>} - An array of the names.
   */
  getNodeNames() {
    return this.getNodeNamesAndNamespaces().map((item) => item.name);
  }

  /**
   * Get the list of nodes and their namespaces discovered by the provided node.
   * @return {Array<{name: string, namespace: string}>} An array of the names and namespaces.
   */
  getNodeNamesAndNamespaces() {
    return rclnodejs.getNodeNames(this.handle, /*getEnclaves=*/ false);
  }

  /**
   * Get the list of nodes and their namespaces with enclaves discovered by the provided node.
   * @return {Array<{name: string, namespace: string, enclave: string}>} An array of the names, namespaces and enclaves.
   */
  getNodeNamesAndNamespacesWithEnclaves() {
    return rclnodejs.getNodeNames(this.handle, /*getEnclaves=*/ true);
  }

  /**
   * Return the number of publishers on a given topic.
   * @param {string} topic - The name of the topic.
   * @returns {number} - Number of publishers on the given topic.
   */
  countPublishers(topic) {
    let expandedTopic = rclnodejs.expandTopicName(
      topic,
      this.name(),
      this.namespace()
    );
    rclnodejs.validateTopicName(expandedTopic);

    return rclnodejs.countPublishers(this.handle, expandedTopic);
  }

  /**
   * Return the number of subscribers on a given topic.
   * @param {string} topic - The name of the topic.
   * @returns {number} - Number of subscribers on the given topic.
   */
  countSubscribers(topic) {
    let expandedTopic = rclnodejs.expandTopicName(
      topic,
      this.name(),
      this.namespace()
    );
    rclnodejs.validateTopicName(expandedTopic);

    return rclnodejs.countSubscribers(this.handle, expandedTopic);
  }

  /**
   * Get the number of clients on a given service name.
   * @param {string} serviceName - the service name
   * @returns {Number}
   */
  countClients(serviceName) {
    if (DistroUtils.getDistroId() <= DistroUtils.getDistroId('humble')) {
      console.warn('countClients is not supported by this version of ROS 2');
      return null;
    }
    return rclnodejs.countClients(this.handle, serviceName);
  }

  /**
   * Get the number of services on a given service name.
   * @param {string} serviceName - the service name
   * @returns {Number}
   */
  countServices(serviceName) {
    if (DistroUtils.getDistroId() <= DistroUtils.getDistroId('humble')) {
      console.warn('countServices is not supported by this version of ROS 2');
      return null;
    }
    return rclnodejs.countServices(this.handle, serviceName);
  }

  /**
   * Get the list of parameter-overrides found on the commandline and
   * in the NodeOptions.parameter_overrides property.
   *
   * @return {Array<Parameter>} - An array of Parameters.
   */
  getParameterOverrides() {
    return Array.from(this._parameterOverrides.values());
  }

  /**
   * Declare a parameter.
   *
   * Internally, register a parameter and it's descriptor.
   * If a parameter-override exists, it's value will replace that of the parameter
   * unless ignoreOverride is true.
   * If the descriptor is undefined, then a ParameterDescriptor will be inferred
   * from the parameter's state.
   *
   * If a parameter by the same name has already been declared then an Error is thrown.
   * A parameter must be undeclared before attempting to redeclare it.
   *
   * @param {Parameter} parameter - Parameter to declare.
   * @param {ParameterDescriptor} [descriptor] - Optional descriptor for parameter.
   * @param {boolean} [ignoreOverride] - When true disregard any parameter-override that may be present.
   * @return {Parameter} - The newly declared parameter.
   */
  declareParameter(parameter, descriptor, ignoreOverride = false) {
    const parameters = this.declareParameters(
      [parameter],
      descriptor ? [descriptor] : [],
      ignoreOverride
    );
    return parameters.length == 1 ? parameters[0] : null;
  }

  /**
   * Declare a list of parameters.
   *
   * Internally register parameters with their corresponding descriptor one by one
   * in the order they are provided. This is an atomic operation. If an error
   * occurs the process halts and no further parameters are declared.
   * Parameters that have already been processed are undeclared.
   *
   * While descriptors is an optional parameter, when provided there must be
   * a descriptor for each parameter; otherwise an Error is thrown.
   * If descriptors is not provided then a descriptor will be inferred
   * from each parameter's state.
   *
   * When a parameter-override is available, the parameter's value
   * will be replaced with that of the parameter-override unless ignoreOverrides
   * is true.
   *
   * If a parameter by the same name has already been declared then an Error is thrown.
   * A parameter must be undeclared before attempting to redeclare it.
   *
   * Prior to declaring the parameters each SetParameterEventCallback registered
   * using setOnParameterEventCallback() is called in succession with the parameters
   * list. Any SetParameterEventCallback that retuns does not return a successful
   * result will cause the entire operation to terminate with no changes to the
   * parameters. When all SetParameterEventCallbacks return successful then the
   * list of parameters is updated.
   *
   * @param {Parameter[]} parameters - The parameters to declare.
   * @param {ParameterDescriptor[]} [descriptors] - Optional descriptors,
   *    a 1-1 correspondence with parameters.
   * @param {boolean} ignoreOverrides - When true, parameter-overrides are
   *    not considered, i.e.,ignored.
   * @return {Parameter[]} - The declared parameters.
   */
  declareParameters(parameters, descriptors = [], ignoreOverrides = false) {
    if (!Array.isArray(parameters)) {
      throw new TypeValidationError('parameters', parameters, 'Array', {
        nodeName: this.name(),
      });
    }
    if (!Array.isArray(descriptors)) {
      throw new TypeValidationError('descriptors', descriptors, 'Array', {
        nodeName: this.name(),
      });
    }
    if (descriptors.length > 0 && parameters.length !== descriptors.length) {
      throw new ValidationError(
        'Each parameter must have a corresponding ParameterDescriptor',
        {
          code: 'PARAMETER_DESCRIPTOR_MISMATCH',
          argumentName: 'descriptors',
          providedValue: descriptors.length,
          expectedType: `Array with length ${parameters.length}`,
          nodeName: this.name(),
        }
      );
    }

    const declaredDescriptors = [];
    const declaredParameters = [];
    const declaredParameterCollisions = [];
    for (let i = 0; i < parameters.length; i++) {
      let parameter =
        !ignoreOverrides && this._parameterOverrides.has(parameters[i].name)
          ? this._parameterOverrides.get(parameters[i].name)
          : parameters[i];

      // stop processing parameters that have already been declared
      if (this._parameters.has(parameter.name)) {
        declaredParameterCollisions.push(parameter);
        continue;
      }

      // create descriptor for parameter if not provided
      let descriptor =
        descriptors.length > 0
          ? descriptors[i]
          : ParameterDescriptor.fromParameter(parameter);

      descriptor.validate();

      declaredDescriptors.push(descriptor);
      declaredParameters.push(parameter);
    }

    if (declaredParameterCollisions.length > 0) {
      const errorMsg =
        declaredParameterCollisions.length == 1
          ? `Parameter(${declaredParameterCollisions[0]}) already declared.`
          : `Multiple parameters already declared, e.g., Parameter(${declaredParameterCollisions[0]}).`;
      throw new Error(errorMsg);
    }

    // register descriptor
    for (const descriptor of declaredDescriptors) {
      this._parameterDescriptors.set(descriptor.name, descriptor);
    }

    const result = this._setParametersAtomically(declaredParameters, true);
    if (!result.successful) {
      // unregister descriptors
      for (const descriptor of declaredDescriptors) {
        this._parameterDescriptors.delete(descriptor.name);
      }

      throw new Error(result.reason);
    }

    return this.getParameters(declaredParameters.map((param) => param.name));
  }

  /**
   * Undeclare a parameter.
   *
   * Readonly parameters can not be undeclared or updated.
   * @param {string} name - Name of parameter to undeclare.
   * @return {undefined} -
   */
  undeclareParameter(name) {
    if (!this.hasParameter(name)) return;

    const descriptor = this.getParameterDescriptor(name);
    if (descriptor.readOnly) {
      throw new Error(
        `${name} parameter is read-only and can not be undeclared`
      );
    }

    this._parameters.delete(name);
    this._parameterDescriptors.delete(name);
  }

  /**
   * Determine if a parameter has been declared.
   * @param {string} name - name of parameter
   * @returns {boolean} - Return true if parameter is declared; false otherwise.
   */
  hasParameter(name) {
    return this._parameters.has(name);
  }

  /**
   * Get a declared parameter by name.
   *
   * If unable to locate a declared parameter then a
   * parameter with type == PARAMETER_NOT_SET is returned.
   *
   * @param {string} name - The name of the parameter.
   * @return {Parameter} - The parameter.
   */
  getParameter(name) {
    return this.getParameters([name])[0];
  }

  /**
   * Get a list of parameters.
   *
   * Find and return the declared parameters.
   * If no names are provided return all declared parameters.
   *
   * If unable to locate a declared parameter then a
   * parameter with type == PARAMETER_NOT_SET is returned in
   * it's place.
   *
   * @param {string[]} [names] - The names of the declared parameters
   *    to find or null indicating to return all declared parameters.
   * @return {Parameter[]} - The parameters.
   */
  getParameters(names = []) {
    let params = [];

    if (names.length == 0) {
      // get all parameters
      params = [...this._parameters.values()];
      return params;
    }

    for (const name of names) {
      const param = this.hasParameter(name)
        ? this._parameters.get(name)
        : new Parameter(name, ParameterType.PARAMETER_NOT_SET);

      params.push(param);
    }

    return params;
  }

  /**
   * Get the types of given parameters.
   *
   * Return the types of given parameters.
   *
   * @param {string[]} [names] - The names of the declared parameters.
   * @return {Uint8Array} - The types.
   */
  getParameterTypes(names = []) {
    let types = [];

    for (const name of names) {
      const descriptor = this._parameterDescriptors.get(name);
      if (descriptor) {
        types.push(descriptor.type);
      }
    }
    return types;
  }

  /**
   * Get the names of all declared parameters.
   *
   * @return {Array<string>} - The declared parameter names or empty array if
   *    no parameters have been declared.
   */
  getParameterNames() {
    return this.getParameters().map((param) => param.name);
  }

  /**
   * Determine if a parameter descriptor exists.
   *
   * @param {string} name - The name of a descriptor to for.
   * @return {boolean} - true if a descriptor has been declared; otherwise false.
   */
  hasParameterDescriptor(name) {
    return !!this.getParameterDescriptor(name);
  }

  /**
   * Get a declared parameter descriptor by name.
   *
   * If unable to locate a declared parameter descriptor then a
   * descriptor with type == PARAMETER_NOT_SET is returned.
   *
   * @param {string} name - The name of the parameter descriptor to find.
   * @return {ParameterDescriptor} - The parameter descriptor.
   */
  getParameterDescriptor(name) {
    return this.getParameterDescriptors([name])[0];
  }

  /**
   * Find a list of declared ParameterDescriptors.
   *
   * If no names are provided return all declared descriptors.
   *
   * If unable to locate a declared descriptor then a
   * descriptor with type == PARAMETER_NOT_SET is returned in
   * it's place.
   *
   * @param {string[]} [names] - The names of the declared parameter
   *    descriptors to find or null indicating to return all declared descriptors.
   * @return {ParameterDescriptor[]} - The parameter descriptors.
   */
  getParameterDescriptors(names = []) {
    let descriptors = [];

    if (names.length == 0) {
      // get all parameters
      descriptors = [...this._parameterDescriptors.values()];
      return descriptors;
    }

    for (const name of names) {
      let descriptor = this._parameterDescriptors.get(name);
      if (!descriptor) {
        descriptor = new ParameterDescriptor(
          name,
          ParameterType.PARAMETER_NOT_SET
        );
      }
      descriptors.push(descriptor);
    }

    return descriptors;
  }

  /**
   * Replace a declared parameter.
   *
   * The parameter being replaced must be a declared parameter who's descriptor
   * is not readOnly; otherwise an Error is thrown.
   *
   * @param {Parameter} parameter - The new parameter.
   * @return {rclnodejs.rcl_interfaces.msg.SetParameterResult} - The result of the operation.
   */
  setParameter(parameter) {
    const results = this.setParameters([parameter]);
    return results[0];
  }

  /**
   * Replace a list of declared parameters.
   *
   * Declared parameters are replaced in the order they are provided and
   * a ParameterEvent is published for each individual parameter change.
   *
   * Prior to setting the parameters each SetParameterEventCallback registered
   * using setOnParameterEventCallback() is called in succession with the parameters
   * list. Any SetParameterEventCallback that retuns does not return a successful
   * result will cause the entire operation to terminate with no changes to the
   * parameters. When all SetParameterEventCallbacks return successful then the
   * list of parameters is updated.
   *
   * If an error occurs, the process is stopped and returned. Parameters
   * set before an error remain unchanged.
   *
   * @param {Parameter[]} parameters - The parameters to set.
   * @return {rclnodejs.rcl_interfaces.msg.SetParameterResult[]} - A list of SetParameterResult, one for each parameter that was set.
   */
  setParameters(parameters = []) {
    return parameters.map((parameter) =>
      this.setParametersAtomically([parameter])
    );
  }

  /**
   * Repalce a list of declared parameters atomically.
   *
   * Declared parameters are replaced in the order they are provided.
   * A single ParameterEvent is published collectively for all changed
   * parameters.
   *
   * Prior to setting the parameters each SetParameterEventCallback registered
   * using setOnParameterEventCallback() is called in succession with the parameters
   * list. Any SetParameterEventCallback that retuns does not return a successful
   * result will cause the entire operation to terminate with no changes to the
   * parameters. When all SetParameterEventCallbacks return successful then the
   * list of parameters is updated.d
   *
   * If an error occurs, the process stops immediately. All parameters updated to
   * the point of the error are reverted to their previous state.
   *
   * @param {Parameter[]} parameters - The parameters to set.
   * @return {rclnodejs.rcl_interfaces.msg.SetParameterResult} - describes the result of setting 1 or more parameters.
   */
  setParametersAtomically(parameters = []) {
    return this._setParametersAtomically(parameters);
  }

  /**
   * Internal method for updating parameters atomically.
   *
   * Prior to setting the parameters each SetParameterEventCallback registered
   * using setOnParameterEventCallback() is called in succession with the parameters
   * list. Any SetParameterEventCallback that retuns does not return a successful
   * result will cause the entire operation to terminate with no changes to the
   * parameters. When all SetParameterEventCallbacks return successful then the
   * list of parameters is updated.
   *
   * @param {Paramerter[]} parameters - The parameters to update.
   * @param {boolean} declareParameterMode - When true parameters are being declared;
   *    otherwise they are being changed.
   * @return {SetParameterResult} - A single collective result.
   */
  _setParametersAtomically(parameters = [], declareParameterMode = false) {
    let result = this._validateParameters(parameters, declareParameterMode);
    if (!result.successful) {
      return result;
    }

    // give all SetParametersCallbacks a chance to veto this change
    for (const callback of this._setParametersCallbacks) {
      result = callback(parameters);
      if (!result.successful) {
        // a callback has vetoed a parameter change
        return result;
      }
    }

    // collectively track updates to parameters for use
    // when publishing a ParameterEvent
    const newParameters = [];
    const changedParameters = [];
    const deletedParameters = [];

    for (const parameter of parameters) {
      if (parameter.type == ParameterType.PARAMETER_NOT_SET) {
        this.undeclareParameter(parameter.name);
        deletedParameters.push(parameter);
      } else {
        this._parameters.set(parameter.name, parameter);
        if (declareParameterMode) {
          newParameters.push(parameter);
        } else {
          changedParameters.push(parameter);
        }
      }
    }

    // create ParameterEvent
    const parameterEvent = new (loader.loadInterface(
      PARAMETER_EVENT_MSG_TYPE
    ))();

    const { seconds, nanoseconds } = this._clock.now().secondsAndNanoseconds;
    parameterEvent.stamp = {
      sec: Number(seconds),
      nanosec: Number(nanoseconds),
    };

    parameterEvent.node =
      this.namespace() === '/'
        ? this.namespace() + this.name()
        : this.namespace() + '/' + this.name();

    if (newParameters.length > 0) {
      parameterEvent['new_parameters'] = newParameters.map((parameter) =>
        parameter.toParameterMessage()
      );
    }
    if (changedParameters.length > 0) {
      parameterEvent['changed_parameters'] = changedParameters.map(
        (parameter) => parameter.toParameterMessage()
      );
    }
    if (deletedParameters.length > 0) {
      parameterEvent['deleted_parameters'] = deletedParameters.map(
        (parameter) => parameter.toParameterMessage()
      );
    }

    // Publish ParameterEvent.
    this._parameterEventPublisher.publish(parameterEvent);

    return {
      successful: true,
      reason: '',
    };
  }

  /**
   * This callback is called when declaring a parameter or setting a parameter.
   * The callback is provided a list of parameters and returns a SetParameterResult
   * to indicate approval or veto of the operation.
   *
   * @callback SetParametersCallback
   * @param {Parameter[]} parameters - The message published
   * @returns {rcl_interfaces.msg.SetParameterResult} -
   *
   * @see [Node.addOnSetParametersCallback]{@link Node#addOnSetParametersCallback}
   * @see [Node.removeOnSetParametersCallback]{@link Node#removeOnSetParametersCallback}
   */

  /**
   * Add a callback to the front of the list of callbacks invoked for parameter declaration
   * and setting. No checks are made for duplicate callbacks.
   *
   * @param {SetParametersCallback} callback - The callback to add.
   * @returns {undefined}
   */
  addOnSetParametersCallback(callback) {
    this._setParametersCallbacks.unshift(callback);
  }

  /**
   * Remove a callback from the list of SetParametersCallbacks.
   * If the callback is not found the process is a nop.
   *
   * @param {SetParametersCallback} callback - The callback to be removed
   * @returns {undefined}
   */
  removeOnSetParametersCallback(callback) {
    const idx = this._setParametersCallbacks.indexOf(callback);
    if (idx > -1) {
      this._setParametersCallbacks.splice(idx, 1);
    }
  }

  /**
   * Get the fully qualified name of the node.
   *
   * @returns {string} - String containing the fully qualified name of the node.
   */
  getFullyQualifiedName() {
    return rclnodejs.getFullyQualifiedName(this.handle);
  }

  /**
   * Get the RMW implementation identifier
   * @returns {string} - The RMW implementation identifier.
   */
  getRMWImplementationIdentifier() {
    return rclnodejs.getRMWImplementationIdentifier();
  }

  /**
   * Return a topic name expanded and remapped.
   * @param {string} topicName - Topic name to be expanded and remapped.
   * @param {boolean} [onlyExpand=false] - If `true`, remapping rules won't be applied.
   * @returns {string} - A fully qualified topic name.
   */
  resolveTopicName(topicName, onlyExpand = false) {
    if (typeof topicName !== 'string') {
      throw new TypeValidationError('topicName', topicName, 'string', {
        nodeName: this.name(),
      });
    }
    return rclnodejs.resolveName(
      this.handle,
      topicName,
      onlyExpand,
      /*isService=*/ false
    );
  }

  /**
   * Return a service name expanded and remapped.
   * @param {string} service - Service name to be expanded and remapped.
   * @param {boolean} [onlyExpand=false] - If `true`, remapping rules won't be applied.
   * @returns {string} - A fully qualified service name.
   */
  resolveServiceName(service, onlyExpand = false) {
    if (typeof service !== 'string') {
      throw new TypeValidationError('service', service, 'string', {
        nodeName: this.name(),
      });
    }
    return rclnodejs.resolveName(
      this.handle,
      service,
      onlyExpand,
      /*isService=*/ true
    );
  }

  // returns on 1st error or result {successful, reason}
  _validateParameters(parameters = [], declareParameterMode = false) {
    for (const parameter of parameters) {
      // detect invalid parameter
      try {
        parameter.validate();
      } catch {
        return {
          successful: false,
          reason: `Invalid ${parameter.name}`,
        };
      }

      // detect undeclared parameter
      if (!this.hasParameterDescriptor(parameter.name)) {
        return {
          successful: false,
          reason: `Parameter ${parameter.name} has not been declared`,
        };
      }

      // detect readonly parameter that can not be updated
      const descriptor = this.getParameterDescriptor(parameter.name);
      if (!declareParameterMode && descriptor.readOnly) {
        return {
          successful: false,
          reason: `Parameter ${parameter.name} is readonly`,
        };
      }

      // validate parameter against descriptor if not an undeclare action
      if (parameter.type != ParameterType.PARAMETER_NOT_SET) {
        try {
          descriptor.validateParameter(parameter);
        } catch {
          return {
            successful: false,
            reason: `Parameter ${parameter.name} does not  readonly`,
          };
        }
      }
    }

    return {
      successful: true,
      reason: null,
    };
  }

  // Get a Map(nodeName->Parameter[]) of CLI parameter args that
  // apply to 'this' node, .e.g., -p mynode:foo:=bar -p hello:=world
  _getNativeParameterOverrides() {
    const overrides = new Map();

    // Get native parameters from rcl context->global_arguments.
    // rclnodejs returns an array of objects, 1 for each node e.g., -p my_node:foo:=bar,
    // and a node named '/**' for global parameter rules,
    // i.e., does not include a node identifier, e.g., -p color:=red
    // {
    //   name: string // node name
    //   parameters[] = {
    //     name: string
    //     type: uint
    //     value: object
    // }
    const cliParamOverrideData = rclnodejs.getParameterOverrides(
      this.context.handle
    );

    // convert native CLI parameterOverrides to Map<nodeName,Array<ParameterOverride>>
    const cliParamOverrides = new Map();
    if (cliParamOverrideData) {
      for (let nodeParamData of cliParamOverrideData) {
        const nodeName = nodeParamData.name;
        const nodeParamOverrides = [];
        for (let paramData of nodeParamData.parameters) {
          const paramOverride = new Parameter(
            paramData.name,
            paramData.type,
            paramData.value
          );
          nodeParamOverrides.push(paramOverride);
        }
        cliParamOverrides.set(nodeName, nodeParamOverrides);
      }
    }

    // collect global CLI global parameters, name == /**
    let paramOverrides = cliParamOverrides.get('/**'); // array of ParameterOverrides
    if (paramOverrides) {
      for (const parameter of paramOverrides) {
        overrides.set(parameter.name, parameter);
      }
    }

    // merge CLI node parameterOverrides with global parameterOverrides, replace existing
    paramOverrides = cliParamOverrides.get(this.name()); // array of ParameterOverrides
    if (paramOverrides) {
      for (const parameter of paramOverrides) {
        overrides.set(parameter.name, parameter);
      }
    }

    return overrides;
  }

  /**
   * Invokes the callback with a raw message of the given type. After the callback completes
   * the message will be destroyed.
   * @param {function} Type - Message type to create.
   * @param {function} callback - Callback to invoke. First parameter will be the raw message,
   * and the second is a function to retrieve the deserialized message.
   * @returns {undefined}
   */
  _runWithMessageType(Type, callback) {
    let message = new Type();

    callback(message.toRawROS(), () => {
      let result = new Type();
      result.deserialize(message.refObject);

      return result;
    });

    Type.destroyRawROS(message);
  }

  _addActionClient(actionClient) {
    this._actionClients.push(actionClient);
    this.syncHandles();
  }

  _addActionServer(actionServer) {
    this._actionServers.push(actionServer);
    this.syncHandles();
  }

  _getValidatedTopic(topicName, noDemangle) {
    if (noDemangle) {
      return topicName;
    }
    const fqTopicName = rclnodejs.expandTopicName(
      topicName,
      this.name(),
      this.namespace()
    );
    validateFullTopicName(fqTopicName);
    return rclnodejs.remapTopicName(this.handle, fqTopicName);
  }
}

/**
 * Create an Options instance initialized with default values.
 * @returns {Options} - The new initialized instance.
 * @static
 * @example
 * {
 *   enableTypedArray: true,
 *   isRaw: false,
 *   qos: QoS.profileDefault,
 *   contentFilter: undefined,
 *   serializationMode: 'default',
 * }
 */
Node.getDefaultOptions = function () {
  return {
    enableTypedArray: true,
    isRaw: false,
    qos: QoS.profileDefault,
    contentFilter: undefined,
    serializationMode: 'default',
  };
};

module.exports = Node;
