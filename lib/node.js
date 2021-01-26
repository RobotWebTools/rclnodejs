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

const ActionInterfaces = require('./action/interfaces.js');
const Client = require('./client.js');
const Clock = require('./clock.js');
const Context = require('./context.js');
const debug = require('debug')('rclnodejs:node');
const GuardCondition = require('./guard_condition.js');
const loader = require('./interface_loader.js');
const Logging = require('./logging.js');
const NodeOptions = require('./node_options.js');
const {
  ParameterType,
  Parameter,
  ParameterDescriptor,
} = require('./parameter.js');
const ParameterService = require('./parameter_service.js');
const Publisher = require('./publisher.js');
const QoS = require('./qos.js');
const Rates = require('./rate.js');
const Service = require('./service.js');
const Subscription = require('./subscription.js');
const TimeSource = require('./time_source.js');
const Timer = require('./timer.js');

// Parameter event publisher constants
const PARAMETER_EVENT_MSG_TYPE = 'rcl_interfaces/msg/ParameterEvent';
const PARAMETER_EVENT_TOPIC = 'parameter_events';

/**
 * @class - Class representing a Node in ROS
 * @hideconstructor
 */

class Node extends rclnodejs.ShadowNode {
  constructor(
    nodeName,
    namespace = '',
    context = Context.defaultContext(),
    options = NodeOptions.defaultOptions
  ) {
    super();

    if (typeof nodeName !== 'string' || typeof namespace !== 'string') {
      throw new TypeError('Invalid argument.');
    }

    this._init(nodeName, namespace, options, context);
    debug(
      'Finish initializing node, name = %s and namespace = %s.',
      nodeName,
      namespace
    );
  }

  _init(name, namespace, options, context) {
    this.handle = rclnodejs.createNode(name, namespace, context.handle);
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
    this._actionClients = [];
    this._actionServers = [];
    this._rateTimerServer = null;
    this._parameterDescriptors = new Map();
    this._parameters = new Map();
    this._parameterService = null;
    this._parameterEventPublisher = null;
    this._setParametersCallbacks = [];
    this._logger = new Logging(rclnodejs.getNodeLoggerName(this.handle));
    this.spinning = false;

    this._parameterEventPublisher = this.createPublisher(
      PARAMETER_EVENT_MSG_TYPE,
      PARAMETER_EVENT_TOPIC
    );

    // initialize _parameterOverrides from parameters defined on the commandline
    this._parameterOverrides = this._getNativeParameterOverrides();

    // override cli parameterOverrides with those specified in options
    if (options.parameterOverrides.length > 0) {
      for (const parameter of options.parameterOverrides) {
        if (!parameter instanceof Parameter) {
          throw new TypeError(
            'Parameter-override must be an instance of Parameter.'
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
  }

  execute(handles) {
    let timersReady = this._timers.filter(
      (timer) => handles.indexOf(timer.handle) !== -1
    );
    let guardsReady = this._guards.filter(
      (guard) => handles.indexOf(guard.handle) !== -1
    );
    let subscriptionsReady = this._subscriptions.filter(
      (subscription) => handles.indexOf(subscription.handle) !== -1
    );
    let clientsReady = this._clients.filter(
      (client) => handles.indexOf(client.handle) !== -1
    );
    let servicesReady = this._services.filter(
      (service) => handles.indexOf(service.handle) !== -1
    );
    let actionClientsReady = this._actionClients.filter(
      (actionClient) => handles.indexOf(actionClient.handle) !== -1
    );
    let actionServersReady = this._actionServers.filter(
      (actionServer) => handles.indexOf(actionServer.handle) !== -1
    );

    timersReady.forEach((timer) => {
      if (timer.isReady()) {
        rclnodejs.callTimer(timer.handle);
        timer.callback();
      }
    });

    subscriptionsReady.forEach((subscription) => {
      if (subscription.isRaw) {
        let rawMessage = rclnodejs.rclTakeRaw(subscription.handle);
        if (rawMessage) {
          subscription.processResponse(rawMessage);
        }
        return;
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
    });

    guardsReady.forEach((guard) => {
      guard.callback();
    });

    clientsReady.forEach((client) => {
      this._runWithMessageType(
        client.typeClass.Response,
        (message, deserialize) => {
          let success = rclnodejs.rclTakeResponse(
            client.handle,
            client.sequenceNumber,
            message
          );
          if (success) {
            client.processResponse(deserialize());
          }
        }
      );
    });

    servicesReady.forEach((service) => {
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
    });

    actionClientsReady.forEach((actionClient) => {
      const properties = actionClient.handle.properties;

      if (properties.isGoalResponseReady) {
        this._runWithMessageType(
          actionClient.typeClass.impl.SendGoalService.Response,
          (message, deserialize) => {
            let sequence = rclnodejs.actionTakeGoalResponse(
              actionClient.handle,
              message
            );
            if (sequence != null) {
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
            if (sequence != null) {
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
            if (sequence != null) {
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
    });

    actionServersReady.forEach((actionServer) => {
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
    });
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
    this.spinning = true;
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
   * @returns {undfined}
   */
  stop() {
    super.stop();
    this.spinning = false;
  }

  /**
   * Terminate spinning - no further events will be received.
   * @returns {undfined}
   * @deprecated since 0.18.0, Use stop().
   */
  stopSpinning() {
    super.stop();
    this.spinning = false;
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
    this._removeEntityFromArray(entity, array);
    if (syncHandles) {
      this.syncHandles();
    }
    entity.handle.release();
  }

  _validateOptions(options) {
    if (
      options !== undefined &&
      (options === null || typeof options !== 'object')
    ) {
      throw new TypeError('Invalid argument of options');
    }

    if (options === undefined) {
      options = {
        enableTypedArray: true,
        isRaw: false,
        qos: QoS.profileDefault,
      };
      return options;
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

    return options;
  }

  /**
   * Create a Timer.
   * @param {number} period - The number representing period in millisecond.
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

    if (typeof period !== 'number' || typeof callback !== 'function') {
      throw new TypeError('Invalid argument');
    }

    // The period unit is millisecond in JavaScript side. When being passed to the
    // C++ side, the value will be converted to nanosecond, which goes into a uint64_t
    // with maxmium value of 2^64-1. So the maxmium is UINT64_MAX in ns, that's 0x10c6f7a0b5ed in ms.
    const MAX_TIMER_PERIOD_IN_MILLISECOND = 0x10c6f7a0b5ed;
    if (period > 0x10c6f7a0b5ed || period < 0) {
      throw new RangeError(
        `Parameter must be between 0.0 and ${MAX_TIMER_PERIOD_IN_MILLISECOND}`
      );
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
      throw new TypeError('Invalid argument');
    }

    const MAX_RATE_HZ_IN_MILLISECOND = 1000.0;
    if (hz <= 0.0 || hz > MAX_RATE_HZ_IN_MILLISECOND) {
      throw new RangeError(
        `Hz must be between 0.0 and ${MAX_RATE_HZ_IN_MILLISECOND}`
      );
    }

    // lazy initialize rateTimerServer
    if (!this._rateTimerServer) {
      this._rateTimerServer = new Rates.RateTimerServer(this);
      await this._rateTimerServer.init();
    }

    const period = Math.round(1000 / hz);
    const timer = this._rateTimerServer.createTimer(period);
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
   * @return {Publisher} - An instance of Publisher.
   */
  createPublisher(typeClass, topic, options) {
    return this._createPublisher(typeClass, topic, options, Publisher);
  }

  _createPublisher(typeClass, topic, options, publisherClass) {
    if (typeof typeClass === 'string' || typeof typeClass === 'object') {
      typeClass = loader.loadInterface(typeClass);
    }
    options = this._validateOptions(options);

    if (typeof typeClass !== 'function' || typeof topic !== 'string') {
      throw new TypeError('Invalid argument');
    }

    let publisher = publisherClass.createPublisher(
      this.handle,
      typeClass,
      topic,
      options
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
   * Create a Subscription.
   * @param {function|string|object} typeClass - The ROS message class,
        OR a string representing the message class, e.g. 'std_msgs/msg/String',
        OR an object representing the message class, e.g. {package: 'std_msgs', type: 'msg', name: 'String'}
   * @param {string} topic - The name of the topic.
   * @param {object} options - The options argument used to parameterize the subscription.
   * @param {boolean} options.enableTypedArray - The topic will use TypedArray if necessary, default: true.
   * @param {QoS} options.qos - ROS Middleware "quality of service" settings for the subscription, default: QoS.profileDefault.
   * @param {boolean} options.isRaw - The topic is serialized when true, default: false.
   * @param {SubscriptionCallback} callback - The callback to be call when receiving the topic subscribed. The topic will be an instance of null-terminated Buffer when options.isRaw is true.
   * @return {Subscription} - An instance of Subscription.
   * @see {@link SubscriptionCallback}
   */
  createSubscription(typeClass, topic, options, callback) {
    if (typeof typeClass === 'string' || typeof typeClass === 'object') {
      typeClass = loader.loadInterface(typeClass);
    }

    if (typeof options === 'function') {
      callback = options;
      options = undefined;
    }
    options = this._validateOptions(options);

    if (
      typeof typeClass !== 'function' ||
      typeof topic !== 'string' ||
      typeof callback !== 'function'
    ) {
      throw new TypeError('Invalid argument');
    }

    let subscription = Subscription.createSubscription(
      this.handle,
      typeClass,
      topic,
      options,
      callback
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

    if (typeof typeClass !== 'function' || typeof serviceName !== 'string') {
      throw new TypeError('Invalid argument');
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

    if (
      typeof typeClass !== 'function' ||
      typeof serviceName !== 'string' ||
      typeof callback !== 'function'
    ) {
      throw new TypeError('Invalid argument');
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
   * Create a guard condition.
   * @param {Function} callback - The callback to be called when the guard condition is triggered.
   * @return {GuardCondition} - An instance of GuardCondition.
   */
  createGuardCondition(callback) {
    if (typeof callback !== 'function') {
      throw new TypeError('Invalid argument');
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
      throw new TypeError('Invalid argument');
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
      throw new TypeError('Invalid argument');
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
      throw new TypeError('Invalid argument');
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
      throw new TypeError('Invalid argument');
    }
    this._destroyEntity(service, this._services);
  }

  /**
   * Destroy a Timer.
   * @param {Timer} timer - The Timer to be destroyed.
   * @return {undefined}
   */
  destroyTimer(timer) {
    if (!(timer instanceof Timer)) {
      throw new TypeError('Invalid argument');
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
      throw new TypeError('Invalid argument');
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
   * @returns {Time} - The current time.
   */
  now() {
    return this.getClock().now();
  }

  /**
   * Get the list of published topics discovered by the provided node for the remote node name.
   * @param {string} nodeName - The name of the node.
   * @param {string} namespace - The name of the namespace.
   * @param {boolean} noDemangle - If true topic names and types returned will not be demangled, default: false.
   * @return {array} - An array of the names and types.
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
   * @return {array} - An array of the names and types.
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
   * Get the list of service topics discovered by the provided node for the remote node name.
   * @param {string} nodeName - The name of the node.
   * @param {string} namespace - The name of the namespace.
   * @return {array} - An array of the names and types.
   */
  getServiceNamesAndTypesByNode(nodeName, namespace) {
    return rclnodejs.getServiceNamesAndTypesByNode(
      this.handle,
      nodeName,
      namespace
    );
  }

  /**
   * Get the list of topics discovered by the provided node.
   * @param {boolean} noDemangle - If true topic names and types returned will not be demangled, default: false.
   * @return {array} - An array of the names and types.
   */
  getTopicNamesAndTypes(noDemangle = false) {
    return rclnodejs.getTopicNamesAndTypes(this.handle, noDemangle);
  }

  /**
   * Get the list of services discovered by the provided node.
   * @return {array} - An array of the names and types.
   */
  getServiceNamesAndTypes() {
    return rclnodejs.getServiceNamesAndTypes(this.handle);
  }

  /**
   * Get the list of nodes discovered by the provided node.
   * @return {array} - An array of the names.
   */
  getNodeNames() {
    return this.getNodeNamesAndNamespaces().map((item) => item.name);
  }

  /**
   * Get the list of nodes and their namespaces discovered by the provided node.
   * @return {array} - An array of the names and namespaces.
   */
  getNodeNamesAndNamespaces() {
    return rclnodejs.getNodeNames(this.handle);
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
   * Get the list of parameter-overrides found on the commandline and
   * in the NodeOptions.parameter_overrides property.
   *
   * @return {array} - An array of Parameters
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
   * @param {boolean} [ignoreOveride] - When true disregard any parameter-override that may be present.
   * @return {Parameter} - The newly declared parameter.
   */
  declareParameter(parameter, descriptor, ignoreOveride = false) {
    const parameters = this.declareParameters(
      [parameter],
      descriptor ? [descriptor] : [],
      ignoreOveride
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
      throw new TypeError('Invalid parameter: expected array of Parameter');
    }
    if (!Array.isArray(descriptors)) {
      throw new TypeError(
        'Invalid parameters: expected array of ParameterDescriptor'
      );
    }
    if (descriptors.length > 0 && parameters.length !== descriptors.length) {
      throw new TypeError(
        'Each parameter must have a cooresponding ParameterDescriptor'
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
   * Get the names of all declared parameters.
   *
   * @return {string[]} - The declared parameter names or empty array if
   *    no parameters have been declared.
   */
  getParameterNames() {
    return this.getParameters().map((param) => param.name);
  }

  /**
   * Determine if a parameter descriptor exists.
   *
   * @param {string} name - The name of a descriptor to for.
   * @return {boolean} - True if a descriptor has been declared; otherwise false.
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
    const descriptors = [];

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

    const secondsAndNanos = this._clock.now().secondsAndNanoseconds;
    parameterEvent.stamp = {
      sec: secondsAndNanos.seconds,
      nanosec: secondsAndNanos.nanoseconds,
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
      parameterEvent[
        'changed_parameters'
      ] = changedParameters.map((parameter) => parameter.toParameterMessage());
    }
    if (deletedParameters.length > 0) {
      parameterEvent[
        'deleted_parameters'
      ] = deletedParameters.map((parameter) => parameter.toParameterMessage());
    }

    // publish ParameterEvent
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

  // returns on 1st error or result {successful, reason}
  _validateParameters(parameters = [], declareParameterMode = false) {
    for (const parameter of parameters) {
      // detect invalid parameter
      try {
        parameter.validate();
      } catch (e) {
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
        } catch (e) {
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

    Type.destoryRawROS(message);
  }

  _addActionClient(actionClient) {
    this._actionClients.push(actionClient);
    this.syncHandles();
  }

  _addActionServer(actionServer) {
    this._actionServers.push(actionServer);
    this.syncHandles();
  }
}

module.exports = Node;
