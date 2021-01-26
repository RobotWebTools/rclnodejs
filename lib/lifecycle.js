// Copyright (c) 2020 Wayne Parrott. All rights reserved.
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
const LifecyclePublisher = require('./lifecycle_publisher.js');
const loader = require('./interface_loader.js');
const Service = require('./service.js');
const Node = require('./node.js');

const SHUTDOWN_TRANSITION_LABEL = rclnodejs.getLifecycleShutdownTransitionLabel();

// An instance of State message constructor used for accessing State
// state machine constants. This interface is lazy initialized at runtime.
let StateInterface;

// An instance of Transition message constructor used for accessing Transition
// state machine constants. This interface is lazy initialized at runtime.
let TransitionInterface;

function getStateInterface() {
  if (!StateInterface) {
    StateInterface = loader.loadInterface('lifecycle_msgs/msg/State');
  }
  return StateInterface;
}

function getTransitionInterface() {
  if (!TransitionInterface) {
    TransitionInterface = loader.loadInterface('lifecycle_msgs/msg/Transition');
  }
  return TransitionInterface;
}

/**
 * @typedef SerializedState
 * @type {object}
 * @property {number} id - code identifying the type of this state.
 * @property {string} label - readable name of this state.
 */

/**
 * The state of the lifecycle state model.
 */
class State {
  /**
   * Create a state.
   * @param {number} id - The id value.
   * @param {string} label - The label value.
   */
  constructor(id, label) {
    this.id = id;
    this.label = label;
  }

  /**
   * Create a State from a SerializedState
   * @param {SerializedState} aSerializedState - The state object.
   * @returns {State} The State converted from a SerializdState
   * @private
   */
  static fromSerializedState(aSerializedState) {
    return new State(aSerializedState.id, aSerializedState.label);
  }
}

/**
 * The intermediate state of the lifecycle state model during a state
 * transition.
 */
class Transition extends State {}

/**
 * Describes a state transition.
 */
class TransitionDescription {
  /**
   * Create a transition description.
   *
   * @param {Transition} transition - The transition
   * @param {State} startState - The initial
   * @param {State} goalState - The target state of a transition activity
   */
  constructor(transition, startState, goalState) {
    this.transition = transition;
    this.startState = startState;
    this.goalState = goalState;
  }
}

/**
 * The values returned by TransitionCallback.
 * @readonly
 * @enum {number}
 */
const CallbackReturnCode = {
  get SUCCESS() {
    return getTransitionInterface().TRANSITION_CALLBACK_SUCCESS;
  },
  get FAILURE() {
    return getTransitionInterface().TRANSITION_CALLBACK_FAILURE;
  },
  get ERROR() {
    return getTransitionInterface().TRANSITION_CALLBACK_ERROR;
  },
};
Object.freeze(CallbackReturnCode);

/**
 * A ValueHolder for a CallbackReturnCode.
 */
class CallbackReturnValue {
  /**
   * Creates a new instance.
   *
   * @param {number} [value=CallbackReturnCode.SUCCESS] - The value property.
   */
  constructor(value = CallbackReturnCode.SUCCESS) {
    this._value = value;
    this._errorMsg = null;
  }

  /**
   * Access the callbackReturnCode.
   * @returns {number} The CallbackReturnCode.
   */
  get value() {
    return this._value;
  }

  set value(value) {
    this._value = value;
  }

  /**
   * Access an optional error message when value is not SUCCESS.
   */
  get errorMsg() {
    return this._errorMsg;
  }

  /**
   * Assign the error message.
   * @param {string} msg - The error message.
   * @returns {unknown} void.
   */
  set errorMsg(msg) {
    this._errorMsg = msg;
  }

  /**
   * Overrides Object.valueOf() to return the 'value' property.
   * @returns {number} The property value.
   */
  valueOf() {
    return this.value;
  }

  /**
   * A predicate to test if the value is SUCCESS.
   * @returns {boolean} Return true if the value is SUCCESS; otherwise return false.
   */
  isSuccess() {
    return this.value === CallbackReturnCode.SUCCESS;
  }

  /**
   * A predicate to test if the value is FAILURE.
   * @returns {boolean} Return true if the value is FAILURE; otherwise return false.
   */
  isFailure() {
    return this.value === CallbackReturnCode.FAILURE;
  }

  /**
   * A predicate to test if the value is ERROR.
   * @returns {boolean} Return true if the value is ERROR; otherwise return false.
   */
  isError() {
    return this.value === CallbackReturnCode.ERROR;
  }

  /**
   * A predicate to test if an error message has been assigned.
   * @returns {boolean} Return true if an error message has been assigned; otherwise return false.
   */
  hasErrorMsg() {
    return !this.isSuccess() && this._errorMsg;
  }
}

/**
 * This callback is invoked when LifecycleNode is transitioning to a new state.
 * @callback TransitionCallback
 * @param {State} previousState - The previous node lifecycle state.
 * @return {CallbackReturnCode} - The result of the callback.
 *
 * @see [LifecycleNode.registerOnConfigure]{@link LifecycleNode#registerOnConfigure}
 * @see [LifecycleNode.registerOnCleanup]{@link LifecycleNode#registerOnCleanup}
 * @see [LifecycleNode.registerOnActivate]{@link LifecycleNode#registerOnActivate}
 * @see [LifecycleNode.registerOnDeactivate]{@link LifecycleNode#registerOnDeactivate}
 * @see [LifecycleNode.registerOnShutdown]{@link LifecycleNode#registerOnShutdown}
 * @see [LifecycleNode.registerOnError]{@link LifecycleNode#registerOnError}
 */

/**
 * A ROS2 managed Node that implements a well-defined life-cycle state model using the
 * {@link https://github.com/ros2/rcl/tree/master/rcl_lifecycle|ros2 client library (rcl) lifecyle api}.
 * @extends Node
 *
 * This class implments the ROS2 life-cycle state-machine defined by the
 * {@link https://github.com/ros2/rclcpp/tree/master/rclcpp_lifecycle}|ROS2 Managed Nodes Design}
 * and parallels the {@link https://github.com/ros2/rclcpp/tree/master/rclcpp_lifecycle|rclcpp lifecycle node }
 * implementation.
 *
 * The design consists of four primary lifecycle states:
 *   UNCONFIGURED
 *   INACTIVE
 *   ACTIVE
 *   FINALIZED.
 *
 * Transitioning between states is accomplished using an action api:
 *   configure()
 *   activate()
 *   deactivate(),
 *   cleanup()
 *   shutdown()
 *
 * During a state transition, the state-machine is in one of the
 * intermediate transitioning states:
 *   CONFIGURING
 *   ACTIVATING
 *   DEACTIVATING
 *   CLEANINGUP
 *   SHUTTING_DOWN
 *   ERROR_PROCESSING
 *
 * Messaging:
 * State changes are published on the '<node_name>/transition_event' topic.
 * Lifecycle service interfaces are also implemented.
 *
 * You can introduce your own state specific behaviors in the form of a
 * {@link TransitionCallback} functions that you register using:
 *   registerOnConfigure(cb)
 *   registerOnActivate(cb)
 *   registerOnDeactivate(cb)
 *   registerOnCleanup(cb)
 *   registerOnShutdown(cb)
 *   registerOnError(cb)
 */

class LifecycleNode extends Node {
  constructor(
    nodeName,
    namespace = '',
    context = Context.defaultContext(),
    options = NodeOptions.defaultOptions
  ) {
    super(nodeName, namespace, context, options);
    this.init();
  }

  init() {
    // initialize native handle to rcl_lifecycle_state_machine
    this._stateMachineHandle = rclnodejs.createLifecycleStateMachine(
      this.handle
    );

    // initialize Map<transitionId,TransitionCallback>
    this._callbackMap = new Map();

    // Setup and register the 4 native rcl lifecycle services thar are
    // part of _stateMachineHandle.
    let srvHandleObj = rclnodejs.getLifecycleSrvNameAndHandle(
      this._stateMachineHandle,
      'srv_get_state'
    );
    let service = new Service(
      srvHandleObj.handle,
      srvHandleObj.name,
      loader.loadInterface('lifecycle_msgs/srv/GetState'),
      this._validateOptions(undefined),
      (request, response) => this._onGetState(request, response)
    );
    this._services.push(service);

    srvHandleObj = rclnodejs.getLifecycleSrvNameAndHandle(
      this._stateMachineHandle,
      'srv_get_available_states'
    );
    service = new Service(
      srvHandleObj.handle,
      srvHandleObj.name,
      loader.loadInterface('lifecycle_msgs/srv/GetAvailableStates'),
      this._validateOptions(undefined),
      (request, response) => this._onGetAvailableStates(request, response)
    );
    this._services.push(service);

    srvHandleObj = rclnodejs.getLifecycleSrvNameAndHandle(
      this._stateMachineHandle,
      'srv_get_available_transitions'
    );
    service = new Service(
      srvHandleObj.handle,
      srvHandleObj.name,
      loader.loadInterface('lifecycle_msgs/srv/GetAvailableTransitions'),
      this._validateOptions(undefined),
      (request, response) => this._onGetAvailableTransitions(request, response)
    );
    this._services.push(service);

    srvHandleObj = rclnodejs.getLifecycleSrvNameAndHandle(
      this._stateMachineHandle,
      'srv_change_state'
    );
    service = new Service(
      srvHandleObj.handle,
      srvHandleObj.name,
      loader.loadInterface('lifecycle_msgs/srv/ChangeState'),
      this._validateOptions(undefined),
      (request, response) => this._onChangeState(request, response)
    );
    this._services.push(service);

    this.syncHandles();
  }

  /**
   * Create a LifecyclePublisher.
   * @param {function|string|object} typeClass - The ROS message class,
        OR a string representing the message class, e.g. 'std_msgs/msg/String',
        OR an object representing the message class, e.g. {package: 'std_msgs', type: 'msg', name: 'String'}
   * @param {string} topic - The name of the topic.
   * @param {object} options - The options argument used to parameterize the publisher.
   * @param {boolean} options.enableTypedArray - The topic will use TypedArray if necessary, default: true.
   * @param {QoS} options.qos - ROS Middleware "quality of service" settings for the publisher, default: QoS.profileDefault.
   * @return {LifecyclePublisher} - An instance of LifecyclePublisher.
   */
  createLifecyclePublisher(typeClass, topic, options) {
    return this._createPublisher(typeClass, topic, options, LifecyclePublisher);
  }

  /**
   * Access the current lifecycle state.
   * @returns {State} The current state.
   */
  get currentState() {
    let currentStateObj = rclnodejs.getCurrentLifecycleState(
      this._stateMachineHandle
    );
    return State.fromSerializedState(currentStateObj);
  }

  /**
   * Retrieve all registered states of the state-machine.
   * @returns {State[]} The states.
   */
  get availableStates() {
    let stateObjs = rclnodejs.getLifecycleStates(this._stateMachineHandle);
    let states = stateObjs.map(
      (stateObj) => new State(stateObj.id, stateObj.label)
    );
    return states;
  }

  /**
   * Retrieve all registered transitions of the state-machine.
   *
   * @returns {TransitionDescription[]} The registered TransitionDescriptions.
   */
  get transitions() {
    let transitionObjs = rclnodejs.getLifecycleTransitions(
      this._stateMachineHandle
    );
    let transitions = transitionObjs.map((transitionDescObj) => {
      let transition = new Transition(
        transitionDescObj.transition.id,
        transitionDescObj.transition.label
      );
      let startState = new State(
        transitionDescObj.start_state.id,
        transitionDescObj.start_state.label
      );
      let goalState = new State(
        transitionDescObj.goal_state.id,
        transitionDescObj.goal_state.label
      );
      return new TransitionDescription(transition, startState, goalState);
    });
    return transitions;
  }

  /**
   * Retrieve the valid transitions available from the current state of the
   * state-machine.
   *
   * @returns {TransitionDescription[]} The available TransitionDescriptions.
   */
  get availableTransitions() {
    let transitionObjs = rclnodejs.getAvailableLifecycleTransitions(
      this._stateMachineHandle
    );
    let transitions = transitionObjs.map((transitionDescObj) => {
      let transition = new Transition(
        transitionDescObj.transition.id,
        transitionDescObj.transition.label
      );
      let startState = new State(
        transitionDescObj.start_state.id,
        transitionDescObj.start_state.label
      );
      let goalState = new State(
        transitionDescObj.goal_state.id,
        transitionDescObj.goal_state.label
      );
      return new TransitionDescription(transition, startState, goalState);
    });
    return transitions;
  }

  /**
   * Register a callback function to be invoked during the configure() action.
   * @param {TransitionCallback} cb - The callback function to invoke.
   * @returns {unknown} void.
   */
  registerOnConfigure(cb) {
    this._callbackMap.set(getStateInterface().TRANSITION_STATE_CONFIGURING, cb);
  }

  /**
   * Register a callback function to be invoked during the activate() action.
   * @param {TransitionCallback} cb - The callback function to invoke.
   * @returns {unknown} void.
   */
  registerOnActivate(cb) {
    this._callbackMap.set(getStateInterface().TRANSITION_STATE_ACTIVATING, cb);
  }

  /**
   * Register a callback function to be invoked during the deactivate() action.
   * @param {TransitionCallback} cb - The callback function to invoke.
   * @returns {unknown} void.
   */
  registerOnDeactivate(cb) {
    this._callbackMap.set(
      getStateInterface().TRANSITION_STATE_DEACTIVATING,
      cb
    );
  }

  /**
   * Register a callback function to be invoked during the cleanup() action.
   * @param {TransitionCallback} cb - The callback function to invoke.
   * @returns {unknown} void.
   */
  registerOnCleanup(cb) {
    this._callbackMap.set(getStateInterface().TRANSITION_STATE_CLEANINGUP, cb);
  }

  /**
   * Register a callback function to be invoked during the shutdown() action.
   * @param {TransitionCallback} cb - The callback function to invoke.
   * @returns {unknown} void
   */
  registerOnShutdown(cb) {
    this._callbackMap.set(
      getStateInterface().TRANSITION_STATE_SHUTTINGDOWN,
      cb
    );
  }

  /**
   * Register a callback function to be invoked when an error occurs during a
   * state transition.
   * @param {TransitionCallback} cb - The callback function to invoke.
   * @returns {unknown} void.
   */
  registerOnError(cb) {
    this._callbackMap.set(
      getStateInterface().TRANSITION_STATE_ERRORPROCESSING,
      cb
    );
  }

  /**
   * Initiate a transition from the UNCONFIGURED state to the INACTIVE state.
   * If an onConfigure callback has been registered it will be invoked.
   *
   * @param {CallbackReturnValue?} callbackReturnValue - value holder for the CallbackReturnCode returned from the callback.
   * @returns {State} The new state, should be INACTIVE.
   * @throws {Error} If transition is invalid for the current state.
   */
  configure(callbackReturnValue) {
    return this._changeState(
      getTransitionInterface().TRANSITION_CONFIGURE,
      callbackReturnValue
    );
  }

  /**
   * Initiate a transition from the INACTIVE state to the ACTIVE state.
   * If an onActivate callback has been registered it will be invoked.
   *
   * @param {CallbackReturnValue?} callbackReturnValue - value holder for the CallbackReturnCode returned from the callback.
   * @returns {State} The new state, should be ACTIVE.
   * @throws {Error} If transition is invalid for the current state.
   */
  activate(callbackReturnValue) {
    return this._changeState(
      getTransitionInterface().TRANSITION_ACTIVATE,
      callbackReturnValue
    );
  }

  /**
   * Initiate a transition from the ACTIVE state to the INACTIVE state.
   * If an onDeactivate callback has been registered it will be invoked.
   *
   * @param {CallbackReturnValue?} callbackReturnValue - value holder for the CallbackReturnCode returned from the callback.
   * @returns {State} The new state, should be INACTIVE.
   * @throws {Error} If transition is invalid for the current state.
   */
  deactivate(callbackReturnValue) {
    return this._changeState(
      getTransitionInterface().TRANSITION_DEACTIVATE,
      callbackReturnValue
    );
  }

  /**
   * Initiate a transition from the INACTIVE state to the UNCONFIGURED state.
   * If an onCleanup callback has been registered it will be invoked.
   *
   * @param {CallbackReturnValue?} callbackReturnValue - value holder for the CallbackReturnCode returned from the callback.
   * @returns {State} The new state, should be INACTIVE.
   * @throws {Error} If transition is invalid for the current state.
   */
  cleanup(callbackReturnValue) {
    return this._changeState(
      getTransitionInterface().TRANSITION_CLEANUP,
      callbackReturnValue
    );
  }

  /**
   * Initiate a transition to the FINALIZED state from any of the following
   * states: UNCONFIGURED, INACTIVE or ACTIVE state. If an onConfigure
   * callback has been registered it will be invoked.
   *
   * @param {CallbackReturnValue?} callbackReturnValue - value holder for the CallbackReturnCode returned from the callback.
   * @returns {State} The new state, should be FINALIZED.
   * @throws {Error} If transition is invalid for the current state.
   */
  shutdown(callbackReturnValue) {
    let state = this.currentState;

    return this._changeState(SHUTDOWN_TRANSITION_LABEL, callbackReturnValue);
  }

  /**
   * The GetState service handler.
   * @param {Object} request - The GetState service request.
   * @param {Object} response - The GetState service response.
   * @returns {unknown} void.
   * @private
   */
  _onGetState(request, response) {
    let result = response.template;

    // eslint-disable-next-line camelcase
    result.current_state = this.currentState;

    response.send(result);
  }

  /**
   * The GetAvailableStates service handler.
   * @param {Object} request - The GetAvailableStates service request.
   * @param {Object} response - The GetAvailableStates service response.
   * @returns {unknown} void.
   * @private
   */
  _onGetAvailableStates(request, response) {
    let result = response.template;

    // eslint-disable-next-line camelcase
    result.available_states = this.availableStates;

    response.send(result);
  }

  /**
   * The GetAvailableTransitions service handler.
   * @param {Object} request - The GetAvailableTransitions service request
   * @param {Object} response - The GetAvailableTranactions service response.
   * @returns {unknown} void.
   */
  _onGetAvailableTransitions(request, response) {
    let result = response.template;

    // eslint-disable-next-line camelcase
    result.available_transitions = this.availableTransitions;

    response.send(result);
  }

  /**
   * The ChangeState service handler.
   * @param {Object} request - The ChangeState service request.
   * @param {Object} response - The ChangeState service response
   * @returns {unknown} void.
   * @private
   */
  _onChangeState(request, response) {
    let result = response.template;

    let transitionId = request.transition.id;
    if (request.transition.label) {
      let transitionObj = rclnodejs.getLifecycleTransitionByLabel(
        this._stateMachineHandle,
        request.transition.label
      );
      if (transitionObj.id) {
        transitionId = transitionObj.id;
      } else {
        result.success = false;
        response.send(result);
        return;
      }
    }

    let callbackReturnValue = new CallbackReturnValue();
    this._changeState(transitionId, callbackReturnValue);

    result.success = callbackReturnValue.isSuccess();
    response.send(result);
  }

  /**
   * Transition to a new lifecycle state.
   * @param {number|string} transitionIdOrLabel - The id or label of the target transition.
   * @param {CallbackReturnValue} callbackReturnValue - An out parameter that holds the CallbackReturnCode.
   * @returns {State} The new state.
   * @throws {Error} If transition is invalid for the current state.
   * @private
   */
  _changeState(transitionIdOrLabel, callbackReturnValue) {
    let initialState = this.currentState;
    let newStateObj =
      typeof transitionIdOrLabel === 'number'
        ? rclnodejs.triggerLifecycleTransitionById(
            this._stateMachineHandle,
            transitionIdOrLabel
          )
        : rclnodejs.triggerLifecycleTransitionByLabel(
            this._stateMachineHandle,
            transitionIdOrLabel
          );

    if (!newStateObj) {
      throw new Error(
        `No transition available from state ${transitionIdOrLabel}.`
      );
    }

    let newState = State.fromSerializedState(newStateObj);

    let cbResult = this._executeCallback(newState, initialState);
    if (callbackReturnValue) callbackReturnValue.value = cbResult;

    let transitioningLabel = this._transitionId2Label(cbResult);
    newState = State.fromSerializedState(
      rclnodejs.triggerLifecycleTransitionByLabel(
        this._stateMachineHandle,
        transitioningLabel
      )
    );

    if (cbResult == CallbackReturnCode.ERROR) {
      cbResult = this._executeCallback(this.currentState, initialState);
      if (callbackReturnValue) callbackReturnValue.value = cbResult;

      transitioningLabel = this._transitionId2Label(cbResult);
      newState = State.fromSerializedState(
        rclnodejs.triggerLifecycleTransitionByLabel(
          this._stateMachineHandle,
          transitioningLabel
        )
      );
    }

    return newState;
  }

  /**
   * Execute the callback function registered with a transition action,
   * e.g. registerOnConfigure(cb).
   * @param {State} state - The state to which the callback is
   * @param {State} prevState - The start state of the transition.
   * @returns {CallbackReturnCode} The callback return code.
   * @private
   */
  _executeCallback(state, prevState) {
    let result = CallbackReturnCode.SUCCESS;
    let callback = this._callbackMap.get(state.id);

    if (callback) {
      try {
        result = callback(prevState);
      } catch (err) {
        console.log('CB exception occured: ', err);
        result = CallbackReturnCode.ERROR;
      }
    }

    return result;
  }

  /**
   * Find the label for the transition with id == transitionId.
   * @param {number} transitionId - A transition id.
   * @returns {string} The label of the transition with id.
   * @private
   */
  _transitionId2Label(transitionId) {
    return rclnodejs.getLifecycleTransitionIdToLabel(transitionId);
  }
}

const Lifecycle = {
  CallbackReturnCode,
  CallbackReturnValue,
  LifecycleNode,
  State,
  Transition,
  TransitionDescription,
};

module.exports = Lifecycle;
