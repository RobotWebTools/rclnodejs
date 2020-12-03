declare module 'rclnodejs' {
  namespace lifecycle {
    /**
     * A simple object representation of State.
     */
    interface SerializedState {
      id: number;
      label: string;
    }

    /**
     * The state of the lifecycle state-model.
     */
    class State {
      /**
       * Create a state.
       * @param id - The id value.
       * @param label - The label value.
       */
      constructor(id: number, label: string);

      /**
       * Create an object representation of state properties.
       * @return The object.
       */
      asMessage(): SerializedState;
    }

    /**
     * The intermediate state of the lifecycle state-model during a state
     * transition process.
     */
    class Transition extends State {}

    /**
     * Describes a state transition.
     */
    class TransitionDescription {
      /**
       * Create a transition description.
       *
       * @param transition - The transition
       * @param startState - The initial
       * @param goalState - The target state of a transition activity
       */
      constructor(transition: Transition, startState: State, goalState: State);

      /**
       * Create an object representation of the transitionDescripton properties.
       *
       * @returns The object representation.
       */
      asMessage(): {
        transition: SerializedState;

        // eslint-disable-next-line camelcase
        start_state: SerializedState;

        // eslint-disable-next-line camelcase
        goal_state: SerializedState;
      };
    }

    /**
     * The values returned by TransitionCallback.
     */
    const enum CallbackReturnCode {
      SUCCESS = 97, // rclnodejs.lifecycle_msgs.msg.TransitionConstructor.TRANSITION_CALLBACK_SUCCESS,
      FAILURE = 98, // rclnodejs.lifecycle_msgs.msg.TransitionConstructor.TRANSITION_CALLBACK_FAILURE,
      ERROR = 99, // rclnodejs.lifecycle_msgs.msg.TransitionConstructor.TRANSITION_CALLBACK_ERROR
    }

    /**
     * A CallbackReturnCode value-holder that is passed as an 'out' parameter
     * to a LifecycleNode's transition actions, e.g., configure().
     */
    class CallbackReturnValue {
      /**
       * Creates a new instance.
       *
       * @param value - Optional value, default = CallbackReturnCode.SUCCESS
       */
      constructor(value?: CallbackReturnCode);

      /**
       * Access the callbackReturnCode.
       * @returns {number} The CallbackReturnCode.
       */
      get value(): CallbackReturnCode;

      /**
       * Assign the callbackReturnCode.
       * @param value - The new value.
       */
      set value(value: CallbackReturnCode);

      /**
       * Access an optional error message when value is not SUCCESS.
       * @returns The errorMsg or undefined if no error message has ben assigned.
       */
      get errorMsg(): string;

      /**
       * Assign the error message.
       * @param msg - The error message.
       */
      set errorMsg(msg: string);

      /**
       * Overrides Object.valueOf() to return the 'value' property.
       * @returns The property value.
       */
      valueOf(): CallbackReturnCode;

      /**
       * A predicate to test if the value is SUCCESS.
       * @returns true if the value is SUCCESS; otherwise return false.
       */
      isSuccess(): boolean;

      /**
       * A predicate to test if the value is FAILURE.
       * @returns true if the value is FAILURE; otherwise return false.
       */
      isFailure(): boolean;

      /**
       * A predicate to test if the value is ERROR.
       * @returns true if the value is ERROR; otherwise return false.
       */
      isError(): boolean;

      /**
       * A predicate to test if an error message has been assigned.
       * @returns true if an error message has been assigned; otherwise return false.
       */
      hasErrorMsg(): boolean;
    }

    /**
     * The callback function invoked during a lifecycle state transition.
     *
     * @param prevState - The state transitioning from.
     * @return The return code.
     */
    type TransitionCallback = (prevState: State) => CallbackReturnCode;

    /**
     * A ROS2 managed Node that implements a well-defined life-cycle state-model using the
     * {@link https://github.com/ros2/rcl/tree/master/rcl_lifecycle|ros2 client library (rcl) lifecyle api}.
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
      /**
       * Access the current lifecycle state.
       * @returns The current state.
       */
      get currentState(): State;

      /**
       * Retrieve all states from the current state.
       * @returns {State[]} All states of the state-machine.
       */
      get availableStates(): State[];

      /**
       * Retrieve all transitions registered with the state-machine.
       *
       * @returns The TransitionDescriptions.
       */
      get transitions(): TransitionDescription[];

      /**
       * Retrieve all transitions available from the current state of the state-machine.
       *
       * @returns The available transitions.
       */
      get availableTransitions(): TransitionDescription[];

      /**
       * Register a callback function to be invoked during the configure() action.
       * @param cb - The callback function to invoke.
       */
      registerOnConfigure(cb: TransitionCallback): void;

      /**
       * Register a callback function to be invoked during the activate() action.
       * @param cb - The callback function to invoke.
       */
      registerOnActivate(cb: TransitionCallback): void;

      /**
       * Register a callback function to be invoked during the deactivate() action.
       * @param  cb - The callback function to invoke.
       */
      registerOnDectivate(cb: TransitionCallback): void;

      /**
       * Register a callback function to be invoked during the cleanup() action.
       * @param cb - The callback function to invoke.
       */
      registerOnCleanup(cb: TransitionCallback): void;

      /**
       * Register a callback function to be invoked during the shutdown() action.
       * @param cb - The callback function to invoke.
       */
      registerOnShutdown(cb: TransitionCallback): void;

      /**
       * Register a callback function to be invoked when an error occurs during a
       * state transition.
       * @param cb - The callback function to invoke.
       */
      registerOnError(cb: TransitionCallback): void;

      /**
       * Initiate a transition from the UNCONFIGURED state to the INACTIVE state.
       * If an onConfigure callback has been registered, it will be invoked.
       *
       * @param callbackReturnValue - value holder for the CallbackReturnCode returned from the callback.
       * @returns The new state, should be INACTIVE.
       */
      configure(callbackReturnValue?: CallbackReturnValue): State;

      /**
       * Initiate a transition from the INACTIVE state to the ACTIVE state.
       * If an onActivate callback has been registered it will be invoked.
       *
       * @param callbackReturnValue - value holder for the CallbackReturnCode returned from the callback.
       * @returns The new state, should be ACTIVE.
       */
      activate(callbackReturnValue?: CallbackReturnValue): State;

      /**
       * Initiate a transition from the ACTIVE state to the INACTIVE state.
       * If an onDeactivate callback has been registered it will be invoked.
       *
       * @param callbackReturnValue - value holder for the CallbackReturnCode returned from the callback.
       * @returns The new state, should be INACTIVE.
       */
      deactivate(callbackReturnValue?: CallbackReturnValue): State;

      /**
       * Initiate a transition from the INACTIVE state to the UNCONFIGURED state.
       * If an onCleanup callback has been registered it will be invoked.
       *
       * @param callbackReturnValue - value holder for the CallbackReturnCode returned from the callback.
       * @returns The new state, should be INACTIVE.
       */
      cleanup(callbackReturnValue?: CallbackReturnValue): State;

      /**
       * Initiate a transition from the ACTIVE state to the FINALIZED state.
       * If an onConfigure callback has been registered it will be invoked.
       *
       * @param callbackReturnValue - value holder for the CallbackReturnCode returned from the callback.
       * @returns  The new state, should be FINALIZED.
       */
      shutdown(callbackReturnValue?: CallbackReturnValue): State;

      /**
       * Create a LifecyclePublisher.
       *
       * @param typeClass - Type of message that will be published.
       * @param topic - Name of the topic the publisher will publish to.
       * @param options - Configuration options, see DEFAULT_OPTIONS
       * @returns New instance of LifecyclePublisher.
       */
      createLifecyclePublisher<T extends TypeClass<MessageTypeClassName>>(
        typeClass: T,
        topic: string,
        options?: Options
      ): LifecyclePublisher<T>;
    }
  } // lifecycle namespace
} // rclnodejs namespace
