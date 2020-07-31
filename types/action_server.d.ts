/* eslint-disable camelcase */
declare module 'rclnodejs' {
  /**
   * Goal handle for working with Action Servers.
   */
  class ServerGoalHandle<T extends TypeClass<ActionTypeClassName>> {
    /**
     * Gets the goal request.
     */
    get request(): ActionGoal<T>;

    /**
     * Gets the goal Id.
     */
    get goalId(): unique_identifier_msgs.msg.UUID;

    /**
     * Gets if the goal handle is active.
     */
    get isActive(): boolean;

    /**
     * Gets if cancellation was requested.
     */
    get isCancelRequested(): boolean;

    /**
     * Gets the status of the goal.
     */
    get status(): string;

    /**
     * Updates the goal handle with the execute status and begins exection.
     *
     * @param callback - An optional callback to use instead of the one provided to the action server.
     */
    execute(callback?: ExecuteCallback<T>): void;

    /**
     * Sends feedback back to the client.
     *
     * @param feedback - The feedback to send back.
     */
    publishFeedback(feedback: ActionFeedback<T>): void;

    /**
     * Updates the goal handle with the succeed status.
     */
    succeed(): void;

    /**
     * Updates the goal handle with the abort status.
     */
    abort(): void;

    /**
     * Updates the goal handle with the canceled status.
     */
    canceled(): void;
  }

  /**
   * Possible goal responses.
   */
  enum GoalResponse {
    REJECT = 1,
    ACCEPT = 2,
  }

  /**
   * Possible cancel responses.
   */
  enum CancelResponse {
    REJECT = 1,
    ACCEPT = 2,
  }

  type ExecuteCallback<T extends TypeClass<ActionTypeClassName>> = (
    goalHandle: ServerGoalHandle<T>
  ) => ActionResult<T>;
  type GoalCallback = () => GoalResponse;
  type HandleAcceptedCallback<T extends TypeClass<ActionTypeClassName>> = (
    goalHandle: ServerGoalHandle<T>
  ) => void;
  type CancelCallback = () => CancelResponse;

  interface ActionServerOptions extends Options<ActionQoS> {
    /**
     * How long in seconds a result is kept by the server after a goal reaches a terminal state in seconds, default: 900.
     */
    resultTimeout?: number;
  }

  /**
   * ROS Action server.
   */
  class ActionServer<T extends TypeClass<ActionTypeClassName>> {
    /**
     * Creates a new action server.
     *
     * @param node - The ROS node to add the action server to.
     * @param typeClass - Type of the action.
     * @param actionName - Name of the action. Used as part of the underlying topic and service names.
     * @param executeCallback - Callback function for processing accepted goals.
     * @param goalCallback - Callback function for handling new goal requests.
     * @param handleAcceptedCallback - Callback function for handling newly accepted goals.
     * @param cancelCallback - Callback function for handling cancel requests.
     * @param options - Action server options.
     */
    constructor(
      node: Node,
      typeClass: T,
      actionName: string,
      executeCallback: ExecuteCallback<T>,
      goalCallback?: GoalCallback,
      handleAcceptedCallback?: HandleAcceptedCallback<T>,
      cancelCallback?: CancelCallback,
      options?: ActionServerOptions
    );

    /**
     * Register a callback for handling newly accepted goals.
     *
     * The provided function is called whenever a new goal has been accepted by this action server.
     * The function should expect an instance of {@link ServerGoalHandle} as an argument,
     * which represents a handle to the goal that was accepted.
     * The goal handle can be used to interact with the goal, e.g. publish feedback,
     * update the status, or execute a deferred goal.
     *
     * There can only be one handle accepted callback per {@link ActionServer},
     * therefore calling this function will replace any previously registered callback.
     *
     * @param handleAcceptedCallback - Callback function, if not provided, then unregisters any previously registered callback.
     */
    registerHandleAcceptedCallback(
      handleAcceptedCallback?: HandleAcceptedCallback<T>
    ): void;

    /**
     * Register a callback for handling new goal requests.
     *
     * The purpose of the goal callback is to decide if a new goal should be accepted or rejected.
     * The callback should take the goal request message as a parameter and must return a {@link GoalResponse} value.
     *
     * @param goalCallback - Callback function, if not provided, then unregisters any previously registered callback.
     */
    registerGoalCallback(goalCallback?: GoalCallback): void;

    /**
     * Register a callback for handling cancel requests.
     *
     * The purpose of the cancel callback is to decide if a request to cancel an on-going
     * (or queued) goal should be accepted or rejected.
     * The callback should take one parameter containing the cancel request and must return a
     * {@link CancelResponse} value.
     *
     * There can only be one cancel callback per {@link ActionServer}, therefore calling this
     * function will replace any previously registered callback.
     * @param cancelCallback - Callback function, if not provided, then unregisters any previously registered callback.
     */
    registerCancelCallback(cancelCallback?: CancelCallback): void;

    /**
     * Register a callback for executing action goals.
     *
     * The purpose of the execute callback is to execute the action goal and return a result when finished.
     * The callback should take one parameter containing goal request and must return a
     * result instance (i.e. `action_type.Result`).
     *
     * There can only be one execute callback per {@link ActionServer}, therefore calling this
     * function will replace any previously registered callback.
     *
     * @param executeCallback - Callback function.
     */
    registerExecuteCallback(executeCallback: ExecuteCallback<T>): void;

    /**
     * Destroy the action server and all goals.
     */
    destroy(): void;
  }
}
