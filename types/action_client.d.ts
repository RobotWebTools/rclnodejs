/* eslint-disable camelcase */
declare module 'rclnodejs' {
  type ActionGoal<T> = T extends ActionTypeClassName
    ? InstanceType<ActionsMap[T]['Goal']>
    : object;
  type ActionFeedback<T> = T extends ActionTypeClassName
    ? InstanceType<ActionsMap[T]['Feedback']>
    : object;
  type ActionResult<T> = T extends ActionTypeClassName
    ? InstanceType<ActionsMap[T]['Result']>
    : object;

  /**
   * Goal handle for working with Action Clients.
   */
  class ClientGoalHandle<T extends TypeClass<ActionTypeClassName>> {
    /**
     * Gets the goal Id.
     */
    get goalId(): unique_identifier_msgs.msg.UUID;

    /**
     * Gets the goal response timestamp.
     */
    get stamp(): builtin_interfaces.msg.Time;

    /**
     * Gets if the goal response was accepted.
     */
    get accepted(): boolean;

    /**
     * Gets the goal status.
     */
    get status(): string;

    /**
     * Send a cancel request for the goal.
     *
     * @returns The cancel response.
     */
    cancelGoal(): Promise<action_msgs.srv.CancelGoal_Response>;

    /**
     * Request the result for the goal.
     *
     * @returns The result response.
     */
    getResult(): Promise<ActionResult<T>>;
  }

  /**
   * ROS Middleware "quality of service" options for action clients.
   */
  interface ActionQoS {
    /**
     * Quality of service option for the goal service, default: QoS.profileServicesDefault.
     */
    goalServiceQosProfile?: QoS | QoS.ProfileRef;

    /**
     * Quality of service option for the result service, default: QoS.profileServicesDefault.
     */
    resultServiceQosProfile?: QoS | QoS.ProfileRef;

    /**
     * Quality of service option for the cancel service, default: QoS.profileServicesDefault.
     */
    cancelServiceQosProfile?: QoS | QoS.ProfileRef;

    /**
     * Quality of service option for the feedback subscription, default: new QoS(QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT, 10).
     */
    feedbackSubQosProfile?: QoS | QoS.ProfileRef;

    /**
     * Quality of service option for the status subscription, default: QoS.profileActionStatusDefault.
     */
    statusSubQosProfile?: QoS | QoS.ProfileRef;
  }

  /**
   * ROS Action client.
   */
  class ActionClient<T extends TypeClass<ActionTypeClassName>> {
    /**
     * Creates a new action client.
     *
     * @param node - The ROS node to add the action client to.
     * @param typeClass - Type of the action.
     * @param actionName - Name of the action.
     * @param options - The topic will use TypedArray if necessary, default: true.
     */
    constructor(
      node: Node,
      typeClass: T,
      actionName: string,
      options?: Options<ActionQoS>
    );

    /**
     * Send a goal and wait for the goal ACK asynchronously.
     *
     * Return a Promise object that is resolved with a ClientGoalHandle when receipt of the goal
     * is acknowledged by an action server, see client state transition https://index.ros.org/doc/ros2/Tutorials/Understanding-ROS2-Actions/
     *
     * @param goal - The goal request.
     * @param feedbackCallback - Callback function for feedback associated with the goal.
     * @param goalUuid - Universally unique identifier for the goal. If None, then a random UUID is generated.
     * @returns A Promise to a goal handle that resolves when the goal request has been accepted or rejected.
     */
    sendGoal(
      goal: ActionGoal<T>,
      feedbackCallback?: (feedbackMessage: ActionFeedback<T>) => void,
      goalUuid?: unique_identifier_msgs.msg.UUID
    ): Promise<ClientGoalHandle<T>>;

    /**
     * Check if there is an action server ready to process requests from this client.
     *
     * @returns True if an action server is ready; otherwise, false.
     */
    isActionServerAvailable(): boolean;

    /**
     * Wait until the action server is available or a timeout is reached. This
     * function polls for the server state so it may not return as soon as the
     * server is available.
     *
     * @param timeout The maximum amount of time to wait for, if timeout
     * is `undefined` or `< 0`, this will wait indefinitely.
     * @returns True if the service is available.
     */
    waitForServer(timeout?: number): Promise<boolean>;

    /**
     * Destroy the underlying action client handle.
     */
    destroy(): void;
  }
}
