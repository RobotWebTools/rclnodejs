/* eslint-disable camelcase */

import { Clock } from 'rclnodejs';
import { Logging } from 'rclnodejs';
import { Parameter, ParameterDescriptor, ParameterType } from 'rclnodejs';
import { QoS } from 'rclnodejs';
import { rcl_interfaces } from 'rclnodejs';

declare module 'rclnodejs' {
  /**
   * Identifies type of ROS message such as msg or srv.
   */
  type TypeClass<T = TypeClassName> =
    | (() => any)
    | T // a string representing the message class, e.g. 'std_msgs/msg/String',
    | {
        // object representing a message class, e.g. {package: 'std_msgs', type: 'msg', name: 'String'}
        package: string;
        type: string;
        name: string;
      };

  /**
   * Configuration options when creating new Publishers, Subscribers,
   * Clients and Services.
   *
   * See {@link DEFAULT_OPTIONS}
   */
  interface Options<T = QoS | QoS.ProfileRef> {
    enableTypedArray?: boolean;
    qos?: T;
  }

  /**
   * Default options when creating a Node, Publisher, Subscription, Client or Service
   *
   * ```ts
   * {
   *   enableTypedArray: true,
   *   qos: QoS.profileDefault
   * }
   * ```
   */
  const DEFAULT_OPTIONS: Options;

  /**
   * A service response to a client request.
   *
   * @remarks
   * You can use {@link response.template | response.template} to get an empty result message.
   */
  class ServiceResponse {
    /**
     * Get an empty response message object.
     * The template will be a message of type: <pkg>.srv.<serviceTypeClass>_Response.
     * e.g., example_interface/srv/AddTwoInts_Response
     */
    readonly template: Message;

    /**
     * The service that this response object is attaching to.
     */
    readonly service: Service;

    /**
     * Send this response to client (the service caller)
     *
     * @param  response - Response message.
     *
     * @remarks
     * see {@link Response.template}
     */
    send(response: Message): void;
  }

  /**
   * A callback for receiving published messages.
   *
   * @param message - The published message.
   *
   * @remarks
   * See {@link Node#createSubscription | Node.createSubscription}
   * See {@link Node#createPublisher | Node.createPublisher}
   * See {@link Publisher}
   * See {@link Subscription}
   */
  type SubscriptionCallback =
    // * @param message - The published message
    (message: Message) => void;

  /**
   * Callback for receiving service requests from a client.
   *
   * @param request - The request sent to the service
   * @param response - The response to the client.
   *
   * @remarks
   * Use {@link Response.send | response.send()} to send response object to client
   *
   * See {@link Node.createService | Node.createService}
   * See {@link Client.sendRequest | Client.sendRequest}
   * See {@link Client}
   * See {@link Service}
   * See {@link Response.send | Response.send}
   */
  type ServiceRequestCallback = (
    request: Message,
    response: ServiceResponse
  ) => void;

  /**
   * Callback for receiving periodic interrupts from a Timer.
   *
   * @remarks
   * See {@link Node.createTimer | Node.createTimer}
   * See {@link Timer}
   */
  type TimerRequestCallback = () => void;

  /**
   * Callback indicating parameters are about to be declared or set.
   * The callback is provided a list of parameters and returns a SetParameterResult
   * to indicate approval or veto of the operation.
   *
   * @param parameters - The parameters to be declared or set
   * @returns A successful property value of true indicates approval of the operation;
   *  otherwise indicates a veto of the operation.
   *
   * @remarks
   * See {@link Node.addOnSetParametersCallback | Node.addOnSetParametersCallback}
   * See {@link Node.removeOnSetParametersCallback | Node.removeOnSetParametersCallback}
   */
  type SetParametersCallback = (
    parameters: Parameter[]
  ) => rcl_interfaces.msg.SetParametersResult;

  /**
   * Standard result of Node.getXXXNamesAndTypes() queries
   *
   * @example
   * ```
   * [
   *   { name: '/rosout', types: [ 'rcl_interfaces/msg/Log' ] },
   *   { name: '/scan',   types: [ 'sensor_msgs/msg/LaserScan' ] },
   *   { name: '/topic',  types: [ 'std_msgs/msg/String' ] }
   * ]
   * ```
   */
  type NamesAndTypesQueryResult = {
    name: string;
    types: Array<string>;
  };

  /**
   * Result of Node.getNodeNames() query
   *
   * @example
   * ```
   * [
   *   { name: 'gazebo',                namespace: '/' },
   *   { name: 'robot_state_publisher', namespace: '/' },
   *   { name: 'cam2image',             namespace: '/demo' }
   * ]
   * ```
   */
  type NodeNamesQueryResult = {
    name: string;
    namespace: string;
  };

  /**
   * Node is the primary entrypoint in a ROS system for communication.
   * It can be used to create ROS entities such as publishers, subscribers,
   * services, clients and timers.
   */
  class Node {
    /**
     * Get the name of the node.
     *
     * @returns The node name.
     */
    name(): string;

    /**
     * Get the namespace of the node.
     *
     * @returns The node namespace.
     */
    namespace(): string;

    /**
     * Get the nodes logger.
     *
     * @returns The logger for the node.
     */
    getLogger(): Logging;

    /**
     * Get the clock used by the node.
     *
     * @returns The nodes clock.
     */
    getClock(): Clock;

    /**
     * Get the current time using the node's clock.
     *
     * @returns The current time.
     */
    now(): Time;

    /**
     * Get the nodeOptions provided through the constructor.
     *
     * @returns The nodeOptions.
     */
    options(): NodeOptions;

    /**
     * Create a Timer.
     *
     * @param period - Elapsed time between interrupt events (milliseconds).
     * @param callback - Called on timeout interrupt.
     * @param clock - Optional clock to use for the timer.
     * @returns New instance of Timer.
     */
    createTimer(
      period: number,
      callback: TimerRequestCallback,
      clock?: Clock
    ): Timer;

    /**
     * Create a Rate.
     *
     * @param hz - The frequency of the rate timer; default is 1 hz.
     * @returns Promise resolving to new instance of Rate.
     */
    createRate(hz: number): Promise<Rate>;

    /**
     * Create a Publisher.
     *
     * @param typeClass - Type of message that will be published.
     * @param topic - Name of the topic the publisher will publish to.
     * @param options - Configuration options, see DEFAULT_OPTIONS
     * @returns New instance of Publisher.
     */
    createPublisher(
      typeClass: TypeClass,
      topic: string,
      options?: Options
    ): Publisher;

    /**
     * Create a Subscription.
     *
     * @param typeClass - Type of ROS messages the subscription will subscribe to
     * @param topic - Name of the topic the subcription will subscribe to.
     * @param options - Configuration options, see DEFAULT_OPTIONS
     * @param callback - Called when a new message is received.
     * @returns New instance of Subscription.
     */
    createSubscription(
      typeClass: TypeClass,
      topic: string,
      options: Options,
      callback: SubscriptionCallback
    ): Subscription;

    /**
     * Create a Client for making server requests.
     *
     * @param typeClass -  Service type.
     * @param serviceName - Service name.
     * @param options - The options argument used to parameterize the client.
     * @returns New instance of Client.
     */
    createClient(
      typeClass: TypeClass,
      serviceName: string,
      options?: Options
    ): Client;

    /**
     * Create a Service.
     *
     * @param typeClass - Service type
     * @param serviceName - Name of the service.
     * @param options - Configuration options
     * @param callback - Notified for receiving incoming requests.
     * @returns An instance of Service.
     */
    createService(
      typeClass: TypeClass,
      serviceName: string,
      options: Options,
      callback: ServiceRequestCallback
    ): Service;

    /**
     * Create a guard condition.
     *
     * @param callback - The callback to be called when the guard condition is triggered.
     * @returns An instance of GuardCondition.
     */
    createGuardCondition(callback: () => any): GuardCondition;

    /**
     * Destroy all entities allocated by this node, including
     * Timers, Publishers, Subscriptions, Clients, Services
     * and Timers.
     */
    destroy(): void;

    /**
     * Destroy a Publisher.
     *
     * @param publisher - Publisher to be destroyed.
     */
    destroyPublisher(publisher: Publisher): void;

    /**
     * Destroy a Subscription.
     *
     * @param subscription - Subscription to be destroyed.
     */
    destroySubscription(subscription: Subscription): void;

    /**
     * Destroy a Client.
     *
     * @param client - Client to be destroyed.
     */
    destroyClient(client: Client): void;

    /**
     * Destroy a Service.
     *
     * @param service - Service to be destroyed.
     */
    destroyService(service: Service): void;

    /**
     * Destroy a Timer.
     *
     * @param timer - Timer to be destroyed.
     */
    destroyTimer(timer: Timer): void;

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
     * @param parameter - Parameter to declare.
     * @param descriptor - Optional descriptor for parameter.
     * @param ignoreOveride - When true disregard any parameter-override that may be present.
     * @returns The newly declared parameter.
     */
    declareParameter(
      parameter: Parameter,
      descriptor?: ParameterDescriptor,
      ignoreOveride?: boolean
    ): Parameter;

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
     * @param parameters - The parameters to declare.
     * @param descriptors - Optional descriptors, a 1-1 correspondence with parameters.
     * @param ignoreOverrides - When true, parameter-overrides are
     *    not considered, i.e.,ignored.
     * @returns The declared parameters.
     */
    declareParameters(
      parameters: Array<Parameter>,
      descriptors?: Array<ParameterDescriptor>,
      ignoreOverrides?: boolean
    ): Array<Parameter>;

    /**
     * Undeclare a parameter.
     *
     * Readonly parameters can not be undeclared or updated.
     * @param name - Name of parameter to undeclare.
     */
    undeclareParameter(name: string): void;

    /**
     * Determine a parameter has been declared.
     *
     * @param name - Name of the parameter
     * @returns True if parameter exists; false otherwise.
     */
    hasParameter(name: string): boolean;

    /**
     * Get a declared parameter by name.
     *
     * If unable to locate a declared parameter then a
     * parameter with type == PARAMETER_NOT_SET is returned.
     *
     * @param name - The name of the parameter.
     * @returns The parameter.
     */

    getParameter(name: string): Parameter;

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
     * @param names - The names of the declared parameters
     *    to find or null indicating to return all declared parameters.
     * @returns The parameters.
     */

    getParameters(name?: Array<string>): Array<Parameter>;

    /**
     * Get the names of all declared parameters.
     *
     * @returns The declared parameter names or empty array if
     *    no parameters have been declared.
     */
    getParameterNames(): Array<string>;

    /**
     * Get the list of parameter-overrides found on the commandline and
     * in the NodeOptions.parameter_overrides property.
     *
     * @returns An array of Parameters
     */
    getParameterOverrides(): Array<Parameter>;

    /**
     * Determine if a parameter descriptor exists.
     *
     * @param name - The name of a descriptor to detect.
     * @returns True if a descriptor has been declared; otherwise false.
     */
    hasParameterDescriptor(name: string): boolean;

    /**
     * Get a declared parameter descriptor by name.
     *
     * If unable to locate a declared parameter descriptor then a
     * descriptor with type == PARAMETER_NOT_SET is returned.
     *
     * @param name - The name of the parameter descriptor to find.
     * @returns The parameter descriptor.
     */
    getParameterDescriptor(name: string): ParameterDescriptor;

    /**
     * Find a list of declared ParameterDescriptors.
     *
     * If no names are provided return all declared descriptors.
     *
     * If unable to locate a declared descriptor then a
     * descriptor with type == PARAMETER_NOT_SET is returned in
     * it's place.
     *
     * @param names - The names of the declared parameter
     *    descriptors to find or null indicating to return all declared descriptors.
     * @returns The parameter descriptors.
     */
    getParameterDescriptors(name?: string[]): ParameterDescriptor[];

    /**
     * Replace a declared parameter.
     *
     * The parameter being replaced must be a declared parameter who's descriptor
     * is not readOnly; otherwise an Error is thrown.
     *
     * @param parameter - The new parameter.
     * @returns The result of the operation.
     */
    setParameter(parameter: Parameter): rcl_interfaces.msg.SetParametersResult;

    /**
     * Replace a list of declared parameters.
     *
     * Declared parameters are replaced in the order they are provided and
     * a ParameterEvent is published for each individual parameter change.
     *
     * If an error occurs, the process is stopped and returned. Parameters
     * set before an error remain unchanged.
     *
     * @param parameters - The parameters to set.
     * @returns An array of SetParameterResult, one for each parameter that was set.
     */
    setParameters(
      parameters: Array<Parameter>
    ): Array<rcl_interfaces.msg.SetParametersResult>;

    /**
     * Repalce a list of declared parameters atomically.
     *
     * Declared parameters are replaced in the order they are provided.
     * A single ParameterEvent is published collectively for all changed
     * parameters.
     *
     * If an error occurs, the process stops immediately. All parameters updated to
     * the point of the error are reverted to their previous state.
     *
     * @param parameters - The parameters to set.
     * @returns Describes the result of setting 1 or more
     */
    setParametersAtomically(
      parameters: Array<Parameter>
    ): rcl_interfaces.msg.SetParametersResult;

    /**
     * Add a callback to the front of the list of callbacks invoked for parameter declaration
     * and setting. No checks are made for duplicate callbacks.
     *
     * @param callback - The callback to add.
     */
    addOnSetParametersCallback(callback: SetParametersCallback): void;

    /**
     * Remove a callback from the list of SetParameterCallbacks.
     * If the callback is not found the process is a nop.
     *
     * @param callback - The callback to be removed
     */
    removeOnSetParametersCallback(call: SetParametersCallback): void;

    /**
     * Get a remote node's published topics.
     *
     * @param remoteNodeName - Name of a remote node.
     * @param namespace - Name of the remote namespace.
     * @param  noDemangle - If true, topic names and types returned will not be demangled, default: false.
     * @returns An array of the names and types.
     *        [
     *          { name: '/rosout', types: [ 'rcl_interfaces/msg/Log' ] },
     *          { name: '/scan', types: [ 'sensor_msgs/msg/LaserScan' ] }
     *        ]
     */
    getPublisherNamesAndTypesByNode(
      remoteNodeName: string,
      namespace?: string,
      noDemangle?: boolean
    ): Array<NamesAndTypesQueryResult>;

    /**
     * Get a remote node's subscribed topics.
     *
     * @param nodeName - Name of the remote node.
     * @param namespace - Name of the remote namespace.
     * @param noDemangle - If true topic, names and types returned will not be demangled, default: false.
     * @returns An array of the names and types.
     *        [
     *          { name: '/topic', types: [ 'std_msgs/msg/String' ] }
     *        ]s
     */
    getSubscriptionNamesAndTypesByNode(
      remoteNodeName: string,
      namespace?: string,
      noDemangle?: boolean
    ): Array<NamesAndTypesQueryResult>;

    /**
     * Get a remote node's service topics.
     *
     * @param remoteNodeName - Name of the remote node.
     * @param namespace - Name of the remote namespace.
     * @returns An array of the names and types.
     *        [
     *          { name: '/rosout', types: [ 'rcl_interfaces/msg/Log' ] },
     *          ...
     *        ]
     */
    getServiceNamesAndTypesByNode(
      remoteNodeName: string,
      namespace?: string
    ): Array<NamesAndTypesQueryResult>;

    /**
     * Get this node's topics and corresponding types.
     *
     * @param noDemangle - If true. topic names and types returned will not be demangled, default: false.
     * @returns An array of the names and types.
     *        [
     *         { name: '/rosout', types: [ 'rcl_interfaces/msg/Log' ] },
     *         { name: '/scan', types: [ 'sensor_msgs/msg/LaserScan' ] },
     *         { name: '/topic', types: [ 'std_msgs/msg/String' ] }
     *        ]
     */
    getTopicNamesAndTypes(
      noDemangle?: boolean
    ): Array<NamesAndTypesQueryResult>;

    /**
     * Get this node's service names and corresponding types.
     *
     * @returns An array of the names and types.
     *        [
     *          { name: '/start_motor', types: [ 'rplidar_ros/srv/Control' ] },
     *          { name: '/stop_motor',  types: [ 'rplidar_ros/srv/Control' ] }
     *        ]
     */
    getServiceNamesAndTypes(): Array<NamesAndTypesQueryResult>;

    /**
     * Get the list of nodes discovered by the provided node.
     *
     * @returns An array of the node names.
     */
    getNodeNames(): string[];

    /**
     * Get the list of nodes and their namespaces discovered by the provided node.
     *
     * @returns An array of the node names and namespaces.
     *        [
     *          { name: 'gazebo',                namespace: '/' },
     *          { name: 'robot_state_publisher', namespace: '/' },
     *          { name: 'cam2image',             namespace: '/demo' }
     *        ]
     */
    getNodeNamesAndNamespaces(): Array<NodeNamesQueryResult>;

    /**
     * Return the number of publishers on a given topic.
     * @param topic - The name of the topic.
     * @returns Number of publishers on the given topic.
     */
    countPublishers(topic: string): number;

    /**
     * Return the number of subscribers on a given topic.
     * @param topic - The name of the topic.
     * @returns Number of subscribers on the given topic.
     */
    countSubscribers(topic: string): number;
  }

}
