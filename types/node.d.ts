import { QoS } from 'rclnodejs';

declare module 'rclnodejs' {

	/**
	 * Identifies type of ROS message such as msg or srv.
	 */
  type TypeClass =
    (() => any) |
    TypeClassName |   // a string representing the message class, e.g. 'std_msgs/msg/String',
    {          // object representing a message class, e.g. {package: 'std_msgs', type: 'msg', name: 'String'}
      package: string;
      type: string;
      name: string
    };


	/**
	 * Configuration options when creating new Publishers, Subscribers,
	 * Clients and Services.
	 *
	 * See {@link DEFAULT_OPTIONS}
	 */
  type Options = {
    enableTypedArray?: boolean;
    qos?: QoS | QoS.ProfileRef;
  }


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
  type SubscriptionCallback = (
    // * @param message - The published message
    (message: Message) => void
  );


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
    (request: Message, response: ServiceResponse) => void
  );


	/**
	 * Callback for receiving periodic interrupts from a Timer.
	 *
	 * @remarks
	 * See {@link Node.createTimer | Node.createTimer}
	 * See {@link Timer}
	 */
  type TimerRequestCallback = (
    () => void
  );

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
  }


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
		 * Create a Timer.
		 *
		 * @param period - Elapsed time between interrupt events (milliseconds).
		 * @param callback - Called on timeout interrupt.
		 * @param context - Context, default is Context.defaultContext().
		 * @returns New instance of Timer.
		 */
    createTimer(period: number, callback: TimerRequestCallback, context?: Context): Timer;

    /**
		 * Create a Publisher.
		 *
		 * @param typeClass - Type of message that will be published.
		 * @param topic - Name of the topic the publisher will publish to.
		 * @param options - Configuration options, see DEFAULT_OPTIONS
		 * @returns New instance of Publisher.
		 */
    createPublisher(typeClass: TypeClass, topic: string, options?: Options): Publisher;

    /**
		 * Create a Subscription.
		 *
		 * @param typeClass - Type of ROS messages the subscription will subscribe to
		 * @param topic - Name of the topic the subcription will subscribe to.
		 * @param options - Configuration options, see DEFAULT_OPTIONS
		 * @param callback - Called when a new message is received.
		 * @returns New instance of Subscription.
		 */
    createSubscription(typeClass: TypeClass, topic: string,
      options: Options, callback: SubscriptionCallback): Subscription;

    /**
		 * Create a Client for making server requests.
		 *
		 * @param typeClass -  Service type.
		 * @param serviceName - Service name.
		 * @param options - The options argument used to parameterize the client.
		 * @returns New instance of Client.
		 */
    createClient(typeClass: TypeClass, serviceName: string, options?: Options): Client;

    /**
		 * Create a Service.
		 *
		 * @param typeClass - Service type
		 * @param serviceName - Name of the service.
		 * @param options - Configuration options
		 * @param callback - Notified for receiving incoming requests.
		 * @returns An instance of Service.
		 */
    createService(typeClass: TypeClass, serviceName: string,
      options: Options, callback: ServiceRequestCallback): Service;


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
    getPublisherNamesAndTypesByNode(remoteNodeName: string, namespace?: string,
      noDemangle?: boolean): Array<NamesAndTypesQueryResult>;

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
    getSubscriptionNamesAndTypesByNode(remoteNodeName: string, namespace?: string,
      noDemangle?: boolean): Array<NamesAndTypesQueryResult>;

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
    getServiceNamesAndTypesByNode(remoteNodeName: string, namespace?: string): Array<NamesAndTypesQueryResult>;

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
    getTopicNamesAndTypes(noDemangle?: boolean): Array<NamesAndTypesQueryResult>;

    /**
		 * Get this node's service names and corresponding types.
		 *
		 * @returns An array of the names and types.
		 *        [
		 *          { name: '/start_motor', types: [ 'rplidar_ros/srv/Control' ] },
		 *          { name: '/stop_motor',  types: [ 'rplidar_ros/srv/Control' ] }
		 *        ]
		 */
    getServiceNamesAndTypes(): Array<NamesAndTypesQueryResult>

  }


  namespace rclnodejs {

    /**
		 * Default options when creating a Node, Publisher, Subscription, Client or Service
		 *
		 * ```ts
		 * {
		 *   enableTypedArray: true,
		 *   qos: QoS.profileDefault
		 * }
		 *
		 * ```
		 */
    export const DEFAULT_OPTIONS: Options;

  }

}
