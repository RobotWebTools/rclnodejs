import { QoS } from 'rclnodejs';

declare module 'rclnodejs' {

     /** 
      * The ROS message class 
      */
     type TypeClass =
          (() => any) |
          string |   // a string representing the message class, e.g. 'std_msgs/msg/String',
          {          // object representing a message class, e.g. {package: 'std_msgs', type: 'msg', name: 'String'}
               package: string; 
               type: string; 
               name: string };    


     /**
      * Options for configuring new Publishers, Subscribers,
      * Clients and Services. 
      * 
      * see DEFAULT_OPTIONS
      */
     type CommunicationOptions = {
          enableTypedArray: boolean;
          qos: QoS | QoS.ProfileRef;
     }


     /**
      * A response to a Client request.
      * 
      * Note: you can use response.template to get an empty result message.
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
           * @see {@link Response#template}
           */
          send(response: Message): void;
     }

     /**
      * A callback for receiving notification when a message is published
      * 
      * @param message - The message published
      * 
      * @see [Node.createSubscription]{@link Node#createSubscription}
      * @see [Node.createPublisher]{@link Node#createPublisher}
      * @see {@link Publisher}
      * @see {@link Subscription}
      */
     type SubscriptionCallback = (
          // * @param message - The message published
          (message: object) => void
     );


     /**
      * Callback for receiving requests for service from a client.
     
      * @param request - The request sent to the service
      * @param response - The response to client.
      * 
           Use [response.send()]{@link Response#send} to send response object to client
     * @return {undefined}
     * @see [Node.createService]{@link Node#createService}
     * @see [Client.sendRequest]{@link Client#sendRequest}
     * @see {@link Client}
     * @see {@link Service}
     * @see {@link Response#send}
     */
     type ServiceRequestCallback = (
          (request: object, response: ServiceResponse) => void
     );


     /**
     * Callback for receiving times-out interrupts from a Timer.
     * 
     * @see [Node.createTimer]{@link Node#createTimer}
     * @see {@link Timer}
     */
     type TimerRequestCallback = (
          () => void
     );

     /**
      * Standard result of Node.getXXXNamesAndTypes() queries 
      * 
      * @example
      * [ 
      *   { name: '/rosout', types: [ 'rcl_interfaces/msg/Log' ] },
      *   { name: '/scan',   types: [ 'sensor_msgs/msg/LaserScan' ] },
      *   { name: '/topic',  types: [ 'std_msgs/msg/String' ] }
      * ]
      */
     type NamesAndTypesQueryResult = {
          name: string;
          types: Array<string>;
     }


     /**
      * 
      */
     class Node {

       /**  
           * Get the name of the node.
           * @return {string}
          */
       name(): string;

       /**
           * Get the namespace of the node.
           * @return {string}
           */
       namespace(): string;

       /**
           * Create a Timer.
           * 
           * @param period - Elapsed time between interrupt events (milliseconds).
           * @param callback - Called on timeout interrupt.
           * @param context - Context, default is Context.defaultContext().
           * @return New instance of Timer.
           */
       createTimer(period: number, callback: TimerRequestCallback, context?: Context): Timer;

       /**
           * Create a Publisher.
           * 
           * @param typeClass - Publish messages of this type.
           * @param topic - Name of the topic.
           * @param options - Configuration options, see DEFAULT_OPTIONS
           * @return New instance of Publisher.
           */
       createPublisher(typeClass: TypeClass, topic: string, options?: CommunicationOptions): Publisher;

       /**
           * Create a Subscription.
           * 
           * @param typeClass -  Listen for messages of this type.
           * @param topic - Listen for messages on this topic.
           * @param options - Configuration options, see DEFAULT_OPTIONS_
           * @param callback - Called when a new message is received.
           * @return New instance of Subscription.
           */
       createSubscription(typeClass: TypeClass, topic: string,
               options: CommunicationOptions, callback: SubscriptionCallback): Subscription;

       /**
           * Create a Client for making server requests.
           * 
           * @param typeClass -  Service requests message type.
           * @param serviceName - The service name to request.
           * @param options - The options argument used to parameterize the client.
           * @return New instance of Client.
           */
       createClient(typeClass: TypeClass, serviceName: string, options?: CommunicationOptions): Client;

       /**
           * Create a Service.
           * 
           * @param typeClass - ROS message class of 
           * @param serviceName - Service name to offer.
           * @param options - Configuration options
           * @param callback - Notified when receiving a request.
           * @return An instance of Service.
           * @see {@link RequestCallback}
           */
       createService(typeClass: TypeClass, serviceName: string,
               options: CommunicationOptions, callback: ServiceRequestCallback): Service;


       /**
           * Destroy all resource allocated by this node, including
           * <code>Timer</code>s/<code>Publisher</code>s/<code>Subscription</code>s
           * /<code>Client</code>s/<code>Service</code>s
           */
       destroy(): void;

       /**
           * Destroy a Publisher.
           * @param publisher - Publisher to be destroyed.
           */
       destroyPublisher(publisher: Publisher): void;

       /**
           * Destroy a Subscription.
           * @param subscription - Subscription to be destroyed.
           */
       destroySubscription(subscription: Subscription): void;

       /**
           * Destroy a Client.
           * @param client - Client to be destroyed.
           */
       destroyClient(client: Client): void;

       /**
           * Destroy a Service.
           * @param service - Service to be destroyed.
           */
       destroyService(service: Service): void;

       /**
           * Destroy a Timer.
           * @param timer - Timer to be destroyed.
           */
       destroyTimer(timer: Timer): void;

         
       /**
           * Get the list of published topics discovered by the provided node for the remote node name.
           * 
           * @param nodeName - Name of the node.
           * @param namespace - Name of the namespace.
           * @param  noDemangle - If true, topic names and types returned will not be demangled, default: false.
           * @return An array of the names and types.
           *        [ 
           *          { name: '/rosout', types: [ 'rcl_interfaces/msg/Log' ] },
           *          { name: '/scan', types: [ 'sensor_msgs/msg/LaserScan' ] } 
           *        ]
           */
       getPublisherNamesAndTypesByNode(nodeName: string, namespace?: string, 
               noDemangle?: boolean): Array<NamesAndTypesQueryResult>;

       /**
           * Get the list of published topics discovered by the provided node for the remote node name.
           * 
           * @param nodeName - ame of the node.
           * @param namespace - Name of the namespace.
           * @param noDemangle - If true topic, names and types returned will not be demangled, default: false.
           * @return An array of the names and types.
           *        [ 
           *          { name: '/topic', types: [ 'std_msgs/msg/String' ] } 
           *        ]s
           */
       getSubscriptionNamesAndTypesByNode(nodeName: string, namespace?: string, 
               noDemangle?: boolean): Array<NamesAndTypesQueryResult>;

       /**
           * Get the list of service topics discovered by the provided node for the remote node name.
           * 
           * @param nodeName - Name of the node.
           * @param namespace - Name of the namespace.
           * @return An array of the names and types.
           *        [ 
           *          { name: '/rosout', types: [ 'rcl_interfaces/msg/Log' ] },
           *          ... 
           *        ]
           */
       getServiceNamesAndTypesByNode(nodeName: string, namespace?: string): Array<NamesAndTypesQueryResult>;

       /**
           * Get the list of topics discovered by the provided node.
           * 
           * @param noDemangle - If true. topic names and types returned will not be demangled, default: false.
           * @return An array of the names and types.
           *        [ 
           *         { name: '/rosout', types: [ 'rcl_interfaces/msg/Log' ] },
           *         { name: '/scan', types: [ 'sensor_msgs/msg/LaserScan' ] },
           *         { name: '/topic', types: [ 'std_msgs/msg/String' ] } 
           *        ]
           */
       getTopicNamesAndTypes(noDemangle?: boolean): Array<NamesAndTypesQueryResult>;

       /**
           * Get the list of services discovered by the provided node.
           * 
           * @return - An array of the names and types.
           *        [ 
           *          { name: '/start_motor', types: [ 'rplidar_ros/srv/Control' ] },
           *          { name: '/stop_motor',  types: [ 'rplidar_ros/srv/Control' ] } 
           *        ]
           */
       getServiceNamesAndTypes(): Array<NamesAndTypesQueryResult>
     }


     namespace rclnodejs {

          /**
           * Default options when creating a new Node, Publisher, Subscription, Client or Service
           * 
           * ```ts
           * {
           *   enableTypedArray: true,
           *   qos: QoS.profileDefault
           * }
           * 
           * ```
           */
          export const DEFAULT_OPTIONS: CommunicationOptions;

     }

}
