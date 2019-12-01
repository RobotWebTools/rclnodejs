import { QoS } from "rclnodejs";

declare module "rclnodejs" {

     /** The ROS message class */
     type TypeClass =
          (() => any) |
          string |   //a string representing the message class, e.g. 'std_msgs/msg/String',
          { package: string; type: string; name: string };    //an object representing the message class, e.g. {package: 'std_msgs', type: 'msg', name: 'String'}


     type CommunicationOptions = {
          enableTypedArray: boolean;
          qos: QoS | QoS.ProfileRef;
     }

     // const DEFAULT_OPTIONS: CommunicationOptions = {
     //      enableTypedArray: true,
     //      qos: QoS.profileDefault
     // };

     /**
      * The response to client
      *
      * @property {Object} template - Get an empty response object that is being sent to client.
      * @property {Service} service - The service that this response object is attaching to.
      *
      * @hideconstructor
      */
     class ServiceResponse {

          readonly template: {};
          readonly service: Service;

          /**
           * Send response to client (the service caller)
           * @param {object} response - The plain JavaScript representing the response.
                Note: you can use .template to get an empty result object.
           * @return {undefined}
           * @see {@link Response#template}
           */
          send(response: object): void;
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
     type SubscriptionCallback = (
          // * @param {Object} message - The message published
          (message: object) => void
     );


     /**
      * This callback is called when a request is received by a service
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
     type ServiceRequestCallback = (
          (request: object, response: ServiceResponse) => void
     );


     /**
     * This callback is called when a timer times-out
     * @callback TimerCallback
     * @return {undefined}
     * @see [Node.createTimer]{@link Node#createTimer}
     * @see {@link Timer}
     */
     type TimerRequestCallback = (
          () => void
     );

     /**
      * Standard result of Node.getXXXNamesAndTypes() queries 
      */
     type NamesAndTypesQueryResult = {
          name: string;
          types: Array<string>;
     }


     class Node {

           /* Get the name of the node.
               * @return {string}
               */
          name(): string;

          /* Get the namespace of the node.
               * @return {string}
               */
          namespace(): string;

          /**
           * Create a Timer.
           * @param {number} period - The number representing period in millisecond.
           * @param {TimerCallback} callback - The callback to be called when timeout.
           * @param {Context} context - The context, default is Context.defaultContext().
           * @return {Timer} - An instance of Timer.
           */
          createTimer(period: number, callback: TimerRequestCallback, context?: Context): Timer;

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
          createPublisher(typeClass: TypeClass, topic: string, options?: CommunicationOptions): Publisher;

          /**
           * Create a Subscription.
           * @param {function|string|object} typeClass - The ROS message class,
                OR a string representing the message class, e.g. 'std_msgs/msg/String',
                    OR an object representing the message class, e.g. {package: 'std_msgs', type: 'msg', name: 'String'}
               * @param {string} topic - The name of the topic.
               * @param {object} options - The options argument used to parameterize the subscription.
               * @param {boolean} options.enableTypedArray - The topic will use TypedArray if necessary, default: true.
               * @param {QoS} options.qos - ROS Middleware "quality of service" settings for the subscription, default: QoS.profileDefault.
               * @param {SubscriptionCallback} callback - The callback to be call when receiving the topic subscribed.
               * @return {Subscription} - An instance of Subscription.
               * @see {@link SubscriptionCallback}
               */
          createSubscription(typeClass: TypeClass, topic: string,
               options: CommunicationOptions, callback: SubscriptionCallback): Subscription;

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
          createClient(typeClass: TypeClass, serviceName: string, options?: CommunicationOptions): Client;

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
          createService(typeClass: TypeClass, serviceName: string,
               options: CommunicationOptions, callback: ServiceRequestCallback): Service;


          /**
           * Destroy all resource allocated by this node, including
           * <code>Timer</code>s/<code>Publisher</code>s/<code>Subscription</code>s
           * /<code>Client</code>s/<code>Service</code>s
           * @return {undefined}
           */
          destroy(): void;

          /**
           * Destroy a Publisher.
           * @param {Publisher} publisher - The Publisher to be destroyed.
           * @return {undefined}
           */
          destroyPublisher(publisher: Publisher): void;

          /**
           * Destroy a Subscription.
           * @param {Subscription} subscription - The Subscription to be destroyed.
           * @return {undefined}
           */
          destroySubscription(subscription: Subscription): void;

          /**
           * Destroy a Client.
           * @param {Client} client - The Client to be destroyed.
           * @return {undefined}
           */
          destroyClient(client: Client): void;

          /**
           * Destroy a Service.
           * @param {Service} service - The Service to be destroyed.
           * @return {undefined}
           */
          destroyService(service: Service): void;

          /**
           * Destroy a Timer.
           * @param {Timer} timer - The Timer to be destroyed.
           * @return {undefined}
           */
          destroyTimer(timer: Timer): void;

         
          /**
           * Get the list of published topics discovered by the provided node for the remote node name.
           * @param {string} nodeName - The name of the node.
           * @param {string} namespace - The name of the namespace.
           * @param {boolean} noDemangle - If true topic names and types returned will not be demangled, default: false.
           * @return {array} - An array of the names and types.
           *                  [ 
           *                       { name: '/rosout', types: [ 'rcl_interfaces/msg/Log' ] },
           *                       { name: '/scan', types: [ 'sensor_msgs/msg/LaserScan' ] } 
           *                  ]
           */
          getPublisherNamesAndTypesByNode(nodeName: string, namespace?: string, noDemangle?: boolean): Array<NamesAndTypesQueryResult>;

          /**
           * Get the list of published topics discovered by the provided node for the remote node name.
           * @param {string} nodeName - The name of the node.
           * @param {string} namespace - The name of the namespace.
           * @param {boolean} noDemangle - If true topic names and types returned will not be demangled, default: false.
           * @return {array} - An array of the names and types.
           *                  [ 
           *                       { name: '/topic', types: [ 'std_msgs/msg/String' ] } 
           *                  ]
           */
          getSubscriptionNamesAndTypesByNode(nodeName: string, namespace?: string, noDemangle?: boolean): Array<NamesAndTypesQueryResult>;

          /**
           * Get the list of service topics discovered by the provided node for the remote node name.
           * @param {string} nodeName - The name of the node.
           * @param {string} namespace - The name of the namespace.
           * @return {array} - An array of the names and types.
           *                    [ 
           *                       { name: '/rosout', types: [ 'rcl_interfaces/msg/Log' ] },
           *                       ... 
           *                    ]
           */
          getServiceNamesAndTypesByNode(nodeName: string, namespace?: string): Array<NamesAndTypesQueryResult>;

          /**
           * Get the list of topics discovered by the provided node.
           * @param {boolean} noDemangle - If true topic names and types returned will not be demangled, default: false.
           * @return {array} - An array of the names and types.
           *                    [ 
           *                       { name: '/rosout', types: [ 'rcl_interfaces/msg/Log' ] },
           *                       { name: '/scan', types: [ 'sensor_msgs/msg/LaserScan' ] },
           *                       { name: '/topic', types: [ 'std_msgs/msg/String' ] } 
           *                    ]
           */
          getTopicNamesAndTypes(noDemangle?: boolean): Array<NamesAndTypesQueryResult>;

          /**
           * Get the list of services discovered by the provided node.
           * @return {array} - An array of the names and types.
           *             [ 
           *               { name: '/start_motor', types: [ 'rplidar_ros/srv/Control' ] },
           *               { name: '/stop_motor',  types: [ 'rplidar_ros/srv/Control' ] } 
           *             ]
           */
          getServiceNamesAndTypes(): Array<NamesAndTypesQueryResult>
     }

}
