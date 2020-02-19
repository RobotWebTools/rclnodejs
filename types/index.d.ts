// eslint-disable-next-line spaced-comment
/// <reference path="base.d.ts" />

declare module 'rclnodejs' {

	/**
	 * Create a node.
	 *
	 * @remarks
	 * See {@link Node}
	 *
	 * @param nodeName - The name used to register in ROS.
	 * @param namespace - The namespace used in ROS, default is an empty string.
	 * @param context - The context, default is Context.defaultContext().
   * @param options - The node options, default is NodeOptions.defaultOptions.
	 * @returns The new Node instance.
	 */
	function createNode(nodeName: string, namespace?: string, context?: Context, options?: NodeOptions): Node;

	/**
	 * Init the module.
	 *
	 * @param context - The context, default is Context.defaultContext().
   * @param argv - The commandline arguments, the default value is process.argv.
	 * @returns A Promise.
	 */
	function init(context?: Context, argv?: string[]): Promise<void>;

	/**
	 * Spin up the node event loop to check for incoming events.
	 *
	 * @param node - The node to be spun.
	 * @param timeout - ms to wait, block forever if negative, return immediately when 0, default is 10.
	 */
	function spin(node: Node, timeout?: number): void;

	/**
	 * Execute one item of work or wait until a timeout expires.
	 * 
	 * @param node - The node to be spun.
	 * @param timeout - ms to wait, block forever if negative, return immediately when 0, default is 10.
	 */
	function spinOnce(node: Node, timeout?: number): void;

	/**
	 * Stop all activity, destroy all nodes and node components.
	 *
	 * @param context - The context, default is Context.defaultContext()
	 */
	function shutdown(context?: Context): void;

	/**
	 * Test if the module is shutdown.
	 *
	 * @returns True if the module is shut down, otherwise return false.
	 */
	function isShutdown(): boolean;

	/**
	 * Get the interface package, which is used by publisher/subscription or client/service.
	 *
	 * @param  name - The name of interface to be required.
	 * @returns The object of the required package/interface.
	 */
	function require<T extends MessageTypeClassName>(name: T): MessageWrapperType<T>;
	function require(name: string): object;

	/**
	 * Generate JavaScript structs files from the IDL of
	 * messages(.msg) and services(.srv).
	 * Search packages which locate under path $AMENT_PREFIX_PATH
	 * and output JS files into the 'generated' folder.
	 * Any existing files under the generated folder will
	 * be overwritten.
	 *
	 * @returns A Promise.
	 */
	function regenerateAll(): Promise<void>;

	/**
	 * Judge if the topic or service is hidden,
	 *
	 * @remarks
	 * See {@link http://design.ros2.org/articles/topic_and_service_names.html#hidden-topic-or-service-names}
	 *
	 * @param name - Name of topic or service.
	 * @returns True if a given topic or service name is hidden, otherwise False.
	 */
	function isTopicOrServiceHidden(name: string): boolean;

	/**
	 * Expand a given topic name using given node name and namespace.
	 *
	 * @param  topicName - Topic name to be expanded.
	 * @param  nodeName - Name of the node that this topic is associated with.
	 * @param  nodeNamespace - Namespace that the topic is within.
	 * @returns Expanded topic name which is fully qualified.
	 */
	function expandTopicName(topicName: string, nodeName: string, nodeNamespace?: string): string


	/**
	 * Create a plain JavaScript message object.
	 *
	 * @param type - type identifier, acceptable formats could be 'std_msgs/std/String'
	 *                                or {package: 'std_msgs', type: 'msg', name: 'String'}
	 * @returns A Message object or undefined if type is not recognized.
	 */
	function createMessageObject(type: TypeClass): Message;

}
