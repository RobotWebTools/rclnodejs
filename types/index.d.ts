// / <reference path="base.d.ts" />

// import { rclnodejs } from "rclnodejs";

declare module 'rclnodejs' {
	

	/**
	 * Create a node.
	 * @param {string} nodeName - The name used to register in ROS.
	 * @param {string} namespace - The namespace used in ROS, default is an empty string.
	 * @param {Context} context - The context, default is Context.defaultContext().
	 * @return {Node} The instance of Node.
	 */
	function createNode(nodeName: string, namespace?: string, context?: Context): Node;

	/**
	* Init the module.
	* @param {Context} context - The context, default is Context.defaultContext().
	* @return {Promise<undefined>} A Promise.
	*/
	function init(context?: Context): Promise<void>;

	/**
	 * Start to spin the node, which triggers the event loop to start to check the incoming events.
	 * @param {Node} node - The node to be spun.
	 * @param {number} [timeout=10] - ms to wait, block forever if negative, don't wait if 0, default is 10.
	 * @return {undefined}
	 */
	function spin(node: Node, timeout?: number): void;

	/**
	 * @param {Context} context - The context to be shutdown.
	 * @return {undefined}
	 */
	function shutdown(context?: Context): void;

	/**
	 * Return status that whether the module is shut down.
	 * @return {boolean} Return true if the module is shut down, otherwise return false.
	 */
	function isShutdown(): boolean;

	/**
	 * Get the interface package, which is used by publisher/subscription or client/service.
	 * @param {string} name - The name of interface to be required.
	 * @return {object} - the object of the required package/interface.
	 */
	function require(name: string): object;

	/**
	 * Search packgaes which locate under path $AMENT_PREFIX_PATH, regenerate all JavaScript structs files from the IDL of
	 * messages(.msg) and services(.srv) and put these files under folder 'generated'. Any existing files under
	 * this folder will be overwritten after the execution.
	 * @return {Promise<undefined>} A Promise.
	 */
	function regenerateAll(): Promise<void>;

	/**
	 * Judge if the topic/service is hidden, see http://design.ros2.org/articles/topic_and_service_names.html#hidden-topic-or-service-names
	 * @param {string} name - Name of topic/service.
	 * @return {boolean} - True if a given topic or service name is hidden, otherwise False.
	 */
	function isTopicOrServiceHidden(name: string): boolean;

	/**
	 * Expand a given topic name using given node name and namespace as well.
	 * @param {string} topicName - Topic name to be expanded.
	 * @param {string} nodeName - Name of the node that this topic is associated with.
	 * @param {string} nodeNamespace - Namespace that the topic is within.
	 * @return {string} Expanded topic name which is fully qualified.
	 */
	function expandTopicName(topicName: string, nodeName: string, nodeNamespace?: string): string


	/**
	 * Create a plain JavaScript by specified type identifier
	 * @param {string|Object} type -- the type identifier, acceptable formats could be 'std_msgs/std/String'
	 *                                or {package: 'std_msgs', type: 'msg', name: 'String'}
	 * @return {Object|undefined} A plain JavaScript of that type
	 */
	function createMessageObject(type: string | object): object;

}