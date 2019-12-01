
declare module "rclnodejs" {

  /**
   * @class - Class representing a Client in ROS
   * @hideconstructor
   */
  class Client extends Entity {

    /**
     * Send the request and will be notified asynchronously if receiving the repsonse.
     * @param {object} request - The request to be submitted.
     * @param {ResponseCallback} callback - Thc callback function for receiving the server response.
     * @return {undefined}
     * @see {@link ResponseCallback}
     */
    sendRequest(request: object, callback: Client.ResponseCallback): void;

    readonly sequenceNumber: number;

    /**
     * Checks if the service is available.
     * @return {boolean} true if the service is available.
     */
    isServiceServerAvailable(): boolean;

    /**
     * Wait until the service server is available or a timeout is reached. This
     * function polls for the service state so it may not return as soon as the
     * service is available.
     * @param {number} timeout The maximum amount of time to wait for, if timeout
     * is `undefined` or `< 0`, this will wait indefinitely.
     * @return {Promise<boolean>} true if the service is available.
     */
    waitForService(timeout: number): Promise<boolean>;

    /**
     * @type {string}
     */
    readonly serviceName: string;

  }

  namespace Client {

    /**
     * This callback is called when a resopnse is sent back from service
     * @callback ResponseCallback
     * @param {Object} response - The response sent from the service
     * @see [Client.sendRequest]{@link Client#sendRequest}
     * @see [Node.createService]{@link Node#createService}
     * @see {@link Client}
     * @see {@link Service}
     */
    export type ResponseCallback = (
      (response: object) => void
    );
  }

}
