

declare module 'rclnodejs' {

  /**
   * A ROS Client that interacts with a Service.
   */
  class Client extends Entity {

    /**
     * Send request to Service. Will be notified asynchronously with the Service response.
     * 
     * @param request - Request to be submitted.
     * @param callback - Callback for receiving the server response.
     */
    sendRequest(request: Message, callback: Client.ResponseCallback): void;

    readonly sequenceNumber: number;

    /**
     * Checks if the service is available.
     * @return true if the service is available.
     */
    isServiceServerAvailable(): boolean;

    /**
     * Wait until the service server is available or a timeout is reached. This
     * function polls for the service state so it may not return as soon as the
     * service is available.
     * 
     * @param timeout - Maximum amount of time to wait for, if timeout
     * is `undefined` or `< 0` then wait indefinitely.
     * @return true if the service is available; otherwise return false.
     */
    waitForService(timeout: number): Promise<boolean>;

    /**
     * Name of the service to make requests to.
     */
    readonly serviceName: string;

  }

  namespace Client {

    /**
     * This callback is called when a resopnse is received back from service
     * 
     * @param response - The response from the service
     * 
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
