declare module 'rclnodejs' {
  /**
   * A ROS service client.
   */
  interface Client<T extends TypeClass<ServiceTypeClassName>> extends Entity {
    /**
     * Make a service request and wait for to be notified asynchronously through a callback.
     *
     * @param request - Request to be submitted.
     * @param callback - Callback for receiving the server response.
     */
    sendRequest(
      request: ServiceRequestMessage<T>,
      callback: Client.ResponseCallback<T>
    ): void;

    /**
     * Checks if the service is ready.
     *
     * @returns true if the service is available.
     */
    isServiceServerAvailable(): boolean;

    /**
     * Wait for service server to become available or the timeout expires.
     *
     * @remarks
     * This method polls for the service state. Thus it may not return the instant the
     * service becomes available.
     *
     *
     * @param timeout - Maximum amount of time to wait. If timeout
     *                  is `< 0` then wait indefinitely, default =
     * 									wait indefinitely.
     * @returns True if the service is available; otherwise return false.
     */
    waitForService(timeout?: number): Promise<boolean>;

    /**
     * Name of the service to which requests are made.
     */
    readonly serviceName: string;
  }

  namespace Client {
    /**
     * A callback for receiving a response from the service
     *
     * @param response - The response from the service
     *
     * @remarks
     * See {@link Client.sendRequest | Client.sendRequest}
     * See {@link Node.createService | Node.createService}
     * See {@link Client}
     * See {@link Service}
     */
    export type ResponseCallback<T extends TypeClass<ServiceTypeClassName>> = (
      response: ServiceResponseMessage<T>
    ) => void;
  }
}
