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
     * Make a service request and return a Promise that resolves with the response.
     *
     * @param request - Request to be submitted.
     * @param options - Optional parameters for the request.
     * @returns Promise that resolves with the service response.
     * @throws TimeoutError if the request times out (when options.timeout is exceeded).
     * @throws AbortError if the request is manually aborted (via options.signal).
     * @throws Error if the request fails for other reasons.
     */
    sendRequestAsync(
      request: ServiceRequestMessage<T>,
      options?: Client.AsyncRequestOptions
    ): Promise<ServiceResponseMessage<T>>;

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

    /**
     * Configure introspection.
     * @param clock - Clock to use for service event timestamps
     * @param QoSProfile - QOS profile for the service event publisher
     * @param introspectionState - The state to set introspection to
     */
    configureIntrospection(
      clock: Clock,
      serviceEventPubQOS: QoS,
      introspectionState: ServiceIntrospectionStates
    ): void;

    /**
     * Get the logger name for this client.
     */
    readonly loggerName: string;
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

    /**
     * Options for async service requests
     */
    export interface AsyncRequestOptions {
      /**
       * Timeout in milliseconds for the request.
       * Internally uses AbortSignal.timeout() for standards compliance.
       */
      timeout?: number;

      /**
       * AbortSignal to cancel the request.
       * When the signal is aborted, the request will be cancelled
       * and the promise will reject with an AbortError.
       *
       * Can be combined with timeout parameter - whichever happens first
       * will abort the request.
       */
      signal?: AbortSignal;
    }
  }
}
