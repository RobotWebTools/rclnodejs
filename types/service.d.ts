declare module 'rclnodejs' {
  type ServiceRequestMessage<T> = T extends ServiceTypeClassName
    ? InstanceType<ServicesMap[T]['Request']>
    : object;

  type ServiceResponseMessage<T> = T extends ServiceTypeClassName
    ? InstanceType<ServicesMap[T]['Response']>
    : object;

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
  type ServiceRequestHandler<T extends TypeClass<ServiceTypeClassName>> = (
    request: ServiceRequestMessage<T>,
    response: ServiceResponse<T>
  ) => void;

  /**
   * Callback for receiving service requests from a client.
   *
   * @param request - The request sent to the service
   * @param response - The response to the client.
   *
   * @deprecated use {@link ServiceRequestHandler | ServiceRequestHandler}
   */
  type ServiceRequestCallback<
    T extends TypeClass<ServiceTypeClassName>
  > = ServiceRequestHandler<T>;

  /**
   * A service response to a client request.
   *
   * @remarks
   * Use {@link response.template | response.template} to get an empty result message.
   */
  class ServiceResponse<T extends TypeClass<ServiceTypeClassName>> {
    /**
     * Get an empty response message object.
     * The template will be a message of type: <pkg>.msg.<serviceTypeClass>_Response.
     * e.g., example_interface/srv/AddTwoInts_Response
     */
    readonly template: ServiceResponseMessage<T>;

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
    send(response: ServiceResponseMessage<T>): void;
  }

  /**
   * A ROS Service server.
   */
  interface ROSService extends Entity {
    /**
     * Name of the service.
     */
    readonly serviceName: string;
  }
}
