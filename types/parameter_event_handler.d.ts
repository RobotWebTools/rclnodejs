// Copyright (c) 2026, The Robot Web Tools Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

declare module 'rclnodejs' {
  /**
   * Options for ParameterEventHandler constructor.
   */
  export interface ParameterEventHandlerOptions {
    /**
     * QoS profile for the parameter_events subscription.
     */
    qos?: QoS;
  }

  /**
   * Opaque handle returned when adding a parameter callback.
   * Used to remove the callback later via removeParameterCallback().
   */
  class ParameterCallbackHandle {
    readonly parameterName: string;
    readonly nodeName: string;
    readonly callback: (parameter: any) => void;
  }

  /**
   * Opaque handle returned when adding a parameter event callback.
   * Used to remove the callback later via removeParameterEventCallback().
   */
  class ParameterEventCallbackHandle {
    readonly callback: (event: any) => void;
  }

  /**
   * ParameterEventHandler - Monitors and responds to parameter changes
   * on any node in the ROS 2 graph.
   *
   * Subscribes to `/parameter_events` and dispatches callbacks when
   * parameters are added, changed, or deleted on any node.
   *
   * Two types of callbacks:
   * - **Parameter callbacks**: for a specific parameter on a specific node
   * - **Event callbacks**: for every ParameterEvent message received
   *
   * @example
   * ```typescript
   * const handler = node.createParameterEventHandler();
   *
   * const handle = handler.addParameterCallback(
   *   'my_param', '/my_node',
   *   (param) => console.log(`Changed: ${param.name}`)
   * );
   *
   * handler.removeParameterCallback(handle);
   * handler.destroy();
   * ```
   */
  class ParameterEventHandler {
    /**
     * Add a callback for a specific parameter on a specific node.
     *
     * @param parameterName - Name of the parameter to monitor.
     * @param nodeName - Fully qualified name of the node (e.g., '/my_node').
     * @param callback - Called with the parameter message when it changes.
     * @returns A handle for removing this callback later.
     */
    addParameterCallback(
      parameterName: string,
      nodeName: string,
      callback: (parameter: any) => void
    ): ParameterCallbackHandle;

    /**
     * Remove a previously added parameter callback.
     *
     * @param handle - The handle returned by addParameterCallback.
     */
    removeParameterCallback(handle: ParameterCallbackHandle): void;

    /**
     * Add a callback that is invoked for every parameter event.
     *
     * @param callback - Called with the full ParameterEvent message.
     * @returns A handle for removing this callback later.
     */
    addParameterEventCallback(
      callback: (event: any) => void
    ): ParameterEventCallbackHandle;

    /**
     * Configure which node parameter events will be received.
     *
     * If nodeNames is omitted or empty, the node filter is cleared.
     * Relative names are resolved against the handler node namespace.
     *
     * @param nodeNames - Node names to filter parameter events from.
     * @returns True if the filter is active or was successfully cleared.
     */
    configureNodesFilter(nodeNames?: string[]): boolean;

    /**
     * Remove a previously added parameter event callback.
     *
     * @param handle - The handle returned by addParameterEventCallback.
     */
    removeParameterEventCallback(handle: ParameterEventCallbackHandle): void;

    /**
     * Check if the handler has been destroyed.
     */
    isDestroyed(): boolean;

    /**
     * Destroy the handler and clean up resources.
     */
    destroy(): void;

    /**
     * Get a specific parameter from a ParameterEvent message.
     *
     * @param event - A ParameterEvent message.
     * @param parameterName - The parameter name to look for.
     * @param nodeName - The node name to match.
     * @returns The matching parameter message, or null.
     */
    static getParameterFromEvent(
      event: any,
      parameterName: string,
      nodeName: string
    ): any | null;

    /**
     * Get all parameters from a ParameterEvent message (new + changed).
     *
     * @param event - A ParameterEvent message.
     * @returns Array of parameter messages.
     */
    static getParametersFromEvent(event: any): any[];
  }
}
