// Copyright (c) 2025 Mahmoud Alghalayini. All rights reserved.
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
   * Options for ParameterClient constructor.
   */
  export interface ParameterClientOptions {
    /**
     * Default timeout in milliseconds for service calls.
     * @default 5000
     */
    timeout?: number;
  }

  /**
   * Options for parameter service calls.
   */
  export interface ParameterServiceCallOptions {
    /**
     * Timeout in milliseconds for this specific call.
     */
    timeout?: number;

    /**
     * AbortSignal to cancel the request.
     */
    signal?: AbortSignal;
  }

  /**
   * Result of a parameter set operation.
   */
  export interface ParameterSetResult {
    /**
     * The name of the parameter.
     */
    name: string;

    /**
     * Whether the operation was successful.
     */
    successful: boolean;

    /**
     * Reason message, typically populated on failure.
     */
    reason: string;
  }

  /**
   * Result of a list parameters operation.
   */
  export interface ListParametersResult {
    /**
     * Array of parameter names found.
     */
    names: string[];

    /**
     * Array of parameter prefixes found.
     */
    prefixes: string[];
  }

  /**
   * Options for listing parameters.
   */
  export interface ListParametersOptions extends ParameterServiceCallOptions {
    /**
     * Optional array of parameter name prefixes to filter by.
     */
    prefixes?: string[];

    /**
     * Depth of parameter namespace to list.
     * @default 0 (unlimited)
     */
    depth?: number;
  }

  /**
   * Parameter to set with name and value.
   */
  export interface ParameterToSet {
    /**
     * The name of the parameter.
     */
    name: string;

    /**
     * The value to set. Type is automatically inferred.
     */
    value: any;
  }

  /**
   * Class for accessing parameters on remote ROS 2 nodes.
   *
   * Provides promise-based APIs to get, set, list, and describe parameters
   * on other nodes in the ROS 2 system.
   */
  export class ParameterClient {
    /**
     * Create a new ParameterClient for accessing parameters on a remote node.
     *
     * @param node - The node to use for creating service clients.
     * @param remoteNodeName - The name of the remote node whose parameters to access.
     * @param options - Optional configuration for the parameter client.
     * @throws {TypeError} If node is not provided or remoteNodeName is not a valid string.
     * @throws {Error} If remoteNodeName is not a valid ROS node name.
     */
    constructor(
      node: Node,
      remoteNodeName: string,
      options?: ParameterClientOptions
    );

    /**
     * Get the name of the remote node this client is connected to.
     */
    readonly remoteNodeName: string;

    /**
     * Get a single parameter from the remote node.
     *
     * @param name - The name of the parameter to retrieve.
     * @param options - Optional timeout and abort signal.
     * @returns Promise that resolves with the Parameter object.
     * @throws {Error} If the parameter is not found or service call fails.
     */
    getParameter(
      name: string,
      options?: ParameterServiceCallOptions
    ): Promise<Parameter>;

    /**
     * Get multiple parameters from the remote node.
     *
     * @param names - Array of parameter names to retrieve.
     * @param options - Optional timeout and abort signal.
     * @returns Promise that resolves with an array of Parameter objects.
     * @throws {Error} If the service call fails.
     * @throws {TypeError} If names is not a non-empty array.
     */
    getParameters(
      names: string[],
      options?: ParameterServiceCallOptions
    ): Promise<Parameter[]>;

    /**
     * Set a single parameter on the remote node.
     *
     * @param name - The name of the parameter to set.
     * @param value - The value to set. Type is automatically inferred.
     * @param options - Optional timeout and abort signal.
     * @returns Promise that resolves with the result.
     * @throws {Error} If the service call fails.
     */
    setParameter(
      name: string,
      value: any,
      options?: ParameterServiceCallOptions
    ): Promise<ParameterSetResult>;

    /**
     * Set multiple parameters on the remote node.
     *
     * @param parameters - Array of parameter objects with name and value.
     * @param options - Optional timeout and abort signal.
     * @returns Promise that resolves with an array of results.
     * @throws {Error} If the service call fails.
     * @throws {TypeError} If parameters is not a non-empty array.
     */
    setParameters(
      parameters: ParameterToSet[],
      options?: ParameterServiceCallOptions
    ): Promise<ParameterSetResult[]>;

    /**
     * List all parameters available on the remote node.
     *
     * @param options - Optional filters, depth, timeout and abort signal.
     * @returns Promise that resolves with parameter names and prefixes.
     * @throws {Error} If the service call fails.
     */
    listParameters(
      options?: ListParametersOptions
    ): Promise<ListParametersResult>;

    /**
     * Describe parameters on the remote node.
     *
     * @param names - Array of parameter names to describe.
     * @param options - Optional timeout and abort signal.
     * @returns Promise that resolves with an array of parameter descriptors.
     * @throws {Error} If the service call fails.
     * @throws {TypeError} If names is not a non-empty array.
     */
    describeParameters(
      names: string[],
      options?: ParameterServiceCallOptions
    ): Promise<any[]>;

    /**
     * Get the types of parameters on the remote node.
     *
     * @param names - Array of parameter names.
     * @param options - Optional timeout and abort signal.
     * @returns Promise that resolves with an array of parameter types.
     * @throws {Error} If the service call fails.
     * @throws {TypeError} If names is not a non-empty array.
     */
    getParameterTypes(
      names: string[],
      options?: ParameterServiceCallOptions
    ): Promise<ParameterType[]>;

    /**
     * Wait for the parameter services to be available on the remote node.
     * This is useful to verify the remote node is running before making calls.
     *
     * @param timeout - Optional timeout in milliseconds.
     * @returns Promise that resolves to true if services are available.
     */
    waitForService(timeout?: number): Promise<boolean>;

    /**
     * Check if the parameter client has been destroyed.
     *
     * @returns True if destroyed, false otherwise.
     */
    isDestroyed(): boolean;

    /**
     * Destroy the parameter client and clean up all service clients.
     * After calling this method, the client cannot be used anymore.
     */
    destroy(): void;
  }
}
