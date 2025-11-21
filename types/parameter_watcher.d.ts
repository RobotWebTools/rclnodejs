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
  import { EventEmitter } from 'events';

  /**
   * Options for ParameterWatcher constructor.
   */
  export interface ParameterWatcherOptions {
    /**
     * Default timeout in milliseconds for service calls.
     * @default 5000
     */
    timeout?: number;
  }

  /**
   * ParameterWatcher - Watches parameter changes on a remote node.
   *
   * Subscribes to /parameter_events and emits 'change' events when
   * watched parameters on the target node are modified.
   */
  class ParameterWatcher extends EventEmitter {
    /**
     * Get the remote node name being watched.
     */
    readonly remoteNodeName: string;

    /**
     * Get the list of watched parameter names.
     */
    readonly watchedParameters: string[];

    /**
     * Start watching for parameter changes.
     * Waits for the remote node's parameter services and subscribes to parameter events.
     *
     * @param timeout - Timeout in milliseconds to wait for services.
     * @returns Promise that resolves to true when watching has started.
     */
    start(timeout?: number): Promise<boolean>;

    /**
     * Get current values of all watched parameters.
     *
     * @param options - Optional request options (timeout, signal).
     * @returns Promise that resolves to an array of Parameter objects.
     */
    getCurrentValues(options?: {
      timeout?: number;
      signal?: AbortSignal;
    }): Promise<Parameter[]>;

    /**
     * Add a parameter name to the watch list.
     *
     * @param name - Parameter name to watch.
     */
    addParameter(name: string): void;

    /**
     * Remove a parameter name from the watch list.
     *
     * @param name - Parameter name to stop watching.
     * @returns True if the parameter was in the watch list.
     */
    removeParameter(name: string): boolean;

    /**
     * Check if the watcher has been destroyed.
     *
     * @returns True if destroyed.
     */
    isDestroyed(): boolean;

    /**
     * Destroy the watcher and clean up resources.
     * Unsubscribes from parameter events and destroys the parameter client.
     */
    destroy(): void;

    /**
     * Event emitted when watched parameters change.
     *
     * @event change
     * @param params - Array of changed parameters (ParameterValue messages).
     */
    on(event: 'change', listener: (params: any[]) => void): this;
    once(event: 'change', listener: (params: any[]) => void): this;
    emit(event: 'change', params: any[]): boolean;
  }
}
