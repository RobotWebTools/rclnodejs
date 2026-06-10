// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
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

// Type surface for the `rclnodejs/rosocket` subpath export.
//
// `rosocket` exposes ROS 2 topics and services as plain WebSocket URLs that
// carry ROS messages as JSON. Only the public `startRosocket` entry point is
// typed here.

import type { EventEmitter } from 'node:events';
import type { IncomingMessage } from 'node:http';
import type { Node } from 'rclnodejs';

/** Options accepted by {@link startRosocket}. */
export interface StartRosocketOptions {
  /** rclnodejs Node to host publishers/subscriptions/clients on. */
  node: Node;
  /** Port to listen on. Default `9000`. */
  port?: number;
  /** Host to bind to. Default `'0.0.0.0'`. */
  host?: string;
  /**
   * Optional default type per topic name,
   * e.g. `{ '/chatter': 'std_msgs/msg/String' }`.
   */
  topicTypes?: Record<string, string>;
  /** Optional default type per service name. */
  serviceTypes?: Record<string, string>;
  /**
   * Optional auth hook called with the raw HTTP upgrade request; return
   * `false` to reject the connection.
   */
  verifyClient?: (req: IncomingMessage) => boolean;
}

/** Handle returned by {@link startRosocket}. */
export interface RosocketServer {
  /**
   * The underlying WebSocket server (a `ws` `WebSocketServer`, typed here as
   * its `EventEmitter` supertype to avoid a hard dependency on `ws` types).
   */
  wss: EventEmitter;
  /** Stop the gateway and close all connections. */
  close: () => Promise<void>;
  /** The port the gateway is listening on. */
  port: number;
}

/**
 * Start a resource-style WebSocket gateway that exposes ROS 2 topics and
 * services as plain WebSocket URLs carrying ROS messages as JSON.
 *
 * URL scheme:
 * - `ws://host:port/topic/<topic_name>?type=<pkg>/msg/<Type>`
 * - `ws://host:port/service/<service_name>?type=<pkg>/srv/<Type>`
 *
 * @param options - Gateway configuration. `options.node` is required.
 * @returns A Promise resolving with the gateway handle.
 */
export function startRosocket(
  options: StartRosocketOptions
): Promise<RosocketServer>;
