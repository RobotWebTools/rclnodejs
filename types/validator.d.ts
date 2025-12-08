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
   * Validator for ROS 2 names (topics, services, nodes, namespaces).
   */
  namespace validator {
    /**
     * Validate a fully-qualified topic or service name.
     * The name must be fully-qualified and already expanded.
     * @param topic - The topic/service name to validate.
     * @returns Always returns true if valid.
     * @throws TypeValidationError if topic is not a string.
     * @throws NameValidationError if the name is invalid.
     */
    function validateFullTopicName(topic: string): true;

    /**
     * Check if a fully-qualified topic name is valid without throwing.
     * @param topic - The topic/service name to check.
     * @returns True if valid, false otherwise.
     */
    function isValidFullTopicName(topic: string): boolean;

    /**
     * Validate a node name.
     * @param name - The node name to validate.
     * @returns Always returns true if valid.
     * @throws TypeValidationError if name is not a string.
     * @throws NameValidationError if the name is invalid.
     */
    function validateNodeName(name: string): true;

    /**
     * Check if a node name is valid without throwing.
     * @param name - The node name to check.
     * @returns True if valid, false otherwise.
     */
    function isValidNodeName(name: string): boolean;

    /**
     * Validate a topic or service name.
     * The name does not have to be fully-qualified and is not expanded.
     * @param topic - The topic/service name to validate.
     * @returns Always returns true if valid.
     * @throws TypeValidationError if topic is not a string.
     * @throws NameValidationError if the name is invalid.
     */
    function validateTopicName(topic: string): true;

    /**
     * Check if a topic name is valid without throwing.
     * @param topic - The topic/service name to check.
     * @returns True if valid, false otherwise.
     */
    function isValidTopicName(topic: string): boolean;

    /**
     * Validate a namespace.
     * @param namespace - The namespace to validate.
     * @returns Always returns true if valid.
     * @throws TypeValidationError if namespace is not a string.
     * @throws NameValidationError if the namespace is invalid.
     */
    function validateNamespace(namespace: string): true;

    /**
     * Check if a namespace is valid without throwing.
     * @param namespace - The namespace to check.
     * @returns True if valid, false otherwise.
     */
    function isValidNamespace(namespace: string): boolean;
  }
}
