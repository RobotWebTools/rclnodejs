// Copyright (c) 2020 Matt Richard. All rights reserved.
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

'use strict';

const rclnodejs = require('bindings')('rclnodejs');

/**
 * Get a list of action names and types for action clients associated with a node.
 * @param {Node} node - The node used for discovery.
 * @param {string} nodeName - The name of a remote node to get action clients for.
 * @param {string} namespace - Namespace of the remote node.
 * @return {array} - An array of the names and types.
 */
function getActionClientNamesAndTypesByNode(node, nodeName, namespace) {
  return rclnodejs.actionGetClientNamesAndTypesByNode(
    node.handle,
    nodeName,
    namespace
  );
}

/**
 * Get a list of action names and types for action servers associated with a node.
 * @param {Node} node - The node used for discovery.
 * @param {string} nodeName - The name of a remote node to get action servers for.
 * @param {string} namespace - Namespace of the remote node.
 * @return {array} - An array of the names and types.
 */
function getActionServerNamesAndTypesByNode(node, nodeName, namespace) {
  return rclnodejs.actionGetServerNamesAndTypesByNode(
    node.handle,
    nodeName,
    namespace
  );
}

/**
 * Get a list of action names and types.
 * @param {Node} node - The node used for discovery.
 * @return {array} - An array of the names and types.
 */
function getActionNamesAndTypes(node) {
  return rclnodejs.actionGetNamesAndTypes(node.handle);
}

module.exports = {
  getActionClientNamesAndTypesByNode,
  getActionServerNamesAndTypesByNode,
  getActionNamesAndTypes,
};
