// Copyright (c) 2019 Intel Corporation. All rights reserved.
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

let defaultContext = undefined;

/**
 * @class - Class representing a Context in ROS
 * @hideconstructor
 */

class Context {
  constructor() {
    this._handle = rclnodejs.createContext();
  }

  /**
   * Shutdown the context.
   * @return {undefined}
   */
  shutdown() {
    rclnodejs.shutdown(this._handle);
  }

  /**
   * Try to shutdown the context.
   * @return {undefined}
   */
  tryShutdown() {
    if (rclnodejs.ok(this._handle)) {
      rclnodejs.shutdown(this._handle);
    }
  }

  handle() {
    return this._handle;
  }

  /**
   * Get the global default Context object.
   * @return {Context} - default Context
   */
  static defaultContext() {
    if (!defaultContext) {
      defaultContext = new Context();
    }
    return defaultContext;
  }

  /**
   * Shutdown the default context.
   * @return {undefined}
   */
  static shutdownDefaultContext() {
    defaultContext.shutdown();
    defaultContext = undefined;
  }
}

module.exports = Context;
