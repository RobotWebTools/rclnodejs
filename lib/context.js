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

let defaultContext = null;

/**
 * @class - Class representing a Context in ROS
 * @hideconstructor
 */
class Context {
  constructor() {
    this._handle = rclnodejs.createContext();
  }

  /**
   * Shut down the context.
   * @return {undefined}
   * @throws {Error} If there is a problem shutting down the context.
   */
  shutdown() {
    rclnodejs.shutdown(this.handle);
    if (this.isDefaultContext) {
      defaultContext = null;
    }
  }

  /**
   * Try to shut down the context.
   * @return {undefined}
   * @throws {Error} If there is a problem shutting down the context.
   */
  tryShutdown() {
    if (this.isOk) {
      this.shutdown();
    }
  }

  /**
   * Get the handle referencing the internal context object. Do not modify it yourself: only pass it to *rclnodejs* functions!
   * @return {undefined} a reference to the internal context object
   */
  get handle() {
    return this._handle;
  }

  /**
   * Check if this context is the default one.
   * @return {boolean}
   */
  get isDefaultContext() {
    return this === defaultContext
  }

  /**
   * Check that the context is valid.
   * @return {boolean}
   */
  get isOk() {
    return rclnodejs.ok(this.handle)
  }

  /**
   * Get the global default Context object.
   * @return {Context} - default Context
   */
  static defaultContext() {
    if (defaultContext === null) {
      defaultContext = new Context();
    }
    return defaultContext;
  }
}

module.exports = Context;
