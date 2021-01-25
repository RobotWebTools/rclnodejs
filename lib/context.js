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
 * Encapsulates the lifecycle of an rcl environment from init to shutdown.
 * A Context serves as a container for a ROS2 RCL environment that holds
 * nodes and the resources created by the nodes, e.g.,
 * publishers, subscriptions, actions, services...v
 *
 * A context has 3 states:
 * ```
 * new Context() --> uninitialized -->
 * *                                  |
 *         ---------------------------
 *        |
 *        v
 * rcl.init(context) --> initialized ->
 *                                     |
 *         ----------------------------
 *        |
 *        v
 * rcl.shutdown(context)
 *        or
 * context.shutdown() ---> shutdown
 * ```
 * Must call rclnodejs.init(context) to initialize the context
 * to the usable 'initialized' (valid) state be using.
 */
class Context {
  /**
   * Access the list of usable (initialized/valid) contexts.
   * @returns {Context[]} Array of valid contexts
   */
  static get instances() {
    let contexts = [];
    for (const ctx of Context._instances) {
      if (ctx.isValid()) {
        contexts.push(ctx);
      }
    }
    return contexts;
  }

  /**
   * Create a new instance in uninitialized state.
   * Call rcl.init(context) to initialize this context state for
   * use in creating nodes, etc.
   * @constructor
   */
  constructor() {
    this._handle = rclnodejs.createContext();
    this._isShutdown = false;
    this._nodes = [];
    Context._instances.push(this);
  }

  /**
   * Access the nodes managed by this context.
   * @returns {Node[]} The nodes.
   */
  get nodes() {
    return Array.from(this._nodes);
  }

  /**
   * Get the handle referencing the internal context object. Do not modify it yourself: only pass it to *rclnodejs* functions!
   * @returns {undefined} a reference to the internal context object
   */
  get handle() {
    return this._handle;
  }

  /**
   * Test if this context has not been initialized by rcl.init(context).
   * @returns {boolean} True if context has been initialized; otherwise false
   */
  isUninitialized() {
    return !this.isShutdown() && !this.isValid();
  }

  /**
   * Test if this context has been initialized, i.e., rcl.init(context),
   * and not shutdown.
   * @returns {boolean} True if context has been initialized; otherwise false
   */
  isInitialized() {
    return !this.isShutdown() && this.isValid();
  }

  /**
   * Test if this context has been shutdown, i.e., context.shutdown().
   * @returns {boolean} True if context has been shutdown; otherwise false
   */
  isShutdown() {
    return this._isShutdown;
  }

  /**
   * Test if this context is the default one.
   * @returns {boolean} whether this is the default context
   */
  isDefaultContext() {
    return this === defaultContext;
  }

  /**
   * Check that the context is in a usable state, i.e., it
   * has been initialized and not yet shutdown.
   * @returns {boolean} whether this context is (still) valid
   */
  isValid() {
    return rclnodejs.isContextValid(this.handle);
  }

  /**
   * Check that the context is valid.
   * @returns {boolean} whether this context is (still) valid
   *
   * @deprecated since 0.18.0, Use Context.isValid()
   */
  get isOk() {
    return this.isValid();
  }

  /**
   * Shut down the context including destroying all nodes.
   * @returns {undefined}
   * @throws {Error} If there is a problem shutting down the context.
   */
  shutdown() {
    if (this.isShutdown()) return;

    // shutdown and remove all nodes
    for (const node of this.nodes) {
      node.destroy();
    }

    if (this.isInitialized()) {
      rclnodejs.shutdown(this.handle);
    }

    this._isShutdown = true;

    // remove context from _instances[]
    const index = Context._instances.indexOf(this);
    if (index > -1) {
      Context._instances.splice(index, 1);
    }

    if (this.isDefaultContext()) {
      defaultContext = null;
    }
  }

  /**
   * Try to shut down the context.
   * @returns {undefined}
   * @throws {Error} If there is a problem shutting down the context.
   */
  tryShutdown() {
    if (this.isInitialized()) {
      this.shutdown();
    }
  }

  onNodeCreated(node) {
    if (!node) {
      throw new Error('Node must be defined to add to Context');
    }

    if (this.isShutdown()) {
      throw new Error('Can not add a Node to a Context that is shutdown');
    }

    if (this._nodes.includes(node)) {
      // do nothing
      return;
    }

    this._nodes.push(node);
  }

  onNodeDestroyed(node) {
    if (!this._nodes) {
      return;
    }

    // remove node from _nodes[]
    const index = this._nodes.indexOf(node);
    if (index > -1) {
      this._nodes.splice(index, 1);
    }
  }

  /**
   * Get the global default Context object.
   * @returns {Context} The default Context
   */
  static defaultContext() {
    if (defaultContext === null) {
      defaultContext = new Context();
    }
    return defaultContext;
  }
}

Context._instances = [];

module.exports = Context;
