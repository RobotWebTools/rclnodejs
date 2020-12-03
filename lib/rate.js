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

const rclnodejs = require('../index.js');
const Context = require('./context.js');
const NodeOptions = require('./node_options.js');

const NOP_FN = () => {};

/**
 * A timer that runs at a regular frequency (hz).
 *
 * A client calls Rate#sleep() to block until the end of the current cycle.
 * This makes Rate useful for looping at a regular frequency (hz). Rate#sleep()
 * avoids blocking the JS event-loop by returning a Promise that the caller
 * should block on, e.g. use 'await rate.sleep()'.
 *
 * Note that Rate.sleep() does not prevent rclnodejs from invoking callbacks
 * such as a subscription or client if the entity's node is spinning. Thus
 * if your intent is to use rate to synchronize when callbacks are invoked
 * then use a spinOnce() just after rate.sleep() as in the example below.
 *
 * Rate runs within it's own private rcl context. This enables it to be
 * available immediately after construction. That is, unlike Timer, Rate
 * does not require a spin or spinOnce to be active.
 *
 * @example
 * async function run() {
 *   await rclnodejs.init();
 *   const node = rclnodejs.createNode('mynode');
 *   const rate = await node.createRate(1); // 1 hz
 *   while (true) {
 *     doSomeStuff();
 *     await rate.sleep();
 *     rclnodejs.spinOnce(node);
 *   }
 * }
 */
class Rate {
  /**
   * Create a new instance.
   * @hideconstructor
   * @param {number} hz - The frequency (hz) between (0.0,1000] hz,
   * @param {Timer} timer - The internal timer used by this instance.
   *   default = 1 hz
   */
  constructor(hz, timer) {
    this._hz = hz;
    this._timer = timer;
  }

  /**
   * Get the frequency in hertz (hz) of this timer.
   *
   * @returns {number} - hertz
   */
  get frequency() {
    return this._hz;
  }

  /**
   * Returns a Promise that when waited on, will block the sender
   * until the end of the current timer cycle.
   *
   * If the Rate has been cancelled, calling this method will
   * result in an error.
   *
   * @example
   * (async () => {
   *   await rate.sleep();
   * })();
   *
   * @returns {Promise} - Waiting on the promise will delay the sender
   * (not the Node event-loop) until the end of the current timer cycle.
   */
  async sleep() {
    if (this.isCanceled()) {
      throw new Error('Rate has been cancelled.');
    }

    return new Promise((resolve) => {
      this._timer.callback = () => {
        this._timer.callback = NOP_FN;
        resolve();
      };
    });
  }

  /**
   * Permanently stops the timing behavior.
   *
   * @returns {undefined}
   */
  cancel() {
    this._timer.cancel();
  }

  /**
   * Determine if this rate has been cancelled.
   *
   * @returns {boolean} - True when cancel() has been called; False otherwise.
   */
  isCanceled() {
    return this._timer.isCanceled();
  }
}

/**
 * Internal class that creates Timer instances in a common private rcl context
 * for use with Rate. The private rcl context ensures that Rate timers do not
 * deadlock waiting for spinOnce/spin on the main rcl context.
 */
class RateTimerServer {
  /**
   * Create a new instance.
   *
   * @constructor
   * @param {Node} parentNode - The parent node for which this server
   *  supplies timers to.
   */
  constructor(parentNode) {
    this._parentNode = parentNode;
    this._context = new Context();
  }

  /**
   * Setup the server's rcl context and node in preparation for creating
   * rate timer instances.
   *
   * @returns {undefined}
   */
  async init() {
    await rclnodejs.init(this._context);

    // create hidden node
    const nodeName = `_${this._parentNode.name()}_rate_timer_server`;
    const nodeNamespace = this._parentNode.namespace();
    const options = new NodeOptions();
    options.startParameterServices = false;
    options.parameterOverrides = this._parentNode.getParameters();
    options.automaticallyDeclareParametersFromOverrides = true;

    this._node = rclnodejs.createNode(
      nodeName,
      nodeNamespace,
      this._context,
      options
    );

    // spin node
    rclnodejs.spin(this._node, 10);
  }

  /**
   * Create a new timer instance with callback set to NOP.
   *
   * @param {number} period - The period in milliseconds
   * @returns {Timer} - The new timer instance.
   */
  createTimer(period) {
    const timer = this._node.createTimer(period, () => {});
    return timer;
  }

  /**
   * Permanently cancel all timers produced by this server and discontinue
   * the ability to create new Timers.
   *
   * The private rcl context is shutdown in the process and may not be
   * restarted.
   *
   * @returns {undefined}
   */
  shutdown() {
    rclnodejs.shutdown(this._context);
  }
}
// module.exports = {Rate, RateTimerServer};
module.exports = { Rate, RateTimerServer };
