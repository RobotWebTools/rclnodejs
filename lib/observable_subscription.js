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

'use strict';

const { Subject } = require('rxjs');

/**
 * A wrapper that provides RxJS Observable support for ROS 2 subscriptions.
 * This class wraps a standard Subscription and emits messages through an Observable.
 *
 * @class ObservableSubscription
 * @hideconstructor
 */
class ObservableSubscription {
  #subscription;
  #subject;
  #destroyed;

  /**
   * Create an ObservableSubscription wrapper.
   * @param {Subscription} subscription - The underlying ROS 2 subscription
   */
  constructor(subscription) {
    this.#subscription = subscription;
    this.#subject = new Subject();
    this.#destroyed = false;
  }

  /**
   * Get the RxJS Observable for this subscription.
   * Use this to pipe operators and subscribe to messages.
   * @type {Observable}
   */
  get observable() {
    return this.#subject.asObservable();
  }

  /**
   * Get the underlying ROS 2 subscription.
   * @type {Subscription}
   */
  get subscription() {
    return this.#subscription;
  }

  /**
   * Get the topic name.
   * @type {string}
   */
  get topic() {
    return this.#subscription.topic;
  }

  /**
   * Check if this observable subscription has been destroyed.
   * @type {boolean}
   */
  get isDestroyed() {
    return this.#destroyed;
  }

  /**
   * Internal method to emit a message to subscribers.
   * Called by the subscription's processResponse.
   * @private
   * @param {any} message - The message to emit
   */
  _emit(message) {
    if (!this.#destroyed) {
      this.#subject.next(message);
    }
  }

  /**
   * Complete the observable and clean up resources.
   * After calling this, no more messages will be emitted.
   */
  complete() {
    if (!this.#destroyed) {
      this.#destroyed = true;
      this.#subject.complete();
    }
  }

  /**
   * Alias for complete() for consistency with RxJS naming.
   */
  destroy() {
    this.complete();
  }
}

module.exports = ObservableSubscription;
