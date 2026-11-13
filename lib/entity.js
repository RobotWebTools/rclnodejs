// Copyright (c) 2017 Intel Corporation. All rights reserved.
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

// Destroying an entity for which there is immediate i/o activity can
// results in a SEGFAULT. To mitigate this situation we hold a
// reference to released handles to prevent them from being
// GC'ed prematurely.
const OBSOLETE_HANDLES = new Set();
let cleanupScheduled = false;
function retainHandleUntilCleanup(handle) {
  OBSOLETE_HANDLES.add(handle);
  if (cleanupScheduled) return;

  cleanupScheduled = true;
  setImmediate(() => {
    cleanupScheduled = false;
    Entity._gcHandles();
  }).unref();
}

/**
 * @class - Class representing a common object in RCL.
 * @ignore
 */

class Entity {
  /**
   * Clears the internal short-lived cache of references to
   * destroyed entities.
   *
   * @ignore
   */
  static _gcHandles() {
    OBSOLETE_HANDLES.clear();
  }

  constructor(handle, typeClass, options) {
    this._handle = handle;
    this._typeClass = typeClass;
    this._options = options;
    this._destroyed = false;
  }

  get handle() {
    return this._handle;
  }

  get options() {
    return this._options;
  }

  set options(options) {
    this._options = options;
  }

  get qos() {
    return this._options.qos;
  }

  get typedArrayEnabled() {
    return this._options.enableTypedArray;
  }

  get typeClass() {
    return this._typeClass;
  }

  /**
   * Release all resources held by this entity.
   * Called by EntityRegistry after removal; not a user-facing destruction API.
   * @ignore
   */
  releaseResources() {
    if (this.isDestroyed()) return;

    this._destroyed = true;
    this.handle.release();
    retainHandleUntilCleanup(this._handle);
  }

  /**
   * Test if this entity has been destroyed and resources released.
   * @returns {boolean} - true when destroyed has previously been called.
   */
  isDestroyed() {
    return this._destroyed;
  }
}

export default Entity;
