// Copyright (c) 2026 RobotWebTools Contributors. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

/**
 * Declarative allow-list of ROS 2 capabilities exposed to the Web Runtime.
 *
 * The registry is the single source of truth for "what may a connected client
 * do?" — every dispatched frame is checked against it. Capabilities not listed
 * here are rejected at the runtime layer before any rclnodejs Node API is
 * touched.
 *
 * Each capability is a tuple `(kind, name, type)` where:
 *   - `kind` is one of `'call' | 'publish' | 'subscribe'`
 *   - `name` is the ROS 2 service or topic name (e.g. `'/cmd_vel'`)
 *   - `type` is the ROS 2 interface name (e.g. `'std_msgs/msg/String'`)
 */
class CapabilityRegistry {
  constructor() {
    // Per-kind allow-lists. Each map is keyed by ROS name and stores the
    // resolved interface type, e.g. `_call.get('/add_two_ints')` returns
    // `'example_interfaces/srv/AddTwoInts'`. Looked up at dispatch time
    // by Dispatcher.resolve(kind, name).
    this._call = new Map(); //      ROS service name -> srv type
    this._publish = new Map(); //   ROS topic name   -> msg type
    this._subscribe = new Map(); // ROS topic name   -> msg type
  }

  /**
   * Register one or more capabilities.
   *
   * @example Shorthand (string value = type name)
   *   registry.expose({
   *     call:      { '/add':    'example_interfaces/srv/AddTwoInts' },
   *     publish:   { '/chatter':'std_msgs/msg/String' },
   *     subscribe: { '/scan':   'sensor_msgs/msg/LaserScan' },
   *   });
   *
   * @example Rich form (object value with metadata)
   *   registry.expose({
   *     subscribe: {
   *       '/scan': { type: 'sensor_msgs/msg/LaserScan' /* future: qos, keep_last, ... *\/ },
   *     },
   *   });
   *
   * The rich form is accepted today but the runtime only consumes
   * `type`; additional fields are reserved for forward-compatibility
   * (e.g. future QoS metadata). Snapshots via {@link list} always
   * return the canonical `{ name: typeName }` form regardless of
   * which form was used here.
   *
   * @param {{
   *   call?:      Object<string, string | {type:string}>,
   *   publish?:   Object<string, string | {type:string}>,
   *   subscribe?: Object<string, string | {type:string}>,
   * }} spec
   * @returns {CapabilityRegistry} this (chainable)
   */
  expose(spec = {}) {
    for (const [name, value] of Object.entries(spec.call || {})) {
      this._call.set(name, _typeOf(value, 'call', name));
    }
    for (const [name, value] of Object.entries(spec.publish || {})) {
      this._publish.set(name, _typeOf(value, 'publish', name));
    }
    for (const [name, value] of Object.entries(spec.subscribe || {})) {
      this._subscribe.set(name, _typeOf(value, 'subscribe', name));
    }
    return this;
  }

  /**
   * Resolve a capability lookup.
   * @param {'call'|'publish'|'subscribe'} kind
   * @param {string} name
   * @returns {{kind:string, name:string, type:string}|null}
   */
  resolve(kind, name) {
    const map = this._mapFor(kind);
    if (!map) return null;
    const type = map.get(name);
    return type ? { kind, name, type } : null;
  }

  /**
   * Snapshot the registered capabilities as a plain object.
   *
   * Always returns the canonical shorthand form
   * (`{ [name]: typeName }`) regardless of how `expose()` was called.
   * Useful for introspection and future OpenAPI export.
   */
  list() {
    return {
      call: Object.fromEntries(this._call),
      publish: Object.fromEntries(this._publish),
      subscribe: Object.fromEntries(this._subscribe),
    };
  }

  _mapFor(kind) {
    if (kind === 'call') return this._call;
    if (kind === 'publish') return this._publish;
    if (kind === 'subscribe') return this._subscribe;
    return null;
  }
}

function _typeOf(value, kind, name) {
  if (typeof value === 'string') {
    if (!value) {
      throw new TypeError(
        `expose.${kind}["${name}"] is an empty string; expected a ROS type name`
      );
    }
    return value;
  }
  if (value && typeof value === 'object' && typeof value.type === 'string') {
    if (!value.type) {
      throw new TypeError(
        `expose.${kind}["${name}"].type is empty; expected a ROS type name`
      );
    }
    return value.type;
  }
  throw new TypeError(
    `expose.${kind}["${name}"]: expected a string or { type: string }`
  );
}

export { CapabilityRegistry };
