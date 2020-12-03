// Copyright (c) 2018 Intel Corporation. All rights reserved.
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
const { Clock, ROSClock } = require('./clock.js');
const { ClockType } = Clock;
const Node = require('./node.js');
const { Parameter, ParameterType } = require('./parameter.js');
const Time = require('./time.js');

const USE_SIM_TIME_PARAM = 'use_sim_time';
const CLOCK_TOPIC = '/clock';

/**
 * @class - Class representing a TimeSource in ROS
 */

class TimeSource {
  /**
   * Create a TimeSource.
   * @param {Node} node - The node to be attached.
   */
  constructor(node) {
    this._node = node;
    this._associatedClocks = [];
    this._clockSubscription = undefined;
    this._lastTimeSet = new Time(0, 0, ClockType.ROS_TIME);
    this._isRosTimeActive = false;

    if (this._node) {
      this.attachNode(this._node);
    }
  }

  get isRosTimeActive() {
    return this._isRosTimeActive;
  }

  set isRosTimeActive(enabled) {
    if (this.isRosTimeActive === enabled) return;

    this._isRosTimeActive = enabled;
    for (const clock in this._associatedClocks) {
      clock.isRosTimeActive = enabled;
    }

    if (enabled) {
      this._subscribeToClockTopic();
    } else if (this._node && this._clockSubscription) {
      this._node.destroySubscription(this._clockSubscription);
      this._node._clockSubscription = null;
    }
  }

  /**
   * Return status that whether the ROS time is active.
   * @name TimeSource#get:isRosTimeActive
   * @function
   * @return {boolean} Return true if the time is active, otherwise return false.
   */

  get isRosTimeActive() {
    return this._isRosTimeActive;
  }

  /**
   * Set the status of time.
   * @param {boolean} enabled - Set the ROS time to be active if enabled is true.
   * @name TimeSource#set:isRosTimeActive
   * @function
   * @return {undefined}
   */

  set isRosTimeActive(enabled) {
    if (this._isRosTimeActive === enabled) return;

    this._isRosTimeActive = enabled;
    this._associatedClocks.forEach((clock) => {
      clock.isRosTimeActive = enabled;
    });
    if (enabled) {
      this._subscribeToClockTopic();
    }
  }

  /**
   * Attach the clock to a Node object.
   * @param {Node} node - The node to be attached.
   * @return {undefined}
   */
  attachNode(node) {
    if (!node instanceof rclnodejs.ShadowNode) {
      throw new TypeError('Invalid argument, must be type of Node');
    }

    if (this._node) {
      this.detachNode();
    }

    this._node = node;

    if (!node.hasParameter(USE_SIM_TIME_PARAM)) {
      node.declareParameter(
        new Parameter(USE_SIM_TIME_PARAM, ParameterType.PARAMETER_BOOL, false)
      );
    }

    const useSimTimeParam = node.getParameter(USE_SIM_TIME_PARAM);
    if (useSimTimeParam.type !== ParameterType.PARAMETER_NOT_SET) {
      if (useSimTimeParam.type === ParameterType.PARAMETER_BOOL) {
        this._isRosTimeActive = useSimTimeParam.value;
      } else {
        node
          .getLogger()
          .error(
            `Invalid type for parameter ${USE_SIM_TIME_PARAM} ${useSimTimeParam.type} should be bool`
          );
      }
    } else {
      node
        .getLogger()
        .debug(
          `${USE_SIM_TIME_PARAM}' parameter not set, using wall time by default`
        );
    }

    if (this.isRosTimeActive) {
      this._subscribeToClockTopic();
    }

    node.addOnSetParametersCallback(this.onParameterEvent.bind(this));
  }

  /**
   * Detach the node which the clock have attached.
   * @return {undefined}
   */
  detachNode() {
    if (this._clockSubscription) {
      if (!this._node) {
        throw new Error(
          'Unable to destroy previously created clock subscription'
        );
      }
      this._node.destroySubscription(this._clockSubscription);
    }
    this._clockSubscription = undefined;
    this._node = undefined;
  }

  /**
   * Attach the clock to a TimeSource object.
   * @param {Clock} clock - The node to be attached.
   * @return {undefined}
   */
  attachClock(clock) {
    if (!(clock instanceof ROSClock)) {
      throw new TypeError('Only clocks with type ROS_TIME can be attached.');
    }
    clock.rosTimeOverride = this._lastTimeSet;
    clock.isRosTimeActive = this._isRosTimeActive;
    this._associatedClocks.push(clock);
  }

  _clockCallback(msg) {
    this._lastTimeSet = Time.fromMsg(msg);
    this._associatedClocks.forEach((clock) => {
      clock.rosTimeOverride = this._lastTimeSet;
    });
  }

  _subscribeToClockTopic() {
    if (!this._clockSubscription && this._node) {
      this._clockSubscription = this._node.createSubscription(
        'builtin_interfaces/msg/Time',
        CLOCK_TOPIC,
        this._clockCallback.bind(this)
      );
    }
  }

  onParameterEvent(parameters = []) {
    for (const parameter of parameters) {
      if (parameter.name === USE_SIM_TIME_PARAM) {
        if (parameter.type === ParameterType.PARAMETER_BOOL) {
          this.isRosTimeActive = parameter.value;
        } else if (this._node) {
          this._node
            .getLogger()
            .error(
              `${USE_SIM_TIME_PARAM} parameter set to something besides a bool`
            );
        }

        break;
      }
    }

    return {
      successful: true,
      reason: '',
    };
  }
}

module.exports = TimeSource;
