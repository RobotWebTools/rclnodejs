// Copyright (c) 2026, The Robot Web Tools Contributors
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

/**
 * Aggregated endpoint information for one action client or server.
 */
class ActionEndpointInfo {
  /**
   * @param {object} info - Raw endpoint information from the native layer.
   * @hideconstructor
   */
  constructor(info) {
    this._goalServiceInfo = info.goal_service_info;
    this._cancelServiceInfo = info.cancel_service_info;
    this._resultServiceInfo = info.result_service_info;
    this._feedbackTopicInfo = info.feedback_topic_info;
    this._statusTopicInfo = info.status_topic_info;
  }

  /**
   * @type {object}
   */
  get goalServiceInfo() {
    return this._goalServiceInfo;
  }

  /**
   * @type {object}
   */
  get cancelServiceInfo() {
    return this._cancelServiceInfo;
  }

  /**
   * @type {object}
   */
  get resultServiceInfo() {
    return this._resultServiceInfo;
  }

  /**
   * @type {object}
   */
  get feedbackTopicInfo() {
    return this._feedbackTopicInfo;
  }

  /**
   * @type {object}
   */
  get statusTopicInfo() {
    return this._statusTopicInfo;
  }

  /**
   * @type {string}
   */
  get nodeName() {
    return this._goalServiceInfo.node_name;
  }

  /**
   * @type {string}
   */
  get nodeNamespace() {
    return this._goalServiceInfo.node_namespace;
  }

  /**
   * @type {string}
   */
  get actionType() {
    return this._goalServiceInfo.service_type.replace(/_SendGoal$/, '');
  }

  /**
   * @type {number}
   */
  get endpointType() {
    return this._goalServiceInfo.endpoint_type;
  }
}

export default ActionEndpointInfo;
