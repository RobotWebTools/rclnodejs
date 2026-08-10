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
    this.goalServiceInfo = info.goal_service_info;
    this.cancelServiceInfo = info.cancel_service_info;
    this.resultServiceInfo = info.result_service_info;
    this.feedbackTopicInfo = info.feedback_topic_info;
    this.statusTopicInfo = info.status_topic_info;
  }

  get nodeName() {
    return this.goalServiceInfo.node_name;
  }

  get nodeNamespace() {
    return this.goalServiceInfo.node_namespace;
  }

  get actionType() {
    return this.goalServiceInfo.service_type.replace(/_SendGoal$/, '');
  }

  get endpointType() {
    return this.goalServiceInfo.endpoint_type;
  }
}

export default ActionEndpointInfo;
