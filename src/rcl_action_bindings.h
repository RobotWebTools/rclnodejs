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

#ifndef SRC_RCL_ACTION_BINDINGS_H_
#define SRC_RCL_ACTION_BINDINGS_H_

#include <napi.h>
#include <rcl_action/rcl_action.h>

#include <memory>
#include <string>
#include <vector>

#include "rcl_bindings.h"

namespace rclnodejs {

Napi::Value ActionCreateClient(const Napi::CallbackInfo& info);
Napi::Value ActionCreateServer(const Napi::CallbackInfo& info);
Napi::Value ActionServerIsAvailable(const Napi::CallbackInfo& info);
Napi::Value ActionSendGoalRequest(const Napi::CallbackInfo& info);
Napi::Value ActionTakeGoalRequest(const Napi::CallbackInfo& info);
Napi::Value ActionSendGoalResponse(const Napi::CallbackInfo& info);
Napi::Value ActionTakeGoalResponse(const Napi::CallbackInfo& info);
Napi::Value ActionSendCancelRequest(const Napi::CallbackInfo& info);
Napi::Value ActionTakeCancelRequest(const Napi::CallbackInfo& info);
Napi::Value ActionSendCancelResponse(const Napi::CallbackInfo& info);
Napi::Value ActionTakeCancelResponse(const Napi::CallbackInfo& info);
Napi::Value ActionSendResultRequest(const Napi::CallbackInfo& info);
Napi::Value ActionTakeResultRequest(const Napi::CallbackInfo& info);
Napi::Value ActionSendResultResponse(const Napi::CallbackInfo& info);
Napi::Value ActionTakeResultResponse(const Napi::CallbackInfo& info);
Napi::Value ActionAcceptNewGoal(const Napi::CallbackInfo& info);
Napi::Value ActionUpdateGoalState(const Napi::CallbackInfo& info);
Napi::Value ActionPublishStatus(const Napi::CallbackInfo& info);
Napi::Value ActionTakeStatus(const Napi::CallbackInfo& info);
Napi::Value ActionGoalHandleIsActive(const Napi::CallbackInfo& info);
Napi::Value ActionNotifyGoalDone(const Napi::CallbackInfo& info);
Napi::Value ActionGoalHandleGetStatus(const Napi::CallbackInfo& info);
Napi::Value ActionPublishFeedback(const Napi::CallbackInfo& info);
Napi::Value ActionTakeFeedback(const Napi::CallbackInfo& info);
Napi::Value ActionProcessCancelRequest(const Napi::CallbackInfo& info);
Napi::Value ActionServerGoalExists(const Napi::CallbackInfo& info);
Napi::Value ActionExpireGoals(const Napi::CallbackInfo& info);
Napi::Value ActionGetClientNamesAndTypesByNode(const Napi::CallbackInfo& info);
Napi::Value ActionGetServerNamesAndTypesByNode(const Napi::CallbackInfo& info);
Napi::Value ActionGetNamesAndTypes(const Napi::CallbackInfo& info);

Napi::Object InitAction(Napi::Env env, Napi::Object exports);

}  // namespace rclnodejs

#endif  // SRC_RCL_ACTION_BINDINGS_H_
