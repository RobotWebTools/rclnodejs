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

#include <node_api.h>
#include <rcutils/logging.h>

#include "macros.h"
#include "rcl_action_client_bindings.h"
#include "rcl_action_goal_bindings.h"
#include "rcl_action_server_bindings.h"
#include "rcl_bindings.h"
#include "rcl_client_bindings.h"
#include "rcl_context_bindings.h"
#include "rcl_graph_bindings.h"
#include "rcl_guard_condition_bindings.h"
#include "rcl_handle.h"
#include "rcl_lifecycle_bindings.h"
#include "rcl_logging_bindings.h"
#include "rcl_names_bindings.h"
#include "rcl_node_bindings.h"
#include "rcl_publisher_bindings.h"
#include "rcl_serialization_bindings.h"
#include "rcl_service_bindings.h"
#include "rcl_subscription_bindings.h"
#include "rcl_time_point_bindings.h"
#include "rcl_timer_bindings.h"
#if ROS_VERSION > 2205  // ROS2 > Humble
#include "rcl_event_handle_bindings.h"
#include "rcl_type_description_service_bindings.h"
#endif
#include "rcl_utilities.h"
#include "ref_napi_bindings.h"
#include "shadow_node.h"

bool IsRunningInElectronRenderer(const Napi::Env& env) {
  Napi::Object global = env.Global();
  Napi::Object process = global.Get("process").As<Napi::Object>();
  Napi::Value processType = process.Get("type");
  return processType.StrictEquals(Napi::String::New(env, "renderer"));
}

Napi::Object InitModule(Napi::Env env, Napi::Object exports) {
// workaround process name mangling by chromium
//
// rcl logging uses `program_invocation_name` to determine the log file,
// chromium mangles the program name to include all args, this causes a
// ENAMETOOLONG error when starting ros. Workaround is to replace the first
// occurence of ' -' with the null terminator. see:
// https://unix.stackexchange.com/questions/432419/unexpected-non-null-encoding-of-proc-pid-cmdline
#if defined(__linux__) && defined(__GLIBC__)
  if (IsRunningInElectronRenderer(env)) {
    auto prog_name = program_invocation_name;
    auto end = strstr(prog_name, " -");
    assert(end);
    prog_name[end - prog_name] = 0;
  }
#endif

  rclnodejs::StoreEnv(env);
  // Init the C++ bindings.
  rclnodejs::InitBindings(env, exports);
  rclnodejs::InitActionClientBindings(env, exports);
  rclnodejs::InitActionGoalBindings(env, exports);
  rclnodejs::InitActionServerBindings(env, exports);
  rclnodejs::InitClientBindings(env, exports);
  rclnodejs::InitContextBindings(env, exports);
  rclnodejs::InitGraphBindings(env, exports);
  rclnodejs::InitGuardConditionBindings(env, exports);
  rclnodejs::InitLoggingBindings(env, exports);
  rclnodejs::InitNamesBindings(env, exports);
  rclnodejs::InitNodeBindings(env, exports);
  rclnodejs::InitPublisherBindings(env, exports);
  rclnodejs::InitServiceBindings(env, exports);
  rclnodejs::InitSubscriptionBindings(env, exports);
  rclnodejs::InitTimePointBindings(env, exports);
  rclnodejs::InitTimerBindings(env, exports);
#if ROS_VERSION > 2205  // ROS2 > Humble
  rclnodejs::InitTypeDescriptionServiceBindings(env, exports);
  rclnodejs::InitEventHandleBindings(env, exports);
#endif
  rclnodejs::InitLifecycleBindings(env, exports);
  rclnodejs::InitSerializationBindings(env, exports);
  rclnodejs::ShadowNode::Init(env, exports);
  rclnodejs::RclHandle::Init(env, exports);

  exports.Set("ref", rclnodejs::InitRefNapi(env));

#ifdef DEBUG_ON
  int result = rcutils_logging_set_logger_level(PACKAGE_NAME,
                                                RCUTILS_LOG_SEVERITY_DEBUG);
  RCUTILS_UNUSED(result);
#endif

  return exports;
}

NODE_API_MODULE(rclnodejs, InitModule)
