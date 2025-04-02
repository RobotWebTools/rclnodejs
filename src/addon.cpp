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

#include "macros.hpp"
#include "rcl_action_bindings.hpp"
#include "rcl_bindings.hpp"
#include "rcl_handle.hpp"
#include "rcl_lifecycle_bindings.hpp"
#include "rcl_utilities.hpp"
#include "shadow_node.hpp"

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

  // Init the C++ bindings.
  rclnodejs::StoreEnv(env);
  rclnodejs::InitBindings(env, exports);
  rclnodejs::InitAction(env, exports);
  rclnodejs::InitLifecycle(env, exports);
  rclnodejs::ShadowNode::Init(env, exports);
  rclnodejs::RclHandle::Init(env, exports);

#ifdef DEBUG_ON
  int result = rcutils_logging_set_logger_level(PACKAGE_NAME,
                                                RCUTILS_LOG_SEVERITY_DEBUG);
  RCUTILS_UNUSED(result);
#endif

  return exports;
}

NODE_API_MODULE(rclnodejs, InitModule)
