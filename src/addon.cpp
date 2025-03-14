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

#include <napi.h>
#include <node_api.h>

#include "macros.hpp"
#include "rcl_action_bindings.hpp"
#include "rcl_bindings.hpp"
#include "rcl_handle.hpp"
#include "rcl_lifecycle_bindings.hpp"
#include "rcutils/logging.h"
#include "rcutils/macros.h"
#include "shadow_node.hpp"

bool IsRunningInElectronRenderer(napi_env env) {
  napi_value global, process, process_type;
  napi_get_global(env, &global);
  napi_get_named_property(env, global, "process", &process);
  napi_get_named_property(env, process, "type", &process_type);

  bool is_renderer;
  napi_value renderer_str;
  napi_create_string_utf8(env, "renderer", NAPI_AUTO_LENGTH, &renderer_str);
  napi_strict_equals(env, process_type, renderer_str, &is_renderer);
  return is_renderer;
}

napi_value Init(napi_env env, napi_value exports) {
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

  napi_value context;
  napi_get_named_property(env, exports, "context", &context);

  for (uint32_t i = 0; i < rclnodejs::binding_methods.size(); i++) {
    napi_value func;
    napi_create_function(env, NULL, 0, rclnodejs::binding_methods[i].function, NULL, &func);
    napi_set_named_property(env, exports, rclnodejs::binding_methods[i].name, func);
  }

  for (uint32_t i = 0; i < rclnodejs::action_binding_methods.size(); i++) {
    napi_value func;
    napi_create_function(env, NULL, 0, rclnodejs::action_binding_methods[i].function, NULL, &func);
    napi_set_named_property(env, exports, rclnodejs::action_binding_methods[i].name, func);
  }

  for (uint32_t i = 0; i < rclnodejs::lifecycle_binding_methods.size(); i++) {
    napi_value func;
    napi_create_function(env, NULL, 0, rclnodejs::lifecycle_binding_methods[i].function, NULL, &func);
    napi_set_named_property(env, exports, rclnodejs::lifecycle_binding_methods[i].name, func);
  }

  rclnodejs::ShadowNode::Init(exports);
  rclnodejs::RclHandle::Init(exports);

#ifdef DEBUG_ON
  int result = rcutils_logging_set_logger_level(PACKAGE_NAME,
                                                RCUTILS_LOG_SEVERITY_DEBUG);
  RCUTILS_UNUSED(result);
#endif

  return exports;
}

NAPI_MODULE(NODE_GYP_MODULE_NAME, Init)
