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

#include <nan.h>

#include "macros.hpp"
#include "rcl_action_bindings.hpp"
#include "rcl_bindings.hpp"
#include "rcl_handle.hpp"
#include "rcl_lifecycle_bindings.hpp"
#include "rcutils/logging.h"
#include "rcutils/macros.h"
#include "shadow_node.hpp"

bool IsRunningInElectronRenderer() {
  auto global = Nan::GetCurrentContext()->Global();
  auto process =
      Nan::To<v8::Object>(Nan::Get(global, Nan::New("process").ToLocalChecked())
                              .ToLocalChecked())
          .ToLocalChecked();
  auto process_type =
      Nan::Get(process, Nan::New("type").ToLocalChecked()).ToLocalChecked();
  return process_type->StrictEquals(Nan::New("renderer").ToLocalChecked());
}

void InitModule(v8::Local<v8::Object> exports) {
// workaround process name mangling by chromium
//
// rcl logging uses `program_invocation_name` to determine the log file,
// chromium mangles the program name to include all args, this causes a
// ENAMETOOLONG error when starting ros. Workaround is to replace the first
// occurence of ' -' with the null terminator. see:
// https://unix.stackexchange.com/questions/432419/unexpected-non-null-encoding-of-proc-pid-cmdline
#if defined(__linux__) && defined(__GLIBC__)
  if (IsRunningInElectronRenderer()) {
    auto prog_name = program_invocation_name;
    auto end = strstr(prog_name, " -");
    assert(end);
    prog_name[end - prog_name] = 0;
  }
#endif

  v8::Local<v8::Context> context = exports->GetIsolate()->GetCurrentContext();

  for (uint32_t i = 0; i < rclnodejs::binding_methods.size(); i++) {
    Nan::Set(
        exports, Nan::New(rclnodejs::binding_methods[i].name).ToLocalChecked(),
        Nan::New<v8::FunctionTemplate>(rclnodejs::binding_methods[i].function)
            ->GetFunction(context)
            .ToLocalChecked());
  }

  for (uint32_t i = 0; i < rclnodejs::action_binding_methods.size(); i++) {
    Nan::Set(
        exports,
        Nan::New(rclnodejs::action_binding_methods[i].name).ToLocalChecked(),
        Nan::New<v8::FunctionTemplate>(
            rclnodejs::action_binding_methods[i].function)
            ->GetFunction(context)
            .ToLocalChecked());
  }

  for (uint32_t i = 0; i < rclnodejs::lifecycle_binding_methods.size(); i++) {
    Nan::Set(
        exports,
        Nan::New(rclnodejs::lifecycle_binding_methods[i].name).ToLocalChecked(),
        Nan::New<v8::FunctionTemplate>(
            rclnodejs::lifecycle_binding_methods[i].function)
            ->GetFunction(context)
            .ToLocalChecked());
  }

  rclnodejs::ShadowNode::Init(exports);
  rclnodejs::RclHandle::Init(exports);

#ifdef DEBUG_ON
  int result = rcutils_logging_set_logger_level(PACKAGE_NAME,
                                                RCUTILS_LOG_SEVERITY_DEBUG);
  RCUTILS_UNUSED(result);
#endif
}

NODE_MODULE(rclnodejs, InitModule);
