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

#include "rcl_bindings.h"

#include <node.h>
#include <rcl/arguments.h>
#include <rcl/rcl.h>

#include <rcpputils/scope_exit.hpp>

#if ROS_VERSION >= 2006
#include <rosidl_runtime_c/string_functions.h>
#else
#include <rosidl_generator_c/string_functions.h>
#endif

#include <memory>
#include <string>
#include <vector>

#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value RemoveROSArgs(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  Napi::Array jsArgv = info[0].As<Napi::Array>();
  size_t argc = jsArgv.Length();
  char** argv = AbstractArgsFromNapiArray(jsArgv);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_arguments_t parsed_args = rcl_get_zero_initialized_arguments();

  rcl_ret_t ret = rcl_parse_arguments(argc, const_cast<const char**>(argv),
                                      allocator, &parsed_args);

  if (RCL_RET_OK != ret) {
    FreeArgs(argv, argc);
    Napi::Error::New(env, "Failed to parse arguments")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  RCPPUTILS_SCOPE_EXIT({
    rcl_ret_t fini_ret = rcl_arguments_fini(&parsed_args);
    if (RCL_RET_OK != fini_ret) {
      // Log error but don't throw since we might be already throwing
    }
  });

  int nonros_argc = 0;
  const char** nonros_argv = nullptr;

  ret = rcl_remove_ros_arguments(const_cast<const char**>(argv), &parsed_args,
                                 allocator, &nonros_argc, &nonros_argv);

  if (RCL_RET_OK != ret) {
    FreeArgs(argv, argc);
    Napi::Error::New(env, "Failed to remove ROS arguments")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  RCPPUTILS_SCOPE_EXIT({ allocator.deallocate(nonros_argv, allocator.state); });

  Napi::Array result = Napi::Array::New(env, nonros_argc);
  for (int i = 0; i < nonros_argc; ++i) {
    result.Set(i, Napi::String::New(env, nonros_argv[i]));
  }

  FreeArgs(argv, argc);

  return result;
}

Napi::Value InitString(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  void* buffer = info[0].As<Napi::Buffer<char>>().Data();
#if ROS_VERSION >= 2006
  rosidl_runtime_c__String* ptr =
      reinterpret_cast<rosidl_runtime_c__String*>(buffer);

  rosidl_runtime_c__String__init(ptr);
#else
  rosidl_generator_c__String* ptr =
      reinterpret_cast<rosidl_generator_c__String*>(buffer);

  rosidl_generator_c__String__init(ptr);
#endif
  return env.Undefined();
}

inline char* GetBufAddr(Napi::Value buf) {
  return buf.As<Napi::Buffer<char>>().Data();
}

Napi::Value FreeMemeoryAtOffset(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string addr_str = info[0].As<Napi::String>().Utf8Value();
  int64_t result = std::stoull(addr_str, 0, 16);
  char* addr = reinterpret_cast<char*>(result);
  int64_t offset =
      info[1].IsNumber() ? info[1].As<Napi::Number>().Int64Value() : 0;
  auto ptr = addr + offset;

  char* val = *reinterpret_cast<char**>(ptr);
  free(val);
  return env.Undefined();
}

Napi::Value CreateArrayBufferFromAddress(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string addr_str = info[0].As<Napi::String>().Utf8Value();
  int64_t result = std::stoull(addr_str, 0, 16);
  char* addr = reinterpret_cast<char*>(result);
  int32_t length = info[1].As<Napi::Number>().Int32Value();

  static_assert(NODE_MAJOR_VERSION > 12, "nodejs version must > 12");
#if (NODE_RUNTIME_ELECTRON && NODE_MODULE_VERSION >= 109)
  // Because V8 sandboxed pointers was enabled since Electron 21, we have to
  // make a deep copy for Electron 21 and up.
  // See more details: https://www.electronjs.org/blog/v8-memory-cage
  Napi::ArrayBuffer array_buffer = Napi::ArrayBuffer::New(env, length);
  memcpy(array_buffer.Data(), addr, length);
  free(addr);
#else
  // For nodejs > 12 or electron < 21, we will take over the ownership of
  // `addr`.
  auto array_buffer = Napi::ArrayBuffer::New(
      env, addr, length, [](Napi::Env /*env*/, void* data) { free(data); });
#endif

  return array_buffer;
}

Napi::Value CreateArrayBufferCleaner(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  auto address = GetBufAddr(info[0]);
  int32_t offset = info[1].As<Napi::Number>().Int32Value();

  char* target = *reinterpret_cast<char**>(address + offset);
  return RclHandle::NewInstance(env, target, nullptr,
                                [](void* ptr) { free(ptr); });
}

Napi::Object InitBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("initString", Napi::Function::New(env, InitString));
  exports.Set("freeMemeoryAtOffset",
              Napi::Function::New(env, FreeMemeoryAtOffset));
  exports.Set("createArrayBufferFromAddress",
              Napi::Function::New(env, CreateArrayBufferFromAddress));
  exports.Set("createArrayBufferCleaner",
              Napi::Function::New(env, CreateArrayBufferCleaner));
  exports.Set("removeROSArgs", Napi::Function::New(env, RemoveROSArgs));
  return exports;
}

}  // namespace rclnodejs
