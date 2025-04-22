// Copyright (c) 2025, The Robot Web Tools Contributors
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
#include "rcl_context_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/logging.h>
#include <rcl/rcl.h>

#include <cstdio>
#include <string>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value Init(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_init_options_init(&init_options, allocator),
                           rcl_get_error_string().str);

  // Preprocess Context
  RclHandle* context_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  // Preprocess argc & argv
  Napi::Array jsArgv = info[1].As<Napi::Array>();
  int argc = jsArgv.Length();
  char** argv = nullptr;

  if (argc > 0) {
    argv = reinterpret_cast<char**>(malloc(argc * sizeof(char*)));
    for (int i = 0; i < argc; i++) {
      std::string arg = jsArgv.Get(i).As<Napi::String>().Utf8Value();
      int len = arg.length() + 1;
      argv[i] = reinterpret_cast<char*>(malloc(len * sizeof(char)));
      snprintf(argv[i], len, "%s", arg.c_str());
    }
  }

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_init(argc, argc > 0 ? argv : nullptr, &init_options, context),
      rcl_get_error_string().str);

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_logging_configure(&context->global_arguments, &allocator),
      rcl_get_error_string().str);

  for (int i = 0; i < argc; i++) {
    free(argv[i]);
  }
  free(argv);

  return env.Undefined();
}

Napi::Value Shutdown(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* context_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  THROW_ERROR_IF_NOT_EQUAL(rcl_shutdown(context), RCL_RET_OK,
                           rcl_get_error_string().str);
  THROW_ERROR_IF_NOT_EQUAL(rcl_logging_fini(), RCL_RET_OK,
                           rcl_get_error_string().str);

  return env.Undefined();
}

int DestroyContext(Napi::Env env, rcl_context_t* context) {
  rcl_ret_t ret = RCL_RET_OK;
  if (context->impl) {
    if (rcl_context_is_valid(context)) {
      if (RCL_RET_OK != rcl_shutdown(context)) {
        Napi::Error::New(env, rcl_get_error_string().str)
            .ThrowAsJavaScriptException();
      }
      ret = rcl_context_fini(context);
    }
  }
  return ret;
}

Napi::Value CreateContext(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(malloc(sizeof(rcl_context_t)));
  *context = rcl_get_zero_initialized_context();
  auto js_obj =
      RclHandle::NewInstance(env, context, nullptr, [&env](void* ptr) {
        rcl_context_t* context = reinterpret_cast<rcl_context_t*>(ptr);
        rcl_ret_t ret = DestroyContext(env, context);
        free(ptr);
        THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, ret, rcl_get_error_string().str);
      });

  return js_obj;
}

Napi::Value IsContextValid(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* context_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());
  bool is_valid = rcl_context_is_valid(context);
  return Napi::Boolean::New(env, is_valid);
}

Napi::Object InitContextBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("init", Napi::Function::New(env, Init));
  exports.Set("shutdown", Napi::Function::New(env, Shutdown));
  exports.Set("createContext", Napi::Function::New(env, CreateContext));
  exports.Set("isContextValid", Napi::Function::New(env, IsContextValid));
  return exports;
}

}  // namespace rclnodejs
