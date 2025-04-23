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

#include "rcl_logging_bindings.h"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include <string>

#include "macros.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value setLoggerLevel(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string name = info[0].As<Napi::String>().Utf8Value();
  int level = info[1].As<Napi::Number>().Int64Value();

  rcutils_ret_t ret = rcutils_logging_set_logger_level(name.c_str(), level);
  if (ret != RCUTILS_RET_OK) {
    Napi::Error::New(env, rcutils_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcutils_reset_error();
  }
  return env.Undefined();
}

Napi::Value GetLoggerEffectiveLevel(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string name = info[0].As<Napi::String>().Utf8Value();
  int logger_level = rcutils_logging_get_logger_effective_level(name.c_str());

  if (logger_level < 0) {
    Napi::Error::New(env, rcutils_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcutils_reset_error();
    return env.Undefined();
  }
  return Napi::Number::New(env, logger_level);
}

Napi::Value Log(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string name = info[0].As<Napi::String>().Utf8Value();
  int severity = info[1].As<Napi::Number>().Int64Value();
  std::string message = info[2].As<Napi::String>().Utf8Value();
  std::string function_name = info[3].As<Napi::String>().Utf8Value();
  size_t line_number = info[4].As<Napi::Number>().Int64Value();
  std::string file_name = info[5].As<Napi::String>().Utf8Value();
  bool enabled = rcutils_logging_logger_is_enabled_for(name.c_str(), severity);

  if (enabled) {
    rcutils_log_location_t logging_location = {function_name.c_str(),
                                               file_name.c_str(), line_number};
    rcutils_log(&logging_location, severity, name.c_str(), "%s",
                message.c_str());
  }

  return Napi::Boolean::New(env, enabled);
}

Napi::Value IsEnableFor(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string name = info[0].As<Napi::String>().Utf8Value();
  int severity = info[1].As<Napi::Number>().Int64Value();
  bool enabled = rcutils_logging_logger_is_enabled_for(name.c_str(), severity);
  return Napi::Boolean::New(env, enabled);
}

Napi::Object InitLoggingBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("setLoggerLevel", Napi::Function::New(env, setLoggerLevel));
  exports.Set("getLoggerEffectiveLevel",
              Napi::Function::New(env, GetLoggerEffectiveLevel));
  exports.Set("log", Napi::Function::New(env, Log));
  exports.Set("isEnableFor", Napi::Function::New(env, IsEnableFor));
  return exports;
}

}  // namespace rclnodejs
