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
#include <rcl/logging.h>
#include <rcl/logging_rosout.h>
#include <rcl/rcl.h>
#include <rcl_logging_interface/rcl_logging_interface.h>

#include <mutex>
#include <string>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

Napi::Value InitRosoutPublisherForNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  if (rcl_logging_rosout_enabled()) {
    rcl_ret_t ret = rcl_logging_rosout_init_publisher_for_node(node);
    if (ret != RCL_RET_OK) {
      Napi::Error::New(env, rcl_get_error_string().str)
          .ThrowAsJavaScriptException();
      rcl_reset_error();
    }
  }
  return env.Undefined();
}

static std::recursive_mutex logging_mutex;

static void rclnodejs_thread_safe_logging_output_handler(
    const rcutils_log_location_t* location, int severity, const char* name,
    rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
  try {
    std::lock_guard<std::recursive_mutex> lock(logging_mutex);
    rcl_logging_multiple_output_handler(location, severity, name, timestamp,
                                        format, args);
  } catch (const std::exception& ex) {
    RCUTILS_SAFE_FWRITE_TO_STDERR(
        "rclnodejs failed to get the global logging mutex: ");
    RCUTILS_SAFE_FWRITE_TO_STDERR(ex.what());
    RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
  } catch (...) {
    RCUTILS_SAFE_FWRITE_TO_STDERR(
        "rclnodejs failed to get the global logging mutex\n");
  }
}

Napi::Value LoggingConfigure(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* context_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  rcl_allocator_t allocator = rcl_get_default_allocator();

  std::lock_guard<std::recursive_mutex> lock(logging_mutex);
  rcl_ret_t ret = rcl_logging_configure_with_output_handler(
      &context->global_arguments, &allocator,
      rclnodejs_thread_safe_logging_output_handler);

  if (ret != RCL_RET_OK) {
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcl_reset_error();
  }
  return env.Undefined();
}

Napi::Value FiniRosoutPublisherForNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  if (rcl_logging_rosout_enabled()) {
    rcl_ret_t ret = rcl_logging_rosout_fini_publisher_for_node(node);
    if (ret != RCL_RET_OK) {
      Napi::Error::New(env, rcl_get_error_string().str)
          .ThrowAsJavaScriptException();
      rcl_reset_error();
    }
  }
  return env.Undefined();
}

Napi::Value LoggingFini(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  std::lock_guard<std::recursive_mutex> lock(logging_mutex);
  rcl_ret_t ret = rcl_logging_fini();
  if (ret != RCL_RET_OK) {
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcl_reset_error();
  }
  return env.Undefined();
}

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

Napi::Value GetLoggingDirectory(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  char* directory_path = nullptr;
  rcl_logging_ret_t ret =
      rcl_logging_get_logging_directory(allocator, &directory_path);

  if (ret != RCL_LOGGING_RET_OK) {
    Napi::Error::New(env, rcutils_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcutils_reset_error();
    return env.Undefined();
  }

  Napi::String result = Napi::String::New(env, directory_path);
  allocator.deallocate(directory_path, allocator.state);
  return result;
}

#if ROS_VERSION > 2205
Napi::Value AddRosoutSublogger(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  std::string logger_name = info[0].As<Napi::String>().Utf8Value();
  std::string sublogger_name = info[1].As<Napi::String>().Utf8Value();

  rcl_ret_t ret = rcl_logging_rosout_add_sublogger(logger_name.c_str(),
                                                   sublogger_name.c_str());
  if (ret == RCL_RET_OK) {
    return Napi::Boolean::New(env, true);
  } else if (ret == RCL_RET_NOT_FOUND) {
    rcl_reset_error();
    return Napi::Boolean::New(env, false);
  } else {
    Napi::Error::New(env, rcl_get_error_string().str)
        .ThrowAsJavaScriptException();
    rcl_reset_error();
    return env.Undefined();
  }
}

Napi::Value RemoveRosoutSublogger(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  std::string logger_name = info[0].As<Napi::String>().Utf8Value();
  std::string sublogger_name = info[1].As<Napi::String>().Utf8Value();

  rcl_ret_t ret = rcl_logging_rosout_remove_sublogger(logger_name.c_str(),
                                                      sublogger_name.c_str());
  if (ret != RCL_RET_OK) {
    rcl_reset_error();
  }
  return env.Undefined();
}
#endif

Napi::Object InitLoggingBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("setLoggerLevel", Napi::Function::New(env, setLoggerLevel));
  exports.Set("getLoggerEffectiveLevel",
              Napi::Function::New(env, GetLoggerEffectiveLevel));
  exports.Set("log", Napi::Function::New(env, Log));
  exports.Set("isEnableFor", Napi::Function::New(env, IsEnableFor));
  exports.Set("getLoggingDirectory",
              Napi::Function::New(env, GetLoggingDirectory));
  exports.Set("initRosoutPublisherForNode",
              Napi::Function::New(env, InitRosoutPublisherForNode));
  exports.Set("finiRosoutPublisherForNode",
              Napi::Function::New(env, FiniRosoutPublisherForNode));
#if ROS_VERSION > 2205
  exports.Set("addRosoutSublogger",
              Napi::Function::New(env, AddRosoutSublogger));
  exports.Set("removeRosoutSublogger",
              Napi::Function::New(env, RemoveRosoutSublogger));
#endif
  exports.Set("loggingConfigure", Napi::Function::New(env, LoggingConfigure));
  exports.Set("loggingFini", Napi::Function::New(env, LoggingFini));
  return exports;
}

}  // namespace rclnodejs
