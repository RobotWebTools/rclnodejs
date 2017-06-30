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

#include "rcl_bindings.hpp"

#include <dlfcn.h>
#include <rcl/error_handling.h>
#include <rcl/node.h>
#include <rcl/rcl.h>
#include <rosidl_generator_c/string_functions.h>
#include <string>

#include "handle_manager.hpp"
#include "rcl_handle.hpp"
#include "shadow_node.hpp"

extern "C" {
  typedef const rosidl_message_type_support_t* (*GET_MSG_TYPE_SUPPORT)();
}

namespace rclnodejs {

const rosidl_message_type_support_t* GetMessageTypeSupportType(
    const std::string& package_name, const std::string& message_sub_folder,
    const std::string& message_name) {
  std::string lib_name = "lib";
  lib_name += package_name;
  lib_name += "__rosidl_typesupport_c.so";
  void* lib = dlopen(lib_name.c_str(), RTLD_NOW|RTLD_GLOBAL);

  GET_MSG_TYPE_SUPPORT function_ptr = nullptr;
  std::string function_name = RCLN_GET_MSG_TYPE_SUPPORT(package_name,
      message_sub_folder, message_name);
  function_ptr = (GET_MSG_TYPE_SUPPORT)dlsym(lib, function_name.c_str());

  if (function_ptr) {
    return function_ptr();
  }

  return nullptr;
}

NAN_METHOD(Init) {
  rcl_ret_t ret = rcl_init(0, nullptr, rcl_get_default_allocator());
  if (ret != RCL_RET_OK)
    Nan::ThrowError(rcl_get_error_string_safe());
}

NAN_METHOD(CreateNode) {
  std::string node_name(*v8::String::Utf8Value(info[0]));
  std::string name_space(*v8::String::Utf8Value(info[1]));

  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(malloc(sizeof(rcl_node_t)));

  *node = rcl_get_zero_initialized_node();
  rcl_node_options_t options = rcl_node_get_default_options();

  if (rcl_node_init(node, node_name.c_str(),
      name_space.c_str(), &options) != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }

  auto handle = RclHandle::NewInstance(node, RclHandleType_ROSNode);
  info.GetReturnValue().Set(handle);
}

NAN_METHOD(CreateTimer) {
  // TODO(minggang): Add support for uint64_t.
  int64_t period = info[0]->IntegerValue();

  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(
      malloc(sizeof(rcl_timer_t)));
  *timer = rcl_get_zero_initialized_timer();

  rcl_ret_t ret = rcl_timer_init(timer, period, nullptr,
                                 rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    Nan::ThrowError("Create timer failed");
    return;
  }

  auto jsObj = RclHandle::NewInstance(timer, RclHandleType_Timer);
  info.GetReturnValue().Set(jsObj);
}

NAN_METHOD(IsTimerReady) {
  RclHandle* timer_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());

  rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(timer_handle->GetPtr());

  bool is_ready = false;
  rcl_ret_t ret = rcl_timer_is_ready(timer, &is_ready);

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }

  info.GetReturnValue().Set(Nan::New(is_ready));
}

NAN_METHOD(DestroyEntity) {
  std::string type(*v8::String::Utf8Value(info[0]));

  rcl_ret_t ret = 0;

  if ("timer" == type) {
    rcl_timer_t* timer = reinterpret_cast<rcl_timer_t*>(
        RclHandle::Unwrap<RclHandle>(
        info[1]->ToObject())->GetPtr());
    ret = rcl_timer_fini(timer);
  }

  if ("publisher" == type) {
    rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
        RclHandle::Unwrap<RclHandle>(
        info[1]->ToObject())->GetPtr());
    rcl_node_t* node = reinterpret_cast<rcl_node_t*>(
        RclHandle::Unwrap<RclHandle>(
        info[2]->ToObject())->GetPtr());
    ret = rcl_publisher_fini(publisher, node);
  }

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
  }
}

NAN_METHOD(CallTimer) {
  RclHandle* timer_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());

  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(timer_handle->GetPtr());

  rcl_ret_t ret = rcl_timer_call(timer);

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }
}

NAN_METHOD(CancelTimer) {
  RclHandle* timer_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());

  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(timer_handle->GetPtr());

  rcl_ret_t ret = rcl_timer_cancel(timer);

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }
}

NAN_METHOD(IsTimerCanceled) {
  RclHandle* timer_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());

  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(timer_handle->GetPtr());

  bool is_canceled = false;

  rcl_ret_t ret = rcl_timer_is_canceled(timer, &is_canceled);

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }

  info.GetReturnValue().Set(Nan::New(is_canceled));
}

NAN_METHOD(ResetTimer) {
  RclHandle* timer_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());

  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(timer_handle->GetPtr());

  rcl_ret_t ret = rcl_timer_reset(timer);

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }
}

NAN_METHOD(TimerGetTimeUntilNextCall) {
  RclHandle* timer_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());

  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(timer_handle->GetPtr());

  int64_t remaining_time = 0;

  rcl_ret_t ret = rcl_timer_get_time_until_next_call(timer, &remaining_time);

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }

  info.GetReturnValue().Set(Nan::New((uint32_t)remaining_time));
}

NAN_METHOD(TimerGetTimeSinceLastCall) {
  RclHandle* timer_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());

  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(timer_handle->GetPtr());

  uint64_t elapsed_time = 0;

  rcl_ret_t ret = rcl_timer_get_time_since_last_call(timer, &elapsed_time);

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }

  info.GetReturnValue().Set(Nan::New((uint32_t)elapsed_time));
}

NAN_METHOD(RclTake) {
  RclHandle* subscription_handle =
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject());
  rcl_subscription_t* subscription =
      reinterpret_cast<rcl_subscription_t*>(subscription_handle->GetPtr());
  void* msg_taken = node::Buffer::Data(info[1]->ToObject());

  rcl_ret_t ret = rcl_take(subscription, msg_taken, nullptr);

  if (ret != RCL_RET_OK && ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    Nan::ThrowError(rcl_get_error_string_safe());
    rcl_reset_error();
    info.GetReturnValue().Set(Nan::False());
    return;
  }

  if (ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
    info.GetReturnValue().Set(Nan::True());
    return;
  }
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(CreateSubscription) {
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(
      RclHandle::Unwrap<RclHandle>(
          info[0]->ToObject())->GetPtr());
  std::string package_name(*Nan::Utf8String(info[1]->ToString()));
  std::string message_sub_folder(*Nan::Utf8String(info[2]->ToString()));
  std::string message_name(*Nan::Utf8String(info[3]->ToString()));
  std::string topic(*Nan::Utf8String(info[4]->ToString()));

  rcl_subscription_t * subscription =
    reinterpret_cast<rcl_subscription_t*>(malloc(sizeof(rcl_subscription_t)));
  *subscription = rcl_get_zero_initialized_subscription();

  rcl_subscription_options_t subscription_ops =
      rcl_subscription_get_default_options();
  const rosidl_message_type_support_t * ts =
      GetMessageTypeSupportType(package_name, message_sub_folder, message_name);

  rcl_ret_t ret =
      rcl_subscription_init(subscription, node, ts, topic.c_str(),
                            &subscription_ops);

  if (ret != RCL_RET_OK) {
    Nan::ThrowError("Create subscription failed");
    return;
  }

  auto jsObj = RclHandle::NewInstance(subscription,
      RclHandleType_ROSSubscription, node);
  info.GetReturnValue().Set(jsObj);
}

NAN_METHOD(ROSIDLStringInit) {
  void* buffer = node::Buffer::Data(info[0]->ToObject());
  rosidl_generator_c__String* ptr =
    reinterpret_cast<rosidl_generator_c__String*>(buffer);

  rosidl_generator_c__String__init(ptr);
  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(ROSIDLStringAssign) {
  void* buffer = node::Buffer::Data(info[0]->ToObject());
  std::string value(*v8::String::Utf8Value(info[1]));
  rosidl_generator_c__String* ptr =
    reinterpret_cast<rosidl_generator_c__String*>(buffer);

  // This call will free the previous allocated C-string
  bool ret = rosidl_generator_c__String__assign(ptr, value.c_str());

  if (ret) {
    // We only book the clean-up call, a.k.a. free(),
    //  of the mallocated C-string itself
    info.GetReturnValue().Set(RclHandle::NewInstance(ptr->data,
        RclHandleType_ROSIDLString));
  } else {
    info.GetReturnValue().Set(Nan::Undefined());
  }
}

NAN_METHOD(CreatePublisher) {
  // Extract arguments
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(
      RclHandle::Unwrap<RclHandle>(
      info[0]->ToObject())->GetPtr());
  std::string packageName(*Nan::Utf8String(info[1]->ToString()));
  std::string messageSubFolder(*Nan::Utf8String(info[2]->ToString()));
  std::string messageName(*Nan::Utf8String(info[3]->ToString()));
  std::string topic(*Nan::Utf8String(info[4]->ToString()));

  // Prepare publisher object
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      malloc(sizeof(rcl_publisher_t)));
  *publisher = rcl_get_zero_initialized_publisher();

  // Get type support object dynamically
  // First, the library name
  // TODO(Kenny): support *.dll/etc. on other platforms
  std::string lib_name = "lib";
  lib_name += packageName;
  lib_name += "__rosidl_typesupport_c.so";
  void* lib = dlopen(lib_name.c_str(), RTLD_NOW|RTLD_GLOBAL);
  // User-friendly error message if wrong library name
  std::string lib_error_message = "Cannot load dynamic library (lib='";
  lib_error_message += lib_name;
  lib_error_message += "'). Check spelling or run rosidl_generator_c.";
  RCLN_THROW_EQUAL(lib, nullptr, lib_error_message.c_str());
  // Second, the function name
  GET_MSG_TYPE_SUPPORT function_ptr = nullptr;
  std::string function_name = RCLN_GET_MSG_TYPE_SUPPORT(packageName,
      messageSubFolder, messageName);
  function_ptr = (GET_MSG_TYPE_SUPPORT)dlsym(lib, function_name.c_str());
  // User-friendly error message if wrong function name
  std::string function_error_message;
  function_error_message +=
      "Cannot create message type support object from rcl (symbol='";
  function_error_message += function_name;
  function_error_message += "'). Check spelling or run rosidl_generator_c.";
  RCLN_THROW_EQUAL(function_ptr, nullptr, function_error_message.c_str());
  // Now call the function and get value
  const rosidl_message_type_support_t * ts = function_ptr();

  // Using default options
  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  // Initialize the publisher
  RCLN_CHECK_AND_THROW(rcl_publisher_init(publisher,
      node, ts, topic.c_str(), &publisher_ops), RCL_RET_OK);

  // Wrap the handle into JS object
  auto newObj = RclHandle::NewInstance(publisher,
      RclHandleType_ROSPublisher, node);

  // Everything is done
  info.GetReturnValue().Set(newObj);
}

NAN_METHOD(PublishMessage) {
  rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
      RclHandle::Unwrap<RclHandle>(info[0]->ToObject())->GetPtr());

  void* buffer = node::Buffer::Data(info[1]->ToObject());
  // auto size = node::Buffer::Length(info[1]->ToObject());

  RCLN_CHECK_AND_THROW(rcl_publish(publisher, buffer), RCL_RET_OK);

  info.GetReturnValue().Set(Nan::Undefined());
}

NAN_METHOD(Spin) {
  if (info.Length() == 1 && info[0]->IsObject()) {
    ShadowNode* node =
        ShadowNode::Unwrap<ShadowNode>(
            info[0]->ToObject());
    node->Spin();
  }
}

NAN_METHOD(Shutdown) {
  rcl_shutdown();
  info.GetReturnValue().Set(Nan::Undefined());
}

uint32_t GetBindingMethodsCount(BindingMethod* methods) {
  uint32_t count = 0;
  while (methods[count].function) {
    count++;
  }
  return count;
}

BindingMethod binding_methods[] = {
  {"init", Init},
  {"createNode", CreateNode},
  {"createTimer", CreateTimer},
  {"isTimerReady", IsTimerReady},
  {"destroyEntity", DestroyEntity},
  {"callTimer", CallTimer},
  {"cancelTimer", CancelTimer},
  {"isTimerCanceled", IsTimerCanceled},
  {"resetTimer", ResetTimer},
  {"timerGetTimeSinceLastCall", TimerGetTimeSinceLastCall},
  {"timerGetTimeUntilNextCall", TimerGetTimeUntilNextCall},
  {"rclTake", RclTake},
  {"createSubscription", CreateSubscription},
  {"rosIDLStringAssign", ROSIDLStringAssign},
  {"rosIDLStringInit", ROSIDLStringInit},

  {"createPublisher", CreatePublisher},
  {"publishMessage", PublishMessage},

  {"spin", Spin},
  {"shutdown", Shutdown},
  {"", nullptr}
};

}  // namespace rclnodejs
