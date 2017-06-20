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

#include <rcl/error_handling.h>
#include <rcl/node.h>
#include <rcl/rcl.h>
#include <string>

#include "handle_manager.hpp"
#include "rcl_handle.hpp"
#include "shadow_node.hpp"

namespace rclnodejs {

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
  if (rcl_node_init(node, node_name.c_str(), name_space.c_str(), &options) !=
      RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }
  info.GetReturnValue().Set(rclnodejs::RclHandle::NewInstance(node));
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

  info.GetReturnValue().Set(rclnodejs::RclHandle::NewInstance(timer));
}

NAN_METHOD(IsTimerReady) {
  rclnodejs::RclHandle* timer_handle =
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(info[0]->ToObject());

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
        rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
        info[1]->ToObject())->GetPtr());
    ret = rcl_timer_fini(timer);
  }

  if ("publisher" == type) {
    rcl_publisher_t* publisher = reinterpret_cast<rcl_publisher_t*>(
        rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
        info[1]->ToObject())->GetPtr());
    rcl_node_t* node = reinterpret_cast<rcl_node_t*>(
        rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(
        info[2]->ToObject())->GetPtr());
    ret = rcl_publisher_fini(publisher, node);
  }

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
  }
}

NAN_METHOD(CallTimer) {
  rclnodejs::RclHandle* timer_handle =
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(info[0]->ToObject());

  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(timer_handle->GetPtr());

  rcl_ret_t ret = rcl_timer_call(timer);

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }
}

NAN_METHOD(CancelTimer) {
  rclnodejs::RclHandle* timer_handle =
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(info[0]->ToObject());

  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(timer_handle->GetPtr());

  rcl_ret_t ret = rcl_timer_cancel(timer);

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }
}

NAN_METHOD(IsTimerCanceled) {
  rclnodejs::RclHandle* timer_handle =
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(info[0]->ToObject());

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
  rclnodejs::RclHandle* timer_handle =
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(info[0]->ToObject());

  rcl_timer_t* timer =
      reinterpret_cast<rcl_timer_t*>(timer_handle->GetPtr());

  rcl_ret_t ret = rcl_timer_reset(timer);

  if (ret != RCL_RET_OK) {
    Nan::ThrowError(rcl_get_error_string_safe());
    return;
  }
}

NAN_METHOD(TimerGetTimeUntilNextCall) {
  rclnodejs::RclHandle* timer_handle =
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(info[0]->ToObject());

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
  rclnodejs::RclHandle* timer_handle =
      rclnodejs::RclHandle::Unwrap<rclnodejs::RclHandle>(info[0]->ToObject());

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

NAN_METHOD(Spin) {
  if (info.Length() == 1 && info[0]->IsObject()) {
    rclnodejs::ShadowNode* node =
        rclnodejs::ShadowNode::Unwrap<rclnodejs::ShadowNode>(
            info[0]->ToObject());
    node->Spin();
  }
}

NAN_METHOD(Shutdown) {
  if (info.Length() == 1 && info[0]->IsObject()) {
    rclnodejs::ShadowNode* node =
        rclnodejs::ShadowNode::Unwrap<rclnodejs::ShadowNode>(
            info[0]->ToObject());
    node->Shutdown();
  }
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

  {"createPublisher", CreatePublisher},
  {"rcl_publish_std_string_message", rcl_publish_std_string_message},
  {"publishMessage", PublishMessage},

  {"spin", Spin},
  {"shutdown", Shutdown},
  {"", nullptr}
};

}  // namespace rclnodejs
