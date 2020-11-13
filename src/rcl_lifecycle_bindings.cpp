// Copyright (c) 2020 Wayne Parrott. All rights reserved.
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

#include "rcl_lifecycle_bindings.hpp"

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "handle_manager.hpp"
#include "lifecycle_msgs/msg/transition_event.h"
#include "lifecycle_msgs/srv/change_state.h"
#include "lifecycle_msgs/srv/get_available_states.h"
#include "lifecycle_msgs/srv/get_available_transitions.h"
#include "lifecycle_msgs/srv/get_state.h"
#include "macros.hpp"
#include "rcl_handle.hpp"
#include "rcl_lifecycle/rcl_lifecycle.h"
#include "rcl_utilities.hpp"

namespace rclnodejs {

static v8::Local<v8::Object> wrapState(const rcl_lifecycle_state_t* state) {
  v8::Local<v8::Object> jsState = Nan::New<v8::Object>();
  Nan::Set(jsState, Nan::New("id").ToLocalChecked(), Nan::New(state->id));
  Nan::Set(jsState, Nan::New("label").ToLocalChecked(),
           Nan::New(state->label).ToLocalChecked());
  return jsState;
}

static v8::Local<v8::Object> wrapTransition(
    const rcl_lifecycle_transition_t* transition) {
  v8::Local<v8::Object> jsTransition = Nan::New<v8::Object>();
  Nan::Set(jsTransition, Nan::New("id").ToLocalChecked(),
           Nan::New(transition->id));
  Nan::Set(jsTransition, Nan::New("label").ToLocalChecked(),
           Nan::New(transition->label).ToLocalChecked());
  return jsTransition;
}

NAN_METHOD(CreateLifecycleStateMachine) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  const rcl_node_options_t* node_options =
      reinterpret_cast<const rcl_node_options_t*>(rcl_node_get_options(node));

  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          malloc(sizeof(rcl_lifecycle_state_machine_t)));
  *state_machine = rcl_lifecycle_get_zero_initialized_state_machine();

  const rosidl_message_type_support_t* pn =
      GetMessageTypeSupport("lifecycle_msgs", "msg", "TransitionEvent");
  const rosidl_service_type_support_t* cs =
      GetServiceTypeSupport("lifecycle_msgs", "ChangeState");
  const rosidl_service_type_support_t* gs =
      GetServiceTypeSupport("lifecycle_msgs", "GetState");
  const rosidl_service_type_support_t* gas =
      GetServiceTypeSupport("lifecycle_msgs", "GetAvailableStates");
  const rosidl_service_type_support_t* gat =
      GetServiceTypeSupport("lifecycle_msgs", "GetAvailableTransitions");
  const rosidl_service_type_support_t* gtg =
      GetServiceTypeSupport("lifecycle_msgs", "GetAvailableTransitions");

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_lifecycle_state_machine_init(
                               state_machine, node, pn, cs, gs, gas, gat, gtg,
                               true, &node_options->allocator),
                           rcl_get_error_string().str);

  auto js_obj = RclHandle::NewInstance(
      state_machine, node_handle, [state_machine, node, node_options] {
        return rcl_lifecycle_state_machine_fini(state_machine, node,
                                                &node_options->allocator);
      });

  info.GetReturnValue().Set(js_obj);
}

NAN_METHOD(GetCurrentLifecycleState) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* state_machine_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  const rcl_lifecycle_state_t* current_state = state_machine->current_state;
  info.GetReturnValue().Set(wrapState(current_state));
}

NAN_METHOD(GetLifecycleTransitionByLabel) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* state_machine_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  std::string transition_label(*Nan::Utf8String(info[1]));

  auto transition = rcl_lifecycle_get_transition_by_label(
      state_machine->current_state, transition_label.c_str());

  info.GetReturnValue().Set(transition == nullptr ? Nan::New<v8::Object>()
                                                  : wrapTransition(transition));
}

// return all registered states
NAN_METHOD(GetLifecycleStates) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* state_machine_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  v8::Local<v8::Array> states = Nan::New<v8::Array>();

  for (uint8_t i = 0; i < state_machine->transition_map.states_size; ++i) {
    const rcl_lifecycle_state_t state = state_machine->transition_map.states[i];
    v8::Local<v8::Object> jsState = wrapState(&state);
    Nan::Set(states, i, jsState);
  }

  info.GetReturnValue().Set(states);
}

// return all registered transitions
NAN_METHOD(GetLifecycleTransitions) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* state_machine_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  v8::Local<v8::Array> jsTransitions = Nan::New<v8::Array>();

  for (uint8_t i = 0; i < state_machine->transition_map.transitions_size; ++i) {
    auto transition = state_machine->transition_map.transitions[i];
    v8::Local<v8::Object> jsTransitionDesc = Nan::New<v8::Object>();
    Nan::Set(jsTransitionDesc, Nan::New("transition").ToLocalChecked(),
             wrapTransition(&transition));
    Nan::Set(jsTransitionDesc, Nan::New("start_state").ToLocalChecked(),
             wrapState(transition.start));
    Nan::Set(jsTransitionDesc, Nan::New("goal_state").ToLocalChecked(),
             wrapState(transition.goal));

    Nan::Set(jsTransitions, i, jsTransitionDesc);
  }

  info.GetReturnValue().Set(jsTransitions);
}

// return the transitions available from the current state
NAN_METHOD(GetAvailableLifecycleTransitions) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* state_machine_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  v8::Local<v8::Array> jsTransitions = Nan::New<v8::Array>();

  for (uint8_t i = 0; i < state_machine->current_state->valid_transition_size;
       ++i) {
    auto transition = state_machine->current_state->valid_transitions[i];
    v8::Local<v8::Object> jsTransitionDesc = Nan::New<v8::Object>();
    Nan::Set(jsTransitionDesc, Nan::New("transition").ToLocalChecked(),
             wrapTransition(&transition));
    Nan::Set(jsTransitionDesc, Nan::New("start_state").ToLocalChecked(),
             wrapState(transition.start));
    Nan::Set(jsTransitionDesc, Nan::New("goal_state").ToLocalChecked(),
             wrapState(transition.goal));

    Nan::Set(jsTransitions, i, jsTransitionDesc);
  }

  info.GetReturnValue().Set(jsTransitions);
}

NAN_METHOD(GetLifecycleSrvNameAndHandle) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* node_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  RclHandle* state_machine_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[1]).ToLocalChecked());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  std::string lifecycle_srv_field_name(*Nan::Utf8String(info[2]));

  std::string service_name;
  rcl_service_t* service;
  if (lifecycle_srv_field_name.compare("srv_get_state") == 0) {
    service_name = "~/get_state";
    service = &state_machine->com_interface.srv_get_state;
  } else if (lifecycle_srv_field_name.compare("srv_get_available_states") ==
             0) {
    service_name = "~/get_available_states";
    service = &state_machine->com_interface.srv_get_available_states;
  } else if (lifecycle_srv_field_name.compare(
                 "srv_get_available_transitions") == 0) {
    service_name = "~/get_available_transitions";
    service = &state_machine->com_interface.srv_get_available_transitions;
  } else if (lifecycle_srv_field_name.compare("srv_change_state") == 0) {
    service_name = "~/change_state";
    service = &state_machine->com_interface.srv_change_state;
  }

  // build result object {name: <srv_name>, handle: <rcl handle of service_t>}
  v8::Local<v8::Object> named_srv_obj = Nan::New<v8::Object>();
  Nan::Set(named_srv_obj, Nan::New("name").ToLocalChecked(),
           Nan::New(service_name).ToLocalChecked());

  // Note: the following hack is to avoid a 'double free()' error that arises
  // when overlapping RclHandles are created for a rcl_lifecycle_state_model and
  // for each of the state-model's 4 services. The deleter for the state-model
  // RclHandle calls rcl_lifecycle_state_machine_fini(state-model) which frees
  // each of the state-model services. Then the RclHandle deleter for each
  // state-model service is invoked resulting in a call to free() on the already
  // freed service ptr.
  // *** Boom - double free error ***.
  //
  // The work-around (hack) is to prevent the RclHandlers wrapping each of the
  // state-machine services from calling free() on their service ptr. This is
  // accomplished by setting each of the service RclHandlers' pointer field to
  // nullptr causing their Reset() method to return immediately when invoked.
  //
  // The implementation of this approach replaces the nop deleter function on
  // each service RclHandler with a new function that is passed a pointer to the
  // service RclHandler. The deleter function then sets the service RclHandler
  // parameter's pointer to nullptr. All joy - no double free()!
  auto srv_handle =
      RclHandle::NewInstance(service, nullptr, [] { return RCL_RET_OK; });
  RclHandle* rclHandle = RclHandle::Unwrap<RclHandle>(srv_handle);
  RclHandle::Unwrap<RclHandle>(srv_handle)->set_deleter([rclHandle] {
      rclHandle->set_ptr(nullptr);
      return RCL_RET_OK;
    });

  Nan::Set(named_srv_obj, Nan::New("handle").ToLocalChecked(), srv_handle);

  info.GetReturnValue().Set(named_srv_obj);
}

// return null if transition exists from current state
NAN_METHOD(TriggerLifecycleTransitionById) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* state_machine_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());
  int transition_id = Nan::To<int64_t>(info[1]).FromJust();

  bool publish = true;
  rcl_ret_t ret = rcl_lifecycle_trigger_transition_by_id(
      state_machine, transition_id, publish);

  if (ret == RCL_RET_OK) {
    const rcl_lifecycle_state_t* current_state = state_machine->current_state;
    info.GetReturnValue().Set(wrapState(current_state));
  }
}

// return null if transition exists from current state
NAN_METHOD(TriggerLifecycleTransitionByLabel) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  RclHandle* state_machine_handle = RclHandle::Unwrap<RclHandle>(
      Nan::To<v8::Object>(info[0]).ToLocalChecked());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());
  std::string transition_label(*Nan::Utf8String(info[1]));

  bool publish = true;
  rcl_ret_t ret = rcl_lifecycle_trigger_transition_by_label(
      state_machine, transition_label.c_str(), publish);

  if (ret == RCL_RET_OK) {
    const rcl_lifecycle_state_t* current_state = state_machine->current_state;
    info.GetReturnValue().Set(wrapState(current_state));
  }
}

static const char* transitionId2Label(int callback_ret) {
  if (callback_ret ==
      lifecycle_msgs__msg__Transition__TRANSITION_CALLBACK_SUCCESS) {
    return rcl_lifecycle_transition_success_label;
  }

  if (callback_ret ==
      lifecycle_msgs__msg__Transition__TRANSITION_CALLBACK_FAILURE) {
    return rcl_lifecycle_transition_failure_label;
  }

  if (callback_ret ==
      lifecycle_msgs__msg__Transition__TRANSITION_CALLBACK_ERROR) {
    return rcl_lifecycle_transition_error_label;
  }

  if (callback_ret == lifecycle_msgs__msg__Transition__TRANSITION_CONFIGURE) {
    return rcl_lifecycle_configure_label;
  }

  if (callback_ret == lifecycle_msgs__msg__Transition__TRANSITION_ACTIVATE) {
    return rcl_lifecycle_activate_label;
  }

  if (callback_ret == lifecycle_msgs__msg__Transition__TRANSITION_DEACTIVATE) {
    return rcl_lifecycle_deactivate_label;
  }

  if (callback_ret == lifecycle_msgs__msg__Transition__TRANSITION_CLEANUP) {
    return rcl_lifecycle_cleanup_label;
  }

  if (callback_ret ==
          lifecycle_msgs__msg__Transition__TRANSITION_UNCONFIGURED_SHUTDOWN ||
      callback_ret ==
          lifecycle_msgs__msg__Transition__TRANSITION_INACTIVE_SHUTDOWN ||
      callback_ret ==
          lifecycle_msgs__msg__Transition__TRANSITION_ACTIVE_SHUTDOWN) {
    return rcl_lifecycle_shutdown_label;
  }

  return rcl_lifecycle_transition_error_label;
}

NAN_METHOD(GetLifecycleTransitionIdToLabel) {
  v8::Local<v8::Context> currentContent = Nan::GetCurrentContext();
  int callback_ret = Nan::To<int64_t>(info[0]).FromJust();
  const char* transition_label = transitionId2Label(callback_ret);
  info.GetReturnValue().Set(Nan::New(transition_label).ToLocalChecked());
}

std::vector<BindingMethod> lifecycle_binding_methods = {
    {"createLifecycleStateMachine", CreateLifecycleStateMachine},
    {"getCurrentLifecycleState", GetCurrentLifecycleState},
    {"getLifecycleTransitionByLabel", GetLifecycleTransitionByLabel},
    {"getLifecycleStates", GetLifecycleStates},
    {"getLifecycleTransitions", GetLifecycleTransitions},
    {"getAvailableLifecycleTransitions", GetAvailableLifecycleTransitions},
    {"triggerLifecycleTransitionById", TriggerLifecycleTransitionById},
    {"triggerLifecycleTransitionByLabel", TriggerLifecycleTransitionByLabel},
    {"getLifecycleSrvNameAndHandle", GetLifecycleSrvNameAndHandle},
    {"getLifecycleTransitionIdToLabel", GetLifecycleTransitionIdToLabel}};

}  // namespace rclnodejs
