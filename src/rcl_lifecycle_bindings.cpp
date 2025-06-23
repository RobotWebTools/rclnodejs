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

#include "rcl_lifecycle_bindings.h"

#include <lifecycle_msgs/msg/transition_event.h>
#include <lifecycle_msgs/srv/change_state.h>
#include <lifecycle_msgs/srv/get_available_states.h>
#include <lifecycle_msgs/srv/get_available_transitions.h>
#include <lifecycle_msgs/srv/get_state.h>
#include <rcl/error_handling.h>
#include <rcl_lifecycle/rcl_lifecycle.h>

#include <memory>
#include <string>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

static Napi::Object wrapState(Napi::Env env,
                              const rcl_lifecycle_state_t* state) {
  Napi::Object jsState = Napi::Object::New(env);
  jsState.Set("id", state->id);
  jsState.Set("label", state->label);
  return jsState;
}

static Napi::Object wrapTransition(
    Napi::Env env, const rcl_lifecycle_transition_t* transition) {
  Napi::Object jsTransition = Napi::Object::New(env);
  jsTransition.Set("id", transition->id);
  jsTransition.Set("label", transition->label);
  return jsTransition;
}

Napi::Value CreateLifecycleStateMachine(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          malloc(sizeof(rcl_lifecycle_state_machine_t)));
  *state_machine = rcl_lifecycle_get_zero_initialized_state_machine();

  const rosidl_message_type_support_t* pn =
      GetMessageTypeSupport("lifecycle_msgs", "msg", "TransitionEvent");
  const rosidl_service_type_support_t* gas =
      GetServiceTypeSupport("lifecycle_msgs", "GetAvailableStates");
  const rosidl_service_type_support_t* gat =
      GetServiceTypeSupport("lifecycle_msgs", "GetAvailableTransitions");
  const rosidl_service_type_support_t* gtg =
      GetServiceTypeSupport("lifecycle_msgs", "GetAvailableTransitions");
  const rosidl_service_type_support_t* cs =
      GetServiceTypeSupport("lifecycle_msgs", "ChangeState");
  const rosidl_service_type_support_t* gs =
      GetServiceTypeSupport("lifecycle_msgs", "GetState");

#if ROS_VERSION >= 2105
  rcl_lifecycle_state_machine_options_t options =
      rcl_lifecycle_get_default_state_machine_options();
  options.enable_com_interface = info[1].As<Napi::Boolean>().Value();

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_lifecycle_state_machine_init(state_machine, node, pn, cs, gs, gas,
                                       gat, gtg, &options),
      rcl_get_error_string().str);

  auto js_obj = RclHandle::NewInstance(
      env, state_machine, node_handle, [node, env](void* ptr) {
        rcl_lifecycle_state_machine_t* state_machine =
            reinterpret_cast<rcl_lifecycle_state_machine_t*>(ptr);
        rcl_ret_t ret = rcl_lifecycle_state_machine_fini(state_machine, node);
        free(ptr);
        THROW_ERROR_IF_NOT_EQUAL_NO_RETURN(RCL_RET_OK, ret,
                                           rcl_get_error_string().str);
      });
#else
  const rcl_node_options_t* node_options =
      reinterpret_cast<const rcl_node_options_t*>(rcl_node_get_options(node));

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_lifecycle_state_machine_init(
                               state_machine, node, pn, cs, gs, gas, gat, gtg,
                               true, &node_options->allocator),
                           rcl_get_error_string().str);

  auto js_obj = RclHandle::NewInstance(
      env, state_machine, node_handle, [node, node_options, env](void* ptr) {
        rcl_lifecycle_state_machine_t* state_machine =
            reinterpret_cast<rcl_lifecycle_state_machine_t*>(ptr);
        rcl_ret_t ret = rcl_lifecycle_state_machine_fini(
            state_machine, node, &node_options->allocator);
        free(ptr);
        THROW_ERROR_IF_NOT_EQUAL_NO_RETURN(RCL_RET_OK, ret,
                                           rcl_get_error_string().str);
      });
#endif

  return js_obj;
}

Napi::Value GetCurrentLifecycleState(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* state_machine_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  const rcl_lifecycle_state_t* current_state = state_machine->current_state;
  return wrapState(env, current_state);
}

Napi::Value GetLifecycleTransitionByLabel(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* state_machine_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  std::string transition_label = info[1].As<Napi::String>();

  auto transition = rcl_lifecycle_get_transition_by_label(
      state_machine->current_state, transition_label.c_str());

  return transition == nullptr ? Napi::Object::New(env)
                               : wrapTransition(env, transition);
}

// return all registered states
Napi::Value GetLifecycleStates(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* state_machine_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  Napi::Array states = Napi::Array::New(env);

  for (uint8_t i = 0; i < state_machine->transition_map.states_size; ++i) {
    const rcl_lifecycle_state_t state = state_machine->transition_map.states[i];
    Napi::Object jsState = wrapState(env, &state);
    states[i] = jsState;
  }

  return states;
}

// return all registered transitions
Napi::Value GetLifecycleTransitions(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* state_machine_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  Napi::Array jsTransitions = Napi::Array::New(env);

  for (uint8_t i = 0; i < state_machine->transition_map.transitions_size; ++i) {
    auto transition = state_machine->transition_map.transitions[i];
    Napi::Object jsTransitionDesc = Napi::Object::New(env);
    jsTransitionDesc.Set("transition", wrapTransition(env, &transition));
    jsTransitionDesc.Set("start_state", wrapState(env, transition.start));
    jsTransitionDesc.Set("goal_state", wrapState(env, transition.goal));

    jsTransitions[i] = jsTransitionDesc;
  }

  return jsTransitions;
}

// return the transitions available from the current state
Napi::Value GetAvailableLifecycleTransitions(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* state_machine_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  Napi::Array jsTransitions = Napi::Array::New(env);

  for (uint8_t i = 0; i < state_machine->current_state->valid_transition_size;
       ++i) {
    auto transition = state_machine->current_state->valid_transitions[i];
    Napi::Object jsTransitionDesc = Napi::Object::New(env);
    jsTransitionDesc.Set("transition", wrapTransition(env, &transition));
    jsTransitionDesc.Set("start_state", wrapState(env, transition.start));
    jsTransitionDesc.Set("goal_state", wrapState(env, transition.goal));

    jsTransitions[i] = jsTransitionDesc;
  }

  return jsTransitions;
}

Napi::Value GetLifecycleSrvNameAndHandle(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* state_machine_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());

  std::string lifecycle_srv_field_name = info[1].As<Napi::String>();

  rcl_service_t* service = nullptr;
  if (lifecycle_srv_field_name.compare("srv_get_state") == 0) {
    service = &state_machine->com_interface.srv_get_state;
  } else if (lifecycle_srv_field_name.compare("srv_get_available_states") ==
             0) {
    service = &state_machine->com_interface.srv_get_available_states;
  } else if (lifecycle_srv_field_name.compare(
                 "srv_get_available_transitions") == 0) {
    service = &state_machine->com_interface.srv_get_available_transitions;
  } else if (lifecycle_srv_field_name.compare("srv_change_state") == 0) {
    service = &state_machine->com_interface.srv_change_state;
  }

  THROW_ERROR_IF_EQUAL(nullptr, service, "Service not found.");

  std::string service_name = rcl_service_get_service_name(service);

  // build result object {name: <srv_name>, handle: <rcl handle of service_t>}
  Napi::Object named_srv_obj = Napi::Object::New(env);
  named_srv_obj.Set("name", service_name);

  // Note: lifecycle Services are created and managed by their
  // rcl_lifecycle_state_machine. Thus we must not manually
  // free the lifecycle_state_machine's service pointers.
  auto srv_handle = RclHandle::NewInstance(env, service, nullptr, nullptr);

  named_srv_obj.Set("handle", srv_handle);
  return named_srv_obj;
}

// return null if transition exists from current state
Napi::Value TriggerLifecycleTransitionById(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* state_machine_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());
  int transition_id = info[1].As<Napi::Number>().Int64Value();

  bool publish = true;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_lifecycle_trigger_transition_by_id(
                               state_machine, transition_id, publish),
                           rcl_get_error_string().str);

  const rcl_lifecycle_state_t* current_state = state_machine->current_state;
  return wrapState(env, current_state);
}

// return null if transition exists from current state
Napi::Value TriggerLifecycleTransitionByLabel(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* state_machine_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());
  std::string transition_label = info[1].As<Napi::String>();

  bool publish = true;

  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_lifecycle_trigger_transition_by_label(
          state_machine, transition_label.c_str(), publish),
      rcl_get_error_string().str);

  const rcl_lifecycle_state_t* current_state = state_machine->current_state;
  return wrapState(env, current_state);
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

Napi::Value GetLifecycleTransitionIdToLabel(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  int callback_ret = info[0].As<Napi::Number>().Int64Value();
  const char* transition_label = transitionId2Label(callback_ret);
  return Napi::String::New(env, transition_label);
}

Napi::Value GetLifecycleShutdownTransitionLabel(
    const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  return Napi::String::New(env, rcl_lifecycle_shutdown_label);
}

Napi::Value IsInitialized(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* state_machine_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());
  const bool is_initialized =
      RCL_RET_OK == rcl_lifecycle_state_machine_is_initialized(state_machine);
  return Napi::Boolean::New(env, is_initialized);
}

Napi::Value Print(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* state_machine_handle =
      RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_lifecycle_state_machine_t* state_machine =
      reinterpret_cast<rcl_lifecycle_state_machine_t*>(
          state_machine_handle->ptr());
  rcl_print_state_machine(state_machine);
  return env.Undefined();
}

Napi::Object InitLifecycleBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("createLifecycleStateMachine",
              Napi::Function::New(env, CreateLifecycleStateMachine));
  exports.Set("getCurrentLifecycleState",
              Napi::Function::New(env, GetCurrentLifecycleState));
  exports.Set("getLifecycleTransitionByLabel",
              Napi::Function::New(env, GetLifecycleTransitionByLabel));
  exports.Set("getLifecycleStates",
              Napi::Function::New(env, GetLifecycleStates));
  exports.Set("getLifecycleTransitions",
              Napi::Function::New(env, GetLifecycleTransitions));
  exports.Set("getAvailableLifecycleTransitions",
              Napi::Function::New(env, GetAvailableLifecycleTransitions));
  exports.Set("triggerLifecycleTransitionById",
              Napi::Function::New(env, TriggerLifecycleTransitionById));
  exports.Set("triggerLifecycleTransitionByLabel",
              Napi::Function::New(env, TriggerLifecycleTransitionByLabel));
  exports.Set("getLifecycleSrvNameAndHandle",
              Napi::Function::New(env, GetLifecycleSrvNameAndHandle));
  exports.Set("getLifecycleTransitionIdToLabel",
              Napi::Function::New(env, GetLifecycleTransitionIdToLabel));
  exports.Set("getLifecycleShutdownTransitionLabel",
              Napi::Function::New(env, GetLifecycleShutdownTransitionLabel));
  exports.Set("isInitialized", Napi::Function::New(env, IsInitialized));
  exports.Set("print", Napi::Function::New(env, Print));
  return exports;
}

}  // namespace rclnodejs
