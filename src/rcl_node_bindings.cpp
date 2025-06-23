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

#ifndef SRC_RCL_NODE_BINDINGS_H_
#define SRC_RCL_NODE_BINDINGS_H_

#include "rcl_node_bindings.h"

#include <rcl/arguments.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rcl/remap.h>
#include <rcl_action/rcl_action.h>
#include <rcl_yaml_param_parser/parser.h>
#include <rcl_yaml_param_parser/types.h>

#include <rcpputils/scope_exit.hpp>
// NOLINTNEXTLINE
#include <string>

#include "macros.h"
#include "rcl_handle.h"
#include "rcl_utilities.h"

namespace rclnodejs {

static const int PARAMETER_NOT_SET = 0;
static const int PARAMETER_BOOL = 1;
static const int PARAMETER_INTEGER = 2;
static const int PARAMETER_DOUBLE = 3;
static const int PARAMETER_STRING = 4;
static const int PARAMETER_BYTE_ARRAY = 5;
static const int PARAMETER_BOOL_ARRAY = 6;
static const int PARAMETER_INTEGER_ARRAY = 7;
static const int PARAMETER_DOUBLE_ARRAY = 8;
static const int PARAMETER_STRING_ARRAY = 9;

/*
Convert parsed ros arguments to parameters.

type Parameter = {
  name: string,
  type: number,
  value: object
}

type Node = {
  name: string,
  parameters: array<Parameter>
}

parameters = array<Node>;
*/
Napi::Object wrapParameters(Napi::Env env, rcl_params_t* parsed_args) {
  Napi::Array nodes = Napi::Array::New(env);

  // iterate over nodes
  for (size_t i = 0; i < parsed_args->num_nodes; i++) {
    Napi::Object node = Napi::Object::New(env);
    node.Set("name", Napi::String::New(env, parsed_args->node_names[i]));

    rcl_node_params_t node_parameters = parsed_args->params[i];

    // iterate over node.parameters
    Napi::Array parameters = Napi::Array::New(env);
    for (size_t j = 0; j < node_parameters.num_params; j++) {
      Napi::Object parameter = Napi::Object::New(env);

      parameter.Set(
          "name",
          Napi::String::New(env, parsed_args->params[i].parameter_names[j]));

      int param_type = PARAMETER_NOT_SET;

      // for each value, find type & actual value
      rcl_variant_t value = node_parameters.parameter_values[j];
      if (value.bool_value != NULL) {  // NOLINT()
        param_type = PARAMETER_BOOL;
        parameter.Set("value", Napi::Boolean::New(env, *value.bool_value));
      } else if (value.integer_value != NULL) {  // NOLINT()
        param_type = PARAMETER_INTEGER;
        parameter.Set("value", Napi::Number::New(env, *value.integer_value));
      } else if (value.double_value != NULL) {  // NOLINT()
        param_type = PARAMETER_DOUBLE;
        parameter.Set("value", Napi::Number::New(env, *value.double_value));
      } else if (value.string_value != NULL) {  // NOLINT()
        param_type = PARAMETER_STRING;
        parameter.Set("value", Napi::String::New(env, value.string_value));
      } else if (value.bool_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_BOOL_ARRAY;
        Napi::Array bool_array = Napi::Array::New(env);

        for (size_t k = 0; k < value.bool_array_value->size; k++) {
          bool_array.Set(
              k, Napi::Boolean::New(env, value.bool_array_value->values[k]));
        }
        parameter.Set("value", bool_array);
      } else if (value.string_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_STRING_ARRAY;
        Napi::Array string_array = Napi::Array::New(env);
        for (size_t k = 0; k < value.string_array_value->size; k++) {
          string_array.Set(
              k, Napi::String::New(env, value.string_array_value->data[k]));
        }
        parameter.Set("value", string_array);
      } else if (value.byte_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_BYTE_ARRAY;
        Napi::Array byte_array = Napi::Array::New(env);
        for (size_t k = 0; k < value.byte_array_value->size; k++) {
          byte_array.Set(
              k, Napi::Number::New(env, value.byte_array_value->values[k]));
        }
        parameter.Set("value", byte_array);
      } else if (value.integer_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_INTEGER_ARRAY;
        Napi::Array int_array = Napi::Array::New(env);
        for (size_t k = 0; k < value.integer_array_value->size; k++) {
          int_array.Set(
              k, Napi::Number::New(env, value.integer_array_value->values[k]));
        }
        parameter.Set("value", int_array);
      } else if (value.double_array_value != NULL) {  // NOLINT()
        param_type = PARAMETER_DOUBLE_ARRAY;
        Napi::Array dbl_array = Napi::Array::New(env);
        for (size_t k = 0; k < value.double_array_value->size; k++) {
          dbl_array.Set(
              k, Napi::Number::New(env, value.double_array_value->values[k]));
        }
        parameter.Set("value", dbl_array);
      }

      parameter.Set("type", Napi::Number::New(env, param_type));
      parameters.Set(j, parameter);
    }

    node.Set("parameters", parameters);
    nodes.Set(i, node);
  }

  return nodes;
}

Napi::Value GetParameterOverrides(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* context_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  rcl_arguments_t* parsed_args = &(context->global_arguments);
  rcl_params_t* params = NULL;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_arguments_get_param_overrides(parsed_args, &params),
      rcl_get_error_string().str);

  if (params == NULL) {
    return env.Undefined();
  }

  Napi::Object result = wrapParameters(env, params);

  rcl_yaml_node_struct_fini(params);
  return result;
}

Napi::Value CreateNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  std::string node_name = info[0].As<Napi::String>().Utf8Value();
  std::string name_space = info[1].As<Napi::String>().Utf8Value();
  RclHandle* context_handle = RclHandle::Unwrap(info[2].As<Napi::Object>());
  rcl_context_t* context =
      reinterpret_cast<rcl_context_t*>(context_handle->ptr());

  Napi::Array jsArgv = info[3].As<Napi::Array>();
  size_t argc = jsArgv.Length();
  char** argv = AbstractArgsFromNapiArray(jsArgv);
  RCPPUTILS_SCOPE_EXIT({ FreeArgs(argv, argc); });

  rcl_arguments_t arguments = rcl_get_zero_initialized_arguments();
  rcl_ret_t ret =
      rcl_parse_arguments(argc, argv, rcl_get_default_allocator(), &arguments);
  if ((ret != RCL_RET_OK) || HasUnparsedROSArgs(arguments)) {
    Napi::Error::New(env, "failed to parse arguments")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  RCPPUTILS_SCOPE_EXIT({
    if (RCL_RET_OK != rcl_arguments_fini(&arguments)) {
      Napi::Error::New(env, "failed to fini arguments")
          .ThrowAsJavaScriptException();
      rcl_reset_error();
    }
  });
  bool use_global_arguments = info[4].As<Napi::Boolean>().Value();
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(malloc(sizeof(rcl_node_t)));
  *node = rcl_get_zero_initialized_node();

  rcl_node_options_t options = rcl_node_get_default_options();
  options.use_global_arguments = use_global_arguments;
  options.arguments = arguments;

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_node_init(node, node_name.c_str(),
                                         name_space.c_str(), context, &options),
                           rcl_get_error_string().str);

  auto handle = RclHandle::NewInstance(env, node, nullptr, [env](void* ptr) {
    rcl_node_t* node = reinterpret_cast<rcl_node_t*>(ptr);
    rcl_ret_t ret = rcl_node_fini(node);
    free(ptr);
    THROW_ERROR_IF_NOT_EQUAL_NO_RETURN(RCL_RET_OK, ret,
                                       rcl_get_error_string().str);
  });

  return handle;
}

Napi::Value GetNodeName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  const char* node_name = rcl_node_get_name(node);
  if (!node_name) {
    return env.Undefined();
  } else {
    return Napi::String::New(env, node_name);
  }
}

Napi::Value GetNamespace(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  const char* node_namespace = rcl_node_get_namespace(node);
  if (!node_namespace) {
    return env.Undefined();
  } else {
    return Napi::String::New(env, node_namespace);
  }
}

Napi::Value GetNodeLoggerName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  const char* node_logger_name = rcl_node_get_logger_name(node);
  if (!node_logger_name) {
    return env.Undefined();
  }

  return Napi::String::New(env, node_logger_name);
}

Napi::Value ActionGetClientNamesAndTypesByNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name = info[1].As<Napi::String>().Utf8Value();
  std::string node_namespace = info[2].As<Napi::String>().Utf8Value();

  rcl_names_and_types_t names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_action_get_client_names_and_types_by_node(
                               node, &allocator, node_name.c_str(),
                               node_namespace.c_str(), &names_and_types),
                           "Failed to action client names and types.");

  Napi::Array result_list = Napi::Array::New(env, names_and_types.names.size);
  ExtractNamesAndTypes(names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&names_and_types),
                           "Failed to destroy names_and_types");

  return result_list;
}

Napi::Value ActionGetServerNamesAndTypesByNode(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string node_name = info[1].As<Napi::String>().Utf8Value();
  std::string node_namespace = info[2].As<Napi::String>().Utf8Value();

  rcl_names_and_types_t names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_action_get_server_names_and_types_by_node(
                               node, &allocator, node_name.c_str(),
                               node_namespace.c_str(), &names_and_types),
                           "Failed to action server names and types");

  Napi::Array result_list = Napi::Array::New(env, names_and_types.names.size);
  ExtractNamesAndTypes(names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&names_and_types),
                           "Failed to destroy names_and_types");

  return result_list;
}

Napi::Value ActionGetNamesAndTypes(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());

  rcl_names_and_types_t names_and_types =
      rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK,
      rcl_action_get_names_and_types(node, &allocator, &names_and_types),
      "Failed to action server names and types");

  Napi::Array result_list = Napi::Array::New(env, names_and_types.names.size);
  ExtractNamesAndTypes(names_and_types, &result_list);

  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK,
                           rcl_names_and_types_fini(&names_and_types),
                           "Failed to destroy names_and_types");

  return result_list;
}

Napi::Value CountPublishers(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string topic_name = info[1].As<Napi::String>().Utf8Value();

  size_t count = 0;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_count_publishers(node, topic_name.c_str(), &count),
      "Failed to count publishers.");

  return Napi::Number::New(env, static_cast<int32_t>(count));
}

Napi::Value CountSubscribers(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string topic_name = info[1].As<Napi::String>().Utf8Value();

  size_t count = 0;
  THROW_ERROR_IF_NOT_EQUAL(
      RCL_RET_OK, rcl_count_subscribers(node, topic_name.c_str(), &count),
      "Failed to count subscribers.");

  return Napi::Number::New(env, static_cast<int32_t>(count));
}

#if ROS_VERSION > 2205  // 2205 == Humble
Napi::Value CountClients(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());
  std::string service_name = info[1].As<Napi::String>().Utf8Value();

  size_t count = 0;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_count_clients(node, service_name.c_str(), &count), RCL_RET_OK,
      rcl_get_error_string().str);

  return Napi::Number::New(env, count);
}

Napi::Value CountServices(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(
      RclHandle::Unwrap(info[0].As<Napi::Object>())->ptr());
  std::string service_name = info[1].As<Napi::String>().Utf8Value();

  size_t count = 0;
  THROW_ERROR_IF_NOT_EQUAL(
      rcl_count_services(node, service_name.c_str(), &count), RCL_RET_OK,
      rcl_get_error_string().str);

  return Napi::Number::New(env, count);
}
#endif

Napi::Value GetNodeNames(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  bool get_enclaves = info[1].As<Napi::Boolean>().Value();
  rcutils_string_array_t node_names =
      rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t node_namespaces =
      rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t enclaves = rcutils_get_zero_initialized_string_array();
  rcl_allocator_t allocator = rcl_get_default_allocator();

  if (get_enclaves) {
    THROW_ERROR_IF_NOT_EQUAL(
        RCL_RET_OK,
        rcl_get_node_names_with_enclaves(node, allocator, &node_names,
                                         &node_namespaces, &enclaves),
        "Failed to get_node_names.");
  } else {
    THROW_ERROR_IF_NOT_EQUAL(
        RCL_RET_OK,
        rcl_get_node_names(node, allocator, &node_names, &node_namespaces),
        "Failed to get_node_names.");
  }

  Napi::Array result_list = Napi::Array::New(env, node_names.size);

  for (size_t i = 0; i < node_names.size; ++i) {
    Napi::Object item = Napi::Object::New(env);
    item.Set("name", Napi::String::New(env, node_names.data[i]));
    item.Set("namespace", Napi::String::New(env, node_namespaces.data[i]));
    if (get_enclaves) {
      item.Set("enclave", Napi::String::New(env, enclaves.data[i]));
    }
    result_list.Set(i, item);
  }

  rcutils_ret_t fini_names_ret = rcutils_string_array_fini(&node_names);
  rcutils_ret_t fini_namespaces_ret =
      rcutils_string_array_fini(&node_namespaces);
  rcutils_ret_t fini_enclaves_ret = rcutils_string_array_fini(&enclaves);
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, fini_names_ret,
                           "Failed to destroy node_names");
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, fini_namespaces_ret,
                           "Failed to destroy node_namespaces");
  THROW_ERROR_IF_NOT_EQUAL(RCL_RET_OK, fini_enclaves_ret,
                           "Failed to fini enclaves string array");
  return result_list;
}

Napi::Value GetFullyQualifiedName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  const char* fully_qualified_node_name =
      rcl_node_get_fully_qualified_name(node);
  if (!fully_qualified_node_name) {
    Napi::Error::New(env, "Fully qualified name not set")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }
  return Napi::String::New(env, fully_qualified_node_name);
}

Napi::Value GetRMWImplementationIdentifier(const Napi::CallbackInfo& info) {
  return Napi::String::New(info.Env(), rmw_get_implementation_identifier());
}

Napi::Value ResolveName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();

  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  const rcl_node_options_t* node_options = rcl_node_get_options(node);
  std::string topic_name = info[1].As<Napi::String>().Utf8Value();
  bool only_expand = info[2].As<Napi::Boolean>().Value();
  bool is_service = info[3].As<Napi::Boolean>().Value();

  char* output_cstr = nullptr;
  rcl_ret_t ret =
      rcl_node_resolve_name(node, topic_name.c_str(), node_options->allocator,
                            is_service, only_expand, &output_cstr);

  auto name_deleter = [&]() {
    node_options->allocator.deallocate(output_cstr,
                                       node_options->allocator.state);
  };

  RCPPUTILS_SCOPE_EXIT({ name_deleter(); });

  if (RCL_RET_OK != ret) {
    Napi::Error::New(env, ("failed to resolve name"))
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  return Napi::String::New(env, output_cstr);
}

Napi::Value RemapTopicName(const Napi::CallbackInfo& info) {
  Napi::Env env = info.Env();
  RclHandle* node_handle = RclHandle::Unwrap(info[0].As<Napi::Object>());
  rcl_node_t* node = reinterpret_cast<rcl_node_t*>(node_handle->ptr());
  std::string topic_name = info[1].As<Napi::String>().Utf8Value();

  const rcl_node_options_t* node_options = rcl_node_get_options(node);
  if (nullptr == node_options) {
    Napi::Error::New(env, "failed to get node options")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }

  const rcl_arguments_t* global_args = nullptr;
  if (node_options->use_global_arguments) {
    global_args = &(node->context->global_arguments);
  }

  char* output_cstr = nullptr;
  rcl_ret_t ret = rcl_remap_topic_name(
      &(node_options->arguments), global_args, topic_name.c_str(),
      rcl_node_get_name(node), rcl_node_get_namespace(node),
      node_options->allocator, &output_cstr);
  if (RCL_RET_OK != ret) {
    Napi::Error::New(env, "failed to remap topic name")
        .ThrowAsJavaScriptException();
    return env.Undefined();
  }
  if (nullptr == output_cstr) {
    return Napi::String::New(env, topic_name);
  }

  auto name_deleter = [&]() {
    node_options->allocator.deallocate(output_cstr,
                                       node_options->allocator.state);
  };
  RCPPUTILS_SCOPE_EXIT({ name_deleter(); });

  return Napi::String::New(env, output_cstr);
}

Napi::Object InitNodeBindings(Napi::Env env, Napi::Object exports) {
  exports.Set("getParameterOverrides",
              Napi::Function::New(env, GetParameterOverrides));
  exports.Set("createNode", Napi::Function::New(env, CreateNode));
  exports.Set("getNodeName", Napi::Function::New(env, GetNodeName));
  exports.Set("getNamespace", Napi::Function::New(env, GetNamespace));
  exports.Set("getNodeLoggerName", Napi::Function::New(env, GetNodeLoggerName));
  exports.Set("actionGetClientNamesAndTypesByNode",
              Napi::Function::New(env, ActionGetClientNamesAndTypesByNode));
  exports.Set("actionGetServerNamesAndTypesByNode",
              Napi::Function::New(env, ActionGetServerNamesAndTypesByNode));
  exports.Set("actionGetNamesAndTypes",
              Napi::Function::New(env, ActionGetNamesAndTypes));
  exports.Set("countPublishers", Napi::Function::New(env, CountPublishers));
  exports.Set("countSubscribers", Napi::Function::New(env, CountSubscribers));
#if ROS_VERSION > 2205  // 2205 == Humble
  exports.Set("countClients", Napi::Function::New(env, CountClients));
  exports.Set("countServices", Napi::Function::New(env, CountServices));
#endif
  exports.Set("getNodeNames", Napi::Function::New(env, GetNodeNames));
  exports.Set("getFullyQualifiedName",
              Napi::Function::New(env, GetFullyQualifiedName));
  exports.Set("getRMWImplementationIdentifier",
              Napi::Function::New(env, GetRMWImplementationIdentifier));
  exports.Set("resolveName", Napi::Function::New(env, ResolveName));
  exports.Set("remapTopicName", Napi::Function::New(env, RemapTopicName));
  return exports;
}

}  // namespace rclnodejs

#endif  // SRC_RCL_NODE_BINDINGS_H_
