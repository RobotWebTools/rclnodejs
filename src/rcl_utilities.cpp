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

#include "rcl_utilities.hpp"

#if defined(OS_MACOS) || defined(OS_LINUX)
#include <dlfcn.h>
#endif
#include <rcl/rcl.h>
#include <string>

#if defined(OS_WINDOWS)
#include "third_party/dlfcn-win32/dlfcn.h"
#endif

namespace rclnodejs {

typedef const rosidl_message_type_support_t* (*GetMessageTypeSupportFunction)();
typedef const rosidl_service_type_support_t* (*GetServiceTypeSupportFunction)();

#if defined(OS_MACOS)
const char* lib_prefix = "lib";
const char* lib_ext = ".dylib";
#elif defined(OS_LINUX)
const char* lib_prefix = "lib";
const char* lib_ext = ".so";
#elif defined(OS_WINDOWS)
const char* lib_prefix = "";
const char* lib_ext = ".dll";
#endif

void* GetTypeSupportFunctionByInterfaceSymbolName(
    const std::string& symbol_name,
    const std::string& lib_name) {
  // If the dlopen fails for any reason, it will return nullptr.
  // You can use GetErrorMessageAndClear() to get error diagnostic.
  void* lib = dlopen(lib_name.c_str(), RTLD_NOW | RTLD_GLOBAL);

  if (lib)
    return dlsym(lib, symbol_name.c_str());
  else
    return nullptr;
}

const rosidl_message_type_support_t* GetMessageTypeSupport(
    const std::string& package_name,
    const std::string& sub_folder,
    const std::string& msg_name) {
  void* function = GetTypeSupportFunctionByInterfaceSymbolName(
      "rosidl_typesupport_c__get_message_type_support_handle__" + package_name +
          "__" + sub_folder + "__" + msg_name,
      lib_prefix + package_name + "__rosidl_typesupport_c" + lib_ext);
  if (function)
    return reinterpret_cast<GetMessageTypeSupportFunction>(function)();
  else
    return nullptr;
}

const rosidl_service_type_support_t* GetServiceTypeSupport(
    const std::string& package_name,
    const std::string& service_name) {
  void* function = GetTypeSupportFunctionByInterfaceSymbolName(
      "rosidl_typesupport_c__get_service_type_support_handle__" + package_name +
          "__srv__" + service_name,
      lib_prefix + package_name + "__rosidl_typesupport_c" + lib_ext);
  if (function)
    return reinterpret_cast<GetServiceTypeSupportFunction>(function)();
  else
    return nullptr;
}

const char* GetErrorMessageAndClear() {
  return dlerror();
}

}  // namespace rclnodejs
