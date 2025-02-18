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

'use strict';

const os = require('os');
const path = require('path');
const { execSync } = require('child_process');

const dependencies = [
  'rcl',
  'rcutils',
  'rmw',
  'rcl_yaml_param_parser',
  'rosidl_typesupport_interface',
  'rcl_action',
  'action_msgs',
  'service_msgs',
  'unique_identifier_msgs',
  'builtin_interfaces',
  'rcl_lifecycle',
  'lifecycle_msgs',
  'rosidl_runtime_c',
  'rosidl_dynamic_typesupport',
  'type_description_interfaces',
];

const command = os.type() === 'Windows_NT' ? 'where ros2' : 'which ros2';

function getROSRootPath() {
  try {
    // Execute the command to find the ROS2 installation path.
    let ros2Path = execSync(command).toString().trim();
    return path.dirname(path.dirname(ros2Path));
  } catch (error) {
    console.error(`Error: ${error.message}`);
  }
}

function getROSLibPath() {
  return path.join(getROSRootPath(), 'lib');
}

function getROSIncludeRootPath() {
  return path.join(getROSRootPath(), 'include');
}

function getIncludePaths() {
  const includeRoot = getROSIncludeRootPath();
  return dependencies.map((dependency) => {
    return path.join(includeRoot, dependency);
  });
}

module.exports = {
  getROSRootPath,
  getROSLibPath,
  getROSIncludeRootPath,
  getIncludePaths,
};
