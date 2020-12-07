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

'use strict';

const path = require('path');
const fs = require('fs');
const generator = require('../rosidl_gen/index.js');

let interfaceLoader = {
  loadInterfaceByObject(obj) {
    //
    // `obj` param structure
    //
    // {
    //   package: 'std_msgs',
    //   type: 'msg',
    //   name: 'String',
    // }
    //
    if (
      typeof obj !== 'object' ||
      !obj ||
      !obj.package ||
      !obj.type ||
      !obj.name
    ) {
      throw new TypeError(
        'Should provide an object argument to get ROS message class'
      );
    }
    return this.loadInterface(obj.package, obj.type, obj.name);
  },

  loadInterfaceByString(name) {
    if (typeof name !== 'string') {
      throw new TypeError(
        'Should provide a string argument to get ROS message class'
      );
    }

    // TODO(Kenny): more checks of the string argument
    if (name.indexOf('/') !== -1) {
      let [packageName, type, messageName] = name.split('/');
      return this.loadInterface(packageName, type, messageName);
    }

    // Suppose the name is a package, and traverse the path to collect the IDL files.
    let packagePath = path.join(generator.generatedRoot, name);

    // eslint-disable-next-line
    let interfaces = fs.readdirSync(packagePath);
    if (interfaces.length > 0) {
      return this.loadInterfaceByPath(packagePath, interfaces);
    }

    throw new TypeError(
      'A string argument in expected in "package/type/message" format'
    );
  },

  loadInterfaceByPath(packagePath, interfaces) {
    let interfaceInfos = [];

    interfaces.forEach((file) => {
      let results = file.match(/\w+__(\w+)__(\w+).js$/);
      let type = results[1];
      let name = results[2];
      let filePath = path.join(packagePath, file).normalize();
      interfaceInfos.push({ name, type, filePath });
    });

    let pkg = { srv: {}, msg: {}, action: {} };
    interfaceInfos.forEach((info) => {
      Object.defineProperty(pkg[info.type], info.name, {
        value: require(info.filePath),
      });
    });

    return pkg;
  },

  loadInterface(packageName, type, messageName) {
    if (arguments.length === 1) {
      const type = arguments[0];
      if (typeof type === 'object') {
        return this.loadInterfaceByObject(type);
      } else if (typeof type === 'string') {
        return this.loadInterfaceByString(type);
      }
      throw new Error(`The message required does not exist: ${type}`);
    }

    if (packageName && type && messageName) {
      let filePath = path.join(
        generator.generatedRoot,
        packageName,
        packageName + '__' + type + '__' + messageName + '.js'
      );
      // eslint-disable-next-line
      if (fs.existsSync(filePath)) {
        return require(filePath);
      }
    }
    throw new Error(
      `The message required does not exist: ${packageName}, ${type}, ${messageName}`
    );
  },
};

module.exports = interfaceLoader;
