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
const generator = require('../rosidl_gen/generator.js');

let interfaceLoader = {
  loadInterfaceInPackage(packageName) {
    let packagePath = path.join(generator.generatedRoot, packageName);
    // The files under the package folder are limited, so it will not cost too
    // much time and result in blocking the main thread.
    // eslint-disable-next-line
    let files = fs.readdirSync(packagePath);
    let interfaceInfos = [];

    files.forEach((file) => {
      let results = file.match(/\w+__(\w+)__(\w+).js$/);
      let type = results[1];
      let name = results[2];
      let filePath = path.join(packagePath, file).normalize();
      interfaceInfos.push({name, type, filePath});
    });

    let pkg = {srv: {}, msg: {}};
    interfaceInfos.forEach((info) => {
      Object.defineProperty(pkg[info.type], info.name, {value: require(info.filePath)});
    });

    return pkg;
  },

  loadInterface(packageName, type, messageName) {
    if (packageName && type && messageName) {
      let filePath = path.join(
        generator.generatedRoot, packageName, packageName + '__' + type + '__' + messageName +'.js');
      // eslint-disable-next-line
      if (fs.existsSync(filePath)) {
        return require(filePath);
      }
    }
    throw new Error('The message required does not exist.');
  }
};

module.exports = interfaceLoader;
