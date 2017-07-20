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
  loadInterfaceInfos(packageName) {
    let packagePath = path.join(generator.generatedRoot, packageName);
    // The files under the package folder are limited, so it will not cost too
    // much time and result in blocking the main thread.
    // eslint-disable-next-line
    let files = fs.readdirSync(packagePath);
    let interfaceInfos = [];

    files.forEach((file) => {
      let name = file.match(/\w+__(\w+)?.js$/)[1];
      let filePath = path.join(packagePath, file).normalize();
      interfaceInfos.push({name, filePath});
    });
    return interfaceInfos;
  },
};

module.exports = interfaceLoader;
