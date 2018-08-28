// Copyright (c) 2018 Intel Corporation. All rights reserved.
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

const dot = require('dot');
const fse = require('fs-extra');
const path = require('path');
const parser = require('../rosidl_parser/rosidl_parser.js');

dot.templateSettings.strip = false;
dot.log = process.env.RCLNODEJS_LOG_VERBOSE || false;
const dots = dot.process({path: path.join(__dirname, '../rosidl_gen/templates')});

function removeExtraSpaceLines(str) {
  return str.replace(/^\s*\n/gm, '');
}

async function writeGeneratedCode(dir, fileName, code) {
  await fse.mkdirs(dir);
  await fse.writeFile(path.join(dir, fileName), code);
}

function generateServiceJSStruct(serviceInfo, dir) {
  dir = path.join(dir, `${serviceInfo.pkgName}`);
  const fileName = serviceInfo.pkgName + '__' + serviceInfo.subFolder + '__' + serviceInfo.interfaceName + '.js';
  const generatedCode = removeExtraSpaceLines(dots.service({serviceInfo: serviceInfo}));
  return writeGeneratedCode(dir, fileName, generatedCode);
}

async function generateMessageJSStruct(messageInfo, dir) {
  const spec = await parser.parseMessageFile(messageInfo.pkgName, messageInfo.filePath);
  dir = path.join(dir, `${spec.baseType.pkgName}`);
  const fileName = spec.baseType.pkgName + '__' + messageInfo.subFolder + '__' + spec.msgName + '.js';

  const generatedCode = removeExtraSpaceLines(dots.message({
    messageInfo: messageInfo,
    spec: spec,
    json: JSON.stringify(spec, null, '  '),
  }));
  await writeGeneratedCode(dir, fileName, generatedCode);
}

async function generateJSStructFromIDL(pkg, dir) {
  const results = [];
  pkg.messages.forEach((messageInfo) => {
    results.push(generateMessageJSStruct(messageInfo, dir));
  });
  pkg.services.forEach((serviceInfo) => {
    results.push(generateServiceJSStruct(serviceInfo, dir));
  });
  await Promise.all(results);
}

module.exports = generateJSStructFromIDL;
