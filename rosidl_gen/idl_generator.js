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
const fs = require('fs').promises;
const fse = require('fs-extra');
const path = require('path');
const parser = require('../rosidl_parser/rosidl_parser.js');
const actionMsgs = require('./action_msgs.js');

class RosIdlDb {
  constructor(pkgs) {
    this.pkgIndex = pkgs;
    this.specIndex = {};
    this.messageInfoIndex = {};
    this.dependentPackagesIndex = {};
    for (let pkg of pkgs.values()) {
      pkg.messages.forEach((messageInfo) => {
        this.messageInfoIndex[
          `${messageInfo.pkgName}__${messageInfo.interfaceName}`
        ] = messageInfo;
      });
    }
  }

  getMessageInfoFromType(type) {
    return this.messageInfoIndex[`${type.pkgName}__${type.type}`];
  }

  async getDependentMessages(pkgName) {
    let dependentPackages = this.dependentPackagesIndex[pkgName];
    if (dependentPackages === undefined) {
      this.dependentPackagesIndex[pkgName] = new Promise(async (res) => {
        const set = new Set();
        const pkgInfo = this.pkgIndex.get(pkgName);
        for (let messageInfo of pkgInfo.messages) {
          const spec = await this.getMessageSpec(messageInfo);
          spec.fields.forEach((field) => {
            if (field.type.pkgName && field.type.pkgName !== pkgName) {
              // `this.getMessageInfoFromType` must always return the
              // same object for the same type for this to work
              set.add(this.getMessageInfoFromType(field.type));
            }
          });
        }
        res(Array.from(set.values()));
      });
      return this.dependentPackagesIndex[pkgName];
    }
    return dependentPackages;
  }

  _messageInfoHash(messageInfo) {
    return `${messageInfo.pkgName}__${messageInfo.interfaceName}`;
  }

  async getMessageSpec(messageInfo) {
    let spec = this.specIndex[this._messageInfoHash(messageInfo)];
    if (spec) {
      return spec;
    }
    const promise = new Promise(async (res) => {
      const spec = await parser.parseMessageFile(
        messageInfo.pkgName,
        messageInfo.filePath
      );
      this.specIndex[this._messageInfoHash(messageInfo)] = spec;
      res(spec);
    });
    this.specIndex[this._messageInfoHash(messageInfo)] = promise;
    return promise;
  }

  // _getDependentPackagesImpl(pkg)

  // getDependentPackages(pkg) {
  //   pkg.messages.forEach((messageInfo) => {

  //   })
  // }
}

dot.templateSettings.strip = false;
dot.log = process.env.RCLNODEJS_LOG_VERBOSE || false;
const dots = dot.process({
  path: path.join(__dirname, '../rosidl_gen/templates'),
});

function pascalToSnakeCase(s) {
  let result = s.replace(/(.)([A-Z][a-z]+)/g, '$1_$2');
  result = result.replace(/([a-z0-9])([A-Z])/g, '$1_$2');
  return result.toLowerCase();
}

function getRosHeaderField(messageInfo) {
  return `${messageInfo.pkgName}/${messageInfo.subFolder}/${pascalToSnakeCase(
    messageInfo.interfaceName
  )}.h`;
}

function removeEmptyLines(str) {
  return str.replace(/^\s*\n/gm, '');
}

async function writeGeneratedCode(dir, fileName, code) {
  await fse.mkdirs(dir);
  await fse.writeFile(path.join(dir, fileName), code);
}

function generateServiceJSStruct(serviceInfo, dir) {
  dir = path.join(dir, `${serviceInfo.pkgName}`);
  const fileName =
    serviceInfo.pkgName +
    '__' +
    serviceInfo.subFolder +
    '__' +
    serviceInfo.interfaceName +
    '.js';
  const generatedCode = removeEmptyLines(
    dots.service({ serviceInfo: serviceInfo })
  );
  return writeGeneratedCode(dir, fileName, generatedCode);
}

async function generateMessageJSStruct(messageInfo, dir, rosIdlDb) {
  const spec = await rosIdlDb.getMessageSpec(messageInfo);
  await generateMessageJSStructFromSpec(messageInfo, dir, spec);
}

function generateMessageJSStructFromSpec(messageInfo, dir, spec) {
  dir = path.join(dir, `${spec.baseType.pkgName}`);
  const fileName =
    spec.baseType.pkgName +
    '__' +
    messageInfo.subFolder +
    '__' +
    spec.msgName +
    '.js';

  const generatedCode = removeEmptyLines(
    dots.message({
      messageInfo: messageInfo,
      spec: spec,
      json: JSON.stringify(spec, null, '  '),
    })
  );
  return writeGeneratedCode(dir, fileName, generatedCode);
}

function getCppOutputDir(pkgName) {
  return path.join('src', 'generated', pkgName);
}

function getJsType(rosType) {
  if (rosType.isArray) {
    return 'object';
  }
  if (rosType.type === 'int64' || rosType.type === 'uint64') {
    return 'bigint';
  } else if (
    rosType.type.startsWith('int') ||
    rosType.type.startsWith('uint') ||
    rosType.type.startsWith('float') ||
    rosType.type === 'double' ||
    rosType.type === 'byte' ||
    rosType.type === 'char'
  ) {
    return 'number';
  } else if (rosType.type === 'string') {
    return 'string';
  } else if (rosType.type === 'bool') {
    return 'boolean';
  }
  return 'object';
}

function isServiceMessage(messageInfo) {
  return (
    messageInfo.interfaceName.endsWith('_Request') ||
    messageInfo.interfaceName.endsWith('_Response')
  );
}

function isInternalField(field) {
  return field.name.startsWith('_');
}

// All messages are combined in one cpp file to improve compile time.
async function generateCppDefinitions(pkgName, pkgInfo, rosIdlDb) {
  const getStructType = (messageInfo) => {
    return `${messageInfo.pkgName}__${messageInfo.subFolder}__${messageInfo.interfaceName}`;
  };

  const getStructTypeFromRosType = (type) => {
    const messageInfo = rosIdlDb.getMessageInfoFromType(type);
    return getStructType(messageInfo);
  };

  const messages = [];

  // this is slower but doing it sequentially maintains ordering
  for (let messageInfo of pkgInfo.messages) {
    if (!isServiceMessage(messageInfo)) {
      messages.push({
        info: messageInfo,
        spec: await rosIdlDb.getMessageSpec(messageInfo),
        structType: getStructType(messageInfo),
      });
    }
  }

  const dependentMessages = await rosIdlDb.getDependentMessages(pkgName);

  const source = removeEmptyLines(
    dots.cppDefinitions({
      pkgName,
      pkgInfo,
      messages,
      dependentMessages,
      rosIdlDb,
      getStructType,
      getRosHeaderField,
      getStructTypeFromRosType,
      getJsType,
      isInternalField,
    })
  );

  const header = removeEmptyLines(
    dots.cppDefinitionsHeader({
      pkgName,
      pkgInfo,
      messages,
      dependentMessages,
      rosIdlDb,
      getStructType,
      getRosHeaderField,
      getStructTypeFromRosType,
      getJsType,
      isInternalField,
    })
  );

  const outputDir = getCppOutputDir(pkgName);
  await fs.mkdir(outputDir, { recursive: true });
  await fs.writeFile(path.join(outputDir, 'definitions.cpp'), source);
  await fs.writeFile(path.join(outputDir, 'definitions.hpp'), header);
}

async function generateActionJSStruct(actionInfo, dir) {
  const spec = await parser.parseActionFile(
    actionInfo.pkgName,
    actionInfo.filePath
  );

  const goalMsg = generateMessageJSStructFromSpec(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_Goal`,
    },
    dir,
    spec.goal
  );

  const resultMsg = generateMessageJSStructFromSpec(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_Result`,
    },
    dir,
    spec.result
  );

  const feedbackMsg = generateMessageJSStructFromSpec(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_Feedback`,
    },
    dir,
    spec.feedback
  );

  const sendGoalRequestSpec = actionMsgs.createSendGoalRequestSpec(
    actionInfo.pkgName,
    actionInfo.interfaceName
  );
  const sendGoalRequestMsg = generateMessageJSStructFromSpec(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_SendGoal_Request`,
    },
    dir,
    sendGoalRequestSpec
  );

  const sendGoalResponseSpec = actionMsgs.createSendGoalResponseSpec(
    actionInfo.pkgName,
    actionInfo.interfaceName
  );
  const sendGoalResponseMsg = generateMessageJSStructFromSpec(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_SendGoal_Response`,
    },
    dir,
    sendGoalResponseSpec
  );

  const sendGoalSrv = generateServiceJSStruct(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_SendGoal`,
    },
    dir
  );

  const getResultRequestSpec = actionMsgs.createGetResultRequestSpec(
    actionInfo.pkgName,
    actionInfo.interfaceName
  );
  const getResultRequestMsg = generateMessageJSStructFromSpec(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_GetResult_Request`,
    },
    dir,
    getResultRequestSpec
  );

  const getResultResponseSpec = actionMsgs.createGetResultResponseSpec(
    actionInfo.pkgName,
    actionInfo.interfaceName
  );
  const getResultResponseMsg = generateMessageJSStructFromSpec(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_GetResult_Response`,
    },
    dir,
    getResultResponseSpec
  );

  const getResultSrv = generateServiceJSStruct(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_GetResult`,
    },
    dir
  );

  const feedbackMessageSpec = actionMsgs.createFeedbackMessageSpec(
    actionInfo.pkgName,
    actionInfo.interfaceName
  );
  const feedbackMessageMsg = generateMessageJSStructFromSpec(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_FeedbackMessage`,
    },
    dir,
    feedbackMessageSpec
  );

  const fileName =
    actionInfo.pkgName +
    '__' +
    actionInfo.subFolder +
    '__' +
    actionInfo.interfaceName +
    '.js';
  const generatedCode = removeEmptyLines(
    dots.action({ actionInfo: actionInfo })
  );
  dir = path.join(dir, actionInfo.pkgName);
  const action = writeGeneratedCode(dir, fileName, generatedCode);

  await Promise.all([
    goalMsg,
    resultMsg,
    feedbackMsg,
    sendGoalRequestMsg,
    sendGoalResponseMsg,
    sendGoalSrv,
    getResultRequestMsg,
    getResultResponseMsg,
    getResultSrv,
    feedbackMessageMsg,
    action,
  ]);
}

async function generateJSStructFromIDL(pkg, dir, rosIdlDb) {
  await Promise.all([
    ...pkg.messages.map((messageInfo) =>
      generateMessageJSStruct(messageInfo, dir, rosIdlDb)
    ),
    ...pkg.services.map((serviceInfo) =>
      generateServiceJSStruct(serviceInfo, dir)
    ),
    ...pkg.actions.map((actionInfo) => generateActionJSStruct(actionInfo, dir)),
  ]);
}

async function generateTypesupportGypi(pkgsEntries) {
  const pkgs = [];
  for (let [pkgName, pkgInfo] of pkgsEntries) {
    if (await fs.stat())
  }
  const rendered = removeEmptyLines(
    dots.typesupportGypi({
      pkgs: pkgsEntries.map(([pkgName, pkgInfo]) => ({
        pkgName,
        pkgInfo,
      })),
    })
  );
  await fs.writeFile(
    path.join('src', 'generated', 'typesupport.gypi'),
    rendered
  );
}

module.exports = {
  RosIdlDb,
  generateJSStructFromIDL,
  generateCppDefinitions,
  generateTypesupportGypi,
};
