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
    this.dependentMessagesIndex = {};
    this.dependentPackagesIndex = {};
    this.linkLibrariesIndex = {};
    for (let pkg of pkgs.values()) {
      pkg.messages.forEach((messageInfo) => {
        this.messageInfoIndex[this._messageInfoHash(messageInfo)] = messageInfo;
      });
    }
  }

  setSpec(messageInfo, spec) {
    this.specIndex[this._messageInfoHash(messageInfo)] = spec;
  }

  getMessageInfoFromType(type) {
    return this.messageInfoIndex[`${type.pkgName}/${type.type}`];
  }

  /**
   * Gets a list of all messages that a message depends on.
   * @param {string} messageInfo Base message.
   * @returns {object[]} An array of message infos.
   */
  async getDependentMessages(messageInfo) {
    const key = this._messageInfoHash(messageInfo);
    let dependentMessages = this.dependentMessagesIndex[key];
    if (dependentMessages === undefined) {
      this.dependentMessagesIndex[key] = new Promise(async (res) => {
        const set = new Set();
        const spec = await this.getMessageSpec(messageInfo);
        spec.fields.forEach((field) => {
          if (
            field.type.pkgName &&
            field.type.pkgName !== messageInfo.pkgName
          ) {
            // `this.getMessageInfoFromType` must always return the
            // same object for the same type for this to work
            set.add(this.getMessageInfoFromType(field.type));
          }
        });
        res(Array.from(set.values()));
      });
      return this.dependentMessagesIndex[key];
    }
    return dependentMessages;
  }

  /**
   * Gets a list of packages that a given package depends on.
   * @param {string} pkgName Name of the base package
   * @returns {Promise<string[]>} An array containing name of all dependent packages.
   */
  async getDependentPackages(pkgName) {
    let dependentPackages = this.dependentPackagesIndex[pkgName];
    if (dependentPackages === undefined) {
      this.dependentPackagesIndex[pkgName] = new Promise(async (res) => {
        const set = new Set();
        const pkgInfo = this.pkgIndex.get(pkgName);
        for (let messageInfo of pkgInfo.messages) {
          const spec = await this.getMessageSpec(messageInfo);
          spec.fields.forEach((field) => {
            if (field.type.pkgName && field.type.pkgName !== pkgName) {
              set.add(field.type.pkgName);
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
    return `${messageInfo.pkgName}/${messageInfo.interfaceName}`;
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

  async getLinkLibraries(pkgName) {
    let linkLibraries = this.linkLibrariesIndex[pkgName];
    if (linkLibraries === undefined) {
      this.linkLibrariesIndex[pkgName] = new Promise(async (res) => {
        const set = new Set();
        const dependentPackages = await this.getDependentPackages(pkgName);
        const dependentLibraries = (
          await Promise.all(
            dependentPackages.map((name) => this.getLinkLibraries(name))
          )
        ).flat();
        dependentLibraries.forEach((lib) => set.add(lib));

        const pkg = this.pkgIndex.get(pkgName);
        const generatorCLibs = await this._guessGeneratorCLibs(pkg);
        generatorCLibs.forEach((lib) => set.add(lib));

        res(Array.from(set.values()));
      });
      linkLibraries = this.linkLibrariesIndex[pkgName];
    }
    return linkLibraries;
  }

  /**
   * Tries to guess the libraries needed to be linked against for generator_c.
   *
   * Normally, the typesupport libraries are built at compiled time of the message packages. The
   * libraries that have to be linked against are based on the name of the cmake target. Because
   *
   *   1. We are a third party library, so our typesupport cannot be built at the compile time.
   *   2. We are use node-gyp as the build tool, so we can't use cmake to find the link libraries.
   *
   * The stopgap solution is then to use regexp on the cmake config files to try to find the
   * required link libraries.
   *
   * @param {string} pkg package info
   * @param {string} amentRoot ament root directory
   * @returns {Promise<string[]>} an array of generator_c libraries found
   */
  async _guessGeneratorCLibs(pkg) {
    const cmakeExport = await fs.readFile(
      path.join(
        pkg.amentRoot,
        'share',
        pkg.pkgName,
        'cmake',
        'ament_cmake_export_libraries-extras.cmake'
      ),
      'utf-8'
    );
    const match = cmakeExport.match(
      /set\s*\(\s*(?:_exported_typesupport_libraries|_exported_libraries)\s*"(.*)"/
    );
    if (!match || match.length < 2) {
      throw new Error(`unable to find generator_c library for ${pkg}`);
    }
    const libraries = match[1].replace(/:/g, ';');
    const generatorCLibs = [];
    libraries.split(';').forEach((lib) => {
      if (lib.endsWith('rosidl_generator_c')) {
        generatorCLibs.push(lib);
      }
    });
    return generatorCLibs;
  }
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

async function generateMessageJSStruct(messageInfo, dir, rosIdlDb, options) {
  const spec = await rosIdlDb.getMessageSpec(messageInfo);
  await generateMessageJSStructFromSpec(messageInfo, dir, spec, options);
}

async function generateMessageJSStructFromSpec(
  messageInfo,
  dir,
  spec,
  options
) {
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
      options,
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

function isInternalField(field) {
  return field.name.startsWith('_');
}

function isServiceMessage(messageInfo) {
  return (
    messageInfo.interfaceName.endsWith('_Request') ||
    messageInfo.interfaceName.endsWith('_Response')
  );
}

// All messages are combined in one cpp file to improve compile time.
async function generateCppDefinitions(pkgName, pkgInfo, rosIdlDb, options) {
  if (options.idlProvider !== 'rosidl') {
    return;
  }

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

  const dependentPackages = await rosIdlDb.getDependentPackages(pkgName);

  const source = removeEmptyLines(
    dots.cppDefinitions({
      pkgName,
      pkgInfo,
      messages,
      dependentPackages,
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
      dependentPackages,
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

async function generateActionJSStruct(actionInfo, dir, options) {
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
    spec.goal,
    options
  );

  const resultMsg = generateMessageJSStructFromSpec(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_Result`,
    },
    dir,
    spec.result,
    options
  );

  const feedbackMsg = generateMessageJSStructFromSpec(
    {
      pkgName: actionInfo.pkgName,
      subFolder: actionInfo.subFolder,
      interfaceName: `${actionInfo.interfaceName}_Feedback`,
    },
    dir,
    spec.feedback,
    options
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
    sendGoalRequestSpec,
    options
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
    sendGoalResponseSpec,
    options
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
    getResultRequestSpec,
    options
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
    getResultResponseSpec,
    options
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
    feedbackMessageSpec,
    options
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

async function generateJSStructFromIDL(pkg, dir, rosIdlDb, options) {
  await Promise.all([
    ...pkg.messages.map((messageInfo) =>
      generateMessageJSStruct(messageInfo, dir, rosIdlDb, options)
    ),
    ...pkg.services.map((serviceInfo) =>
      generateServiceJSStruct(serviceInfo, dir)
    ),
    ...pkg.actions.map((actionInfo) =>
      generateActionJSStruct(actionInfo, dir, options)
    ),
  ]);
}

async function generateTypesupportGypi(pkgsEntries, rosIdlDb, options) {
  await fs.mkdir(path.join('src', 'generated'), { recursive: true });
  if (options.idlProvider !== 'rosidl') {
    await fs.writeFile(
      path.join('src', 'generated', 'typesupport.gypi'),
      '# not using rosidl\n{}'
    );
    return;
  }

  const pkgs = await Promise.all(
    pkgsEntries.map(async ([pkgName, pkgInfo]) => ({
      pkgName,
      pkgInfo,
      linkLibraries: await rosIdlDb.getLinkLibraries(pkgName),
      dependencies: await rosIdlDb.getDependentPackages(pkgName),
    }))
  );
  const rendered = removeEmptyLines(dots.typesupportGypi({ pkgs }));
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
