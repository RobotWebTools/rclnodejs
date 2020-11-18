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
const actionMsgs = require('./action_msgs.js');

dot.templateSettings.strip = false;
dot.log = process.env.RCLNODEJS_LOG_VERBOSE || false;
const dots = dot.process({
  path: path.join(__dirname, '../rosidl_gen/templates'),
});

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

async function generateMessageJSStruct(messageInfo, dir) {
  const spec = await parser.parseMessageFile(
    messageInfo.pkgName,
    messageInfo.filePath
  );
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

async function generateJSStructFromIDL(pkg, dir) {
  await Promise.all([
    ...pkg.messages.map((messageInfo) =>
      generateMessageJSStruct(messageInfo, dir)
    ),
    ...pkg.services.map((serviceInfo) =>
      generateServiceJSStruct(serviceInfo, dir)
    ),
    ...pkg.actions.map((actionInfo) => generateActionJSStruct(actionInfo, dir)),
  ]);
}

module.exports = generateJSStructFromIDL;
