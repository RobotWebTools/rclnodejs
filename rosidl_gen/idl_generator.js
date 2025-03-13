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
const { minify } = require('terser');
const fse = require('fs-extra');
const path = require('path');
const parser = require('../rosidl_parser/rosidl_parser.js');
const actionMsgs = require('./action_msgs.js');
const DistroUtils = require('../lib/distro.js');

dot.templateSettings.strip = false;
dot.log = process.env.RCLNODEJS_LOG_VERBOSE || false;
const isDebug = !!process.argv.find((arg) => arg === '--debug');
const dots = dot.process({
  path: path.join(__dirname, '../rosidl_gen/templates'),
});

/**
 * Output generated code to disk. Do not overwrite
 * an existing file. If file already exists do nothing.
 * @param {string} dir
 * @param {string} fileName
 * @param {string} code
 */
async function writeGeneratedCode(dir, fileName, code) {
  let result = null;
  if (!isDebug && fileName.endsWith('.js')) {
    try {
      result = await minify(code);
    } catch (error) {
      console.error(`Error minifying ${fileName}:`, error);
      result = null;
    }
  }

  await fse.mkdirs(dir);
  await fse.writeFile(path.join(dir, fileName), result ? result.code : code);
}

async function generateServiceJSStruct(
  serviceInfo,
  dir,
  isActionService = true
) {
  dir = path.join(dir, `${serviceInfo.pkgName}`);
  const fileName =
    serviceInfo.pkgName +
    '__' +
    serviceInfo.subFolder +
    '__' +
    serviceInfo.interfaceName +
    '.js';
  const generatedSrvCode = dots.service({ serviceInfo: serviceInfo });

  // We are going to only generate the service JavaScript file if it meets one
  // of the followings:
  // 1. It's a action's request/response service.
  // 2. For pre-Humble ROS 2 releases, because it doesn't support service
  //    introspection.
  if (
    isActionService ||
    DistroUtils.getDistroId() <= DistroUtils.getDistroId('humble')
  ) {
    return writeGeneratedCode(dir, fileName, generatedSrvCode);
  }

  return writeGeneratedCode(dir, fileName, generatedSrvCode).then(() => {
    return generateServiceEventMsg(serviceInfo, dir);
  });
}

async function generateServiceEventMsg(serviceInfo, dir) {
  const fileName = serviceInfo.interfaceName + '.msg';
  const generatedEvent = dots.service_event({ serviceInfo: serviceInfo });

  return writeGeneratedCode(dir, fileName, generatedEvent).then(() => {
    serviceInfo.interfaceName += '_Event';
    serviceInfo.filePath = path.join(dir, fileName);
    return generateServiceEventJSStruct(serviceInfo, dir);
  });
}

async function generateServiceEventJSStruct(msgInfo, dir) {
  const spec = await parser.parseMessageFile(msgInfo.pkgName, msgInfo.filePath);
  // Pass `msgInfo.subFolder` to the `spec`, because some .srv files of the
  // package may not be put under srv/ folder, e.g., slam_toolbox.
  spec.subFolder = msgInfo.subFolder;

  // Remove the `.msg` files generated in `generateServiceEventMsg()` to avoid
  // being found later.
  fse.removeSync(msgInfo.filePath);
  const eventFileName =
    msgInfo.pkgName +
    '__' +
    msgInfo.subFolder +
    '__' +
    msgInfo.interfaceName +
    '.js';

  // Set `msgInfo.isServiceEvent` to true, so when generating the JavaScript
  // message files for the service event leveraging message.dot, it will use
  // "__srv__" to require the JS files for the request/response of a specific
  // service, e.g.,
  // const AddTwoInts_RequestWrapper = require('../../generated/example_interfaces/example_interfaces__srv__AddTwoInts_Request.js');
  // const AddTwoInts_ResponseWrapper = require('../../generated/example_interfaces/example_interfaces__srv__AddTwoInts_Response.js');
  msgInfo.isServiceEvent = true;
  const generatedCode = dots.message({
    messageInfo: msgInfo,
    spec: spec,
    json: JSON.stringify(spec, null, '  '),
    isDebug: isDebug,
  });

  return writeGeneratedCode(dir, eventFileName, generatedCode);
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

  const generatedCode = dots.message({
    messageInfo: messageInfo,
    spec: spec,
    json: JSON.stringify(spec, null, '  '),
    isDebug: isDebug,
  });
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
  const generatedCode = dots.action({ actionInfo: actionInfo });
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
  const results = [];
  pkg.messages.forEach((messageInfo) => {
    results.push(generateMessageJSStruct(messageInfo, dir));
  });
  pkg.services.forEach((serviceInfo) => {
    results.push(
      generateServiceJSStruct(serviceInfo, dir, /*isActionService=*/ false)
    );
  });
  pkg.actions.forEach((actionInfo) => {
    results.push(generateActionJSStruct(actionInfo, dir));
  });
  await Promise.all(results);
}

module.exports = generateJSStructFromIDL;
