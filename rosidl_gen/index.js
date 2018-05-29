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

const fse = require('fs-extra');
const generateIDLFromAction = require('./action_generator.js');
const generateJSStructFromIDL = require('./idl_generator.js');
const packages = require('./packages.js');
const path = require('path');

const generatedRoot = path.join(__dirname, '../generated/');
const installedPackagePaths = process.env.AMENT_PREFIX_PATH.split(path.delimiter);

async function generateJSStructFromAction(pkg) {
  const results = [];
  pkg.actions.forEach(action => {
    // TODO(minggang): Currently the ament build tool does not support to generate
    // the .msg files from .action files, so we will not check the existence of these
    // files. Once ament support the action, we will check it first.
    results.push(generateIDLFromAction(action));
  });
  const idls = await Promise.all(results);
  idls.forEach((idl) => {
    idl.generatedInterfaces.forEach((interfaceName) => {
      pkg.messages.push({
        pkgName: pkg.pkgName,
        interfaceName,
        subFolder: 'msg',
        filePath: path.join(idl.dir, interfaceName + '.msg')});
    });
  });

  // After adding the idls generated just now, it's going to create the JavaScript
  // files for the whole package.
  await generateJSStructFromIDL(pkg, generatedRoot);
}

async function generateInPath(path) {
  const results = [];
  const pkgs = await packages.findPackagesInDirectory(path);
  pkgs.forEach((pkg) => {
    if (pkg.actions.length !== 0) {
      // Generate the JavaScript message struct from .action file.
      results.push(generateJSStructFromAction(pkg));
    } else {
      // Generate the JavaScript message struct from .msg/.srv file.
      results.push(generateJSStructFromIDL(pkg, generatedRoot));
    }
  });
  await Promise.all(results);
}

async function generateAll(forcedGenerating) {
  const results = [];

  // If we want to create the JavaScript files compulsively (|forcedGenerating| equals to true)
  // or the JavaScript files have not been created (|exist| equals to false),
  // all the JavaScript files will be created.
  const exist = await fse.exists(generatedRoot);
  if (forcedGenerating || !exist) {
    await fse.copy(path.join(__dirname, 'generator.json'), path.join(generatedRoot, 'generator.json'));
    installedPackagePaths.forEach((path) => {
      results.push(generateInPath(path));
    });
    await Promise.all(results);
  }
}

const generator = {
  version() {
    // eslint-disable-next-line
    return fse.readJsonSync(path.join(__dirname, 'generator.json')).version;
  },

  generateAll,
  generateInPath,
  generatedRoot
};

module.exports = generator;
