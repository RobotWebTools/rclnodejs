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
const generateJSStructFromIDL = require('./idl_generator.js');
const packages = require('./packages.js');
const path = require('path');

const generatedRoot = path.join(__dirname, '../generated/');
const serviceMsgPath = path.join(generatedRoot, 'srv_msg');

function getInstalledPackagePaths() {
  return process.env.AMENT_PREFIX_PATH.split(path.delimiter);
}

async function generateInPath(path) {
  const pkgs = await packages.findPackagesInDirectory(path);

  const pkgsInfo = Array.from(pkgs.values());

  await Promise.all(
    pkgsInfo.map((pkgInfo) => generateJSStructFromIDL(pkgInfo, generatedRoot))
  );
}

async function generateAll(forcedGenerating) {
  // If we want to create the JavaScript files compulsively (|forcedGenerating| equals to true)
  // or the JavaScript files have not been created (|exist| equals to false),
  // all the JavaScript files will be created.
  const exist = await fse.exists(generatedRoot);
  if (forcedGenerating || !exist) {
    if (exist) {
      // recursively clear all previously generated struct files
      await fse.emptyDir(generatedRoot);
    }

    await fse.copy(
      path.join(__dirname, 'generator.json'),
      path.join(generatedRoot, 'generator.json')
    );
    await fse.mkdir(serviceMsgPath);
    // Process in AMENT_PREFIX_PATH in reverse order to
    // such that interfaces defined earlier on the AMENX_PREFIX_PATH
    // have higher priority over earlier versions and will override
    // them - occurences of this are expected to be rare.
    for (let path of getInstalledPackagePaths().reverse()) {
      await generateInPath(path);
    }
  }
}

const generator = {
  version() {
    return fse.readJsonSync(path.join(__dirname, 'generator.json')).version;
  },

  generateAll,
  generateInPath,
  generatedRoot,
};

module.exports = generator;
