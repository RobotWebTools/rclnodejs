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

const fse = require('../lib/utils.js');
const generateJSStructFromIDL = require('./idl_generator.js');
const packages = require('./packages.js');
const path = require('path');
const idlConvertor = require('../rosidl_convertor/idl_convertor.js');
const generatedRoot = path.join(__dirname, '../generated/');
const serviceMsgPath = path.join(generatedRoot, 'srv_msg');
const idlPath = path.join(generatedRoot, 'share');
const useIDL = !!process.argv.find((arg) => arg === '--idl');

function getInstalledPackagePaths() {
  return process.env.AMENT_PREFIX_PATH.split(path.delimiter);
}

async function generateInPath(path) {
  let pkgsInfo = null;
  if (!useIDL) {
    pkgsInfo = Array.from(
      (await packages.findPackagesInDirectory(path)).values()
    );
  } else {
    const idlPkgs = await packages.findPackagesInDirectory(path, useIDL);
    await fse.ensureDir(idlPath);
    const promises = [];
    idlPkgs.forEach((pkg) => {
      pkg.idls.forEach((idl) => {
        promises.push(idlConvertor(idl.pkgName, idl.filePath, idlPath));
      });
    });
    await Promise.all(promises);
    const pkgsFromIdl = await packages.findPackagesInDirectory(idlPath, false);
    pkgsInfo = Array.from(pkgsFromIdl.values());
  }

  await Promise.all(
    pkgsInfo.map((pkgInfo) => generateJSStructFromIDL(pkgInfo, generatedRoot))
  );
}

function generateInPathSyncWorker(targetPath) {
  try {
    // Use child_process.spawnSync for truly synchronous execution
    const result = require('child_process').spawnSync(
      'node',
      [path.join(__dirname, 'generate_worker.js')],
      {
        env: { ...process.env, WORKER_TARGET_PATH: targetPath },
        encoding: 'utf8',
        timeout: 30000,
      }
    );

    if (result.error) {
      throw result.error;
    }

    if (result.status !== 0) {
      throw new Error(
        `Worker process exited with code ${result.status}. stderr: ${result.stderr}`
      );
    }

    return result.stdout;
  } catch (error) {
    throw error;
  }
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
  generateInPathSyncWorker,
  generatedRoot,
  getInstalledPackagePaths,
};

module.exports = generator;
