// Copyright (c) 2025, The Robot Web Tools Contributors
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

const fse = require('../lib/utils.js');
const generateJSStructFromIDL = require('./idl_generator.js');
const packages = require('./packages.js');
const path = require('path');
const idlConvertor = require('../rosidl_convertor/idl_convertor.js');

const generatedRoot = path.join(__dirname, '../generated/');
const idlPath = path.join(generatedRoot, 'share');
const useIDL = !!process.argv.find((arg) => arg === '--idl');

// Get target path from environment variable instead of workerData
const targetPath = process.env.WORKER_TARGET_PATH;

async function generateInPath(targetPath) {
  let pkgsInfo = null;
  if (!useIDL) {
    pkgsInfo = Array.from(
      (await packages.findPackagesInDirectory(targetPath)).values()
    );
  } else {
    const idlPkgs = await packages.findPackagesInDirectory(targetPath, useIDL);
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

async function main() {
  try {
    await generateInPath(targetPath);
    process.exit(0);
  } catch (error) {
    console.error('Worker generation failed:', error.message);
    process.exit(1);
  }
}

main();
