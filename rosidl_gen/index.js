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

const child_process = require('child_process');
const fsp = require('fs').promises;
const fse = require('fs-extra');
const {
  RosIdlDb,
  generateJSStructFromIDL,
  generateCppDefinitions,
  generateTypesupportGypi: generateTypesupportGyp,
} = require('./idl_generator.js');
const os = require('os');
const packages = require('./packages.js');
const path = require('path');

const generatedRoot = path.join(__dirname, '../generated/');
const installedPackagePaths = process.env.AMENT_PREFIX_PATH.split(
  path.delimiter
);

async function generateInPaths(paths, options) {
  const pkgsInPaths = await Promise.all(
    paths.map((path) => packages.findPackagesInDirectory(path))
  );
  const pkgs = new Map();
  pkgsInPaths.forEach((m, i) => {
    for (let [pkgName, pkgInfo] of m.entries()) {
      pkgs.set(pkgName, { ...pkgInfo, amentRoot: paths[i] });
    }
  });
  // skip this package as it contains message that is invalid
  pkgs.delete('libstatistics_collector');

  const rosIdlDb = new RosIdlDb(pkgs);

  const pkgsInfo = Array.from(pkgs.values());
  const pkgsEntries = Array.from(pkgs.entries());

  await Promise.all(
    pkgsInfo.map((pkgInfo) =>
      generateJSStructFromIDL(pkgInfo, generatedRoot, rosIdlDb, options)
    )
  );

  if (options.idlProvider === 'rosidl') {
    await Promise.all(
      pkgsEntries.map(([pkgName, pkgInfo]) => {
        generateCppDefinitions(pkgName, pkgInfo, rosIdlDb, options);
      })
    );
    await generateTypesupportGyp(pkgsEntries, rosIdlDb, options);
    await child_process.spawn('npx', ['--no-install', 'node-gyp', 'rebuild'], {
      cwd: `${__dirname}/../src/generated`,
      stdio: 'inherit',
      env: {
        ...process.env,
        JOBS: os.cpus().length,
      },
    });
  }
}

async function generateAll(forcedGenerating) {
  // If we want to create the JavaScript files compulsively (|forcedGenerating| equals to true)
  // or the JavaScript files have not been created (|exist| equals to false),
  // all the JavaScript files will be created.
  const exist = await fse.exists(generatedRoot);
  if (forcedGenerating || !exist) {
    await fse.copy(
      path.join(__dirname, 'generator.json'),
      path.join(generatedRoot, 'generator.json')
    );
    const useRosidl =
      process.env.RCLNODEJS_USE_ROSIDL ||
      process.env.npm_config_rclnodejs_use_rosidl;
    const options = {
      idlProvider: useRosidl ? 'rosidl' : 'ref',
    };
    await generateInPaths(installedPackagePaths, options);
    await fsp.writeFile(
      path.join(generatedRoot, 'generator-options.js'),
      `module.exports = ${JSON.stringify(options)}`
    );
  }
}

const generator = {
  version() {
    // eslint-disable-next-line
    return fse.readJsonSync(path.join(__dirname, 'generator.json')).version;
  },

  generateAll,
  generateInPaths,
  generatedRoot,
};

module.exports = generator;
