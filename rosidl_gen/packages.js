// Copyright (c) 2017 Intel Corporation. All rights reserved.
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

const debug = require('debug')('rosidl_gen:packages');
const fs = require('fs');
const readline = require('readline');
const path = require('path');
const walk = require('walk');
const os = require('os');
const pkgFilters = require('../rosidl_gen/filter.js');

const fsp = fs.promises;

const generatedRoot = path.join(__dirname, '../generated/');
const serviceMsgPath = path.join(generatedRoot, 'srv_msg');

function getPackageName(filePath, amentExecuted) {
  if (os.type() === 'Windows_NT') {
    filePath = filePath.replace(/\\/g, '/');
  }

  if (amentExecuted) {
    return filePath.match(/\w+\/share\/(\w+)\//)[1];
  }

  let folders = path.parse(filePath).dir.split('/');
  let packageName = folders.pop();

  // If |packageName| equals to the file's extension, e.g. msg/srv, one level
  // up directory will be used as the package name.
  return packageName === path.parse(filePath).ext.substr(1)
    ? folders.pop()
    : packageName;
}

function getSubFolder(filePath, amentExecuted) {
  if (os.type() === 'Windows_NT') {
    filePath = filePath.replace(/\\/g, '/');
  }

  if (amentExecuted) {
    return filePath.match(/\w+\/share\/\w+\/(\w+)\//)[1];
  }
  // If the |amentExecuted| equals to false, the file's extension will be assigned as
  // the name of sub folder.
  return path.parse(filePath).ext.substr(1);
}

function grabInterfaceInfo(filePath, amentExecuted) {
  let pkgName = getPackageName(filePath, amentExecuted);
  let interfaceName = path.parse(filePath).name;
  let subFolder = getSubFolder(filePath, amentExecuted);
  const isServiceEvent = false;
  return { pkgName, interfaceName, subFolder, filePath, isServiceEvent };
}

function addInterfaceInfo(info, type, pkgMap) {
  let pkgName = info.pkgName;
  if (!pkgMap.has(pkgName)) {
    pkgMap.set(pkgName, { messages: [], services: [], actions: [], pkgName });
  }
  let pkg = pkgMap.get(pkgName);
  pkg[type].push(info);
}

/**
 * Gets all ament packages with ros messages in an ament index.
 * @param {string} rootDir Path to the ament root directory
 * @returns {string[]} array of package names
 */
async function getAmentPackages(rootDir) {
  try {
    const files = await fsp.readdir(
      path.join(
        rootDir,
        'share',
        'ament_index',
        'resource_index',
        'rosidl_interfaces'
      )
    );
    return files;
  } catch (e) {
    if (!e.code === 'ENOENT') {
      throw e;
    }
    return [];
  }
}

/**
 * Get paths to all rosidl resources in an ament package
 * @param {string} packageName name of the package
 * @param {string} amentRoot ament root directory
 * @returns {string[]} array of rosidl ament resources in a package
 */
async function getPackageDefinitionsFiles(packageName, amentRoot) {
  const rosFiles = [];
  const rl = readline.createInterface(
    fs.createReadStream(
      path.join(
        amentRoot,
        'share',
        'ament_index',
        'resource_index',
        'rosidl_interfaces',
        packageName
      ),
      { emitClose: true }
    )
  );
  rl.on('line', (line) => {
    rosFiles.push(path.join(amentRoot, 'share', packageName, line));
  });
  await new Promise((res) => {
    rl.on('close', res);
  });
  return rosFiles;
}

async function generateMsgForSrv(filePath, interfaceInfo, pkgMap) {
  const requestMsgName = `${path.parse(filePath).name}_Request.msg`;
  const responseMsgName = `${path.parse(filePath).name}_Response.msg`;
  const data = await fsp.readFile(filePath, 'utf8');

  const arr = data.split(/-{3,}/);
  if (arr.length == 2) {
    const packagePath = path.join(serviceMsgPath, interfaceInfo.pkgName);
    if (!fs.existsSync(packagePath)) {
      fs.mkdirSync(packagePath);
    }

    await fsp.writeFile(path.join(packagePath, requestMsgName), arr[0]);
    await fsp.writeFile(path.join(packagePath, responseMsgName), arr[1]);
    let requestInfo = Object.assign({}, interfaceInfo);
    requestInfo.filePath = path.join(packagePath, requestMsgName);
    requestInfo.interfaceName = requestInfo.interfaceName + '_Request';
    let responseInfo = Object.assign({}, interfaceInfo);
    responseInfo.filePath = path.join(packagePath, responseMsgName);
    responseInfo.interfaceName = responseInfo.interfaceName + '_Response';

    addInterfaceInfo(requestInfo, 'messages', pkgMap);
    addInterfaceInfo(responseInfo, 'messages', pkgMap);
  }
}

async function addInterfaceInfos(filePath, dir, pkgMap) {
  const interfaceInfo = grabInterfaceInfo(filePath, true);
  const ignore = pkgFilters.matchesAny(interfaceInfo);
  if (!ignore) {
    if (path.extname(filePath) === '.msg') {
      // Some .msg files were generated prior to 0.3.2 for .action files,
      // which has been disabled. So these files should be ignored here.
      if (path.dirname(dir).split(path.sep).pop() !== 'action') {
        addInterfaceInfo(interfaceInfo, 'messages', pkgMap);
      }
    } else if (path.extname(filePath) === '.srv') {
      const requestMsgName = `${path.parse(filePath).name}_Request.msg`;
      if (!fs.existsSync(path.join(path.dirname(filePath), requestMsgName))) {
        await generateMsgForSrv(filePath, interfaceInfo, pkgMap);
      }
      addInterfaceInfo(interfaceInfo, 'services', pkgMap);
    } else if (path.extname(filePath) === '.action') {
      addInterfaceInfo(interfaceInfo, 'actions', pkgMap);
    } else {
      // we ignore all other files
    }
  }
}

/**
 * Collects all packages in a directory by using the ament index.
 * @param {string} dir - the directory to search in
 * @return {Promise<Map<string, object>>} A mapping from the package name to some info about it.
 */
async function findAmentPackagesInDirectory(dir) {
  const pkgs = await getAmentPackages(dir);
  const files = await Promise.all(
    pkgs.map((pkg) => getPackageDefinitionsFiles(pkg, dir))
  );

  const rosFiles = files.flat();
  const pkgMap = new Map();
  await Promise.all(
    rosFiles.map((filePath) => addInterfaceInfos(filePath, dir, pkgMap))
  );
  return pkgMap;
}

/**
 * Collects all packages in a directory.
 * @param {string} dir - the directory to search in
 * @return {Promise<Map<string, object>>} A mapping from the package name to some info about it.
 */
async function findPackagesInDirectory(dir) {
  return new Promise((resolve, reject) => {
    let amentExecuted = true;

    // If there is a folder named 'share' under the root path, we consider that
    // the ament build tool has been executed and |amentExecuted| will be true.
    fs.access(path.join(dir, 'share'), (err) => {
      if (err) {
        amentExecuted = false;
      }

      if (amentExecuted) {
        return resolve(findAmentPackagesInDirectory(dir));
      }

      let walker = walk.walk(dir, { followLinks: true });
      let pkgMap = new Map();
      walker.on('file', (root, file, next) => {
        const interfaceInfo = grabInterfaceInfo(
          path.join(root, file.name),
          amentExecuted
        );
        const ignore = pkgFilters.matchesAny(interfaceInfo);
        if (!ignore) {
          if (path.extname(file.name) === '.msg') {
            // Some .msg files were generated prior to 0.3.2 for .action files,
            // which has been disabled. So these files should be ignored here.
            if (path.dirname(root).split(path.sep).pop() !== 'action') {
              addInterfaceInfo(interfaceInfo, 'messages', pkgMap);
            }
          } else if (path.extname(file.name) === '.srv') {
            addInterfaceInfo(interfaceInfo, 'services', pkgMap);
          } else if (path.extname(file.name) === '.action') {
            addInterfaceInfo(interfaceInfo, 'actions', pkgMap);
          } else {
            // we ignore all other files
          }
        }
        next();
      });

      walker.on('end', () => {
        resolve(pkgMap);
      });

      walker.on('errors', (root, stats, next) => {
        debug(stats);
        next();
      });
    });
  });
}

let packages = {
  findPackagesInDirectory: findPackagesInDirectory,
};

module.exports = packages;
