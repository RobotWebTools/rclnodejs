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

const fs = require('fs');
const path = require('path');
const walk = require('walk');
const os = require('os');

const generatedRoot = path.join(__dirname, '../generated/');
let pkgMap = new Map();

function getPackageName(filePath, amentExecuted) {
  if (os.type() === 'Windows_NT') {
    filePath = filePath.replace(/\\/g, '/');
  }

  if (amentExecuted) {
    return filePath.match(/\w+\/share\/(\w+)\//)[1];
  }
  // If the |amentExecuted| equals to false, the name of the folder where this file locates
  // will be assigned as the name of the package.
  let folders = path.parse(filePath).dir.split('/');
  return folders.pop();
};

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
  return {pkgName, interfaceName, subFolder, filePath};
}

function addInterfaceInfo(info, type) {
  let pkgName = info.pkgName;
  if (!pkgMap.has(pkgName)) {
    pkgMap.set(pkgName, {messages: [], services: [], actions: []});
  }
  let pkg = pkgMap.get(pkgName);
  pkg[type].push(info);
}

function findPackagesInDirectory(dir, ignoreAction = false) {
  return new Promise((resolve, reject) => {
    let amentExecuted = true;

    // If there is a folder named 'share' under the root path, we consider that
    // the ament build tool has been executed and |amentExecuted| will be true.
    fs.access((dir + '/share'), (err) => {
      if (err) {
        amentExecuted = false;
      }
      dir = amentExecuted ? (dir + '/share') : dir;

      let walker = walk.walk(dir, {followLinks: true});
      pkgMap.clear();
      walker.on('file', (root, file, next) => {
        if (path.extname(file.name) === '.msg') {
          addInterfaceInfo(grabInterfaceInfo(root + '/' + file.name, amentExecuted), 'messages');
        } else if (path.extname(file.name) === '.srv') {
          addInterfaceInfo(grabInterfaceInfo(root + '/' + file.name, amentExecuted), 'services');
        } else if (!ignoreAction && path.extname(file.name) === '.action') {
          addInterfaceInfo(grabInterfaceInfo(root + '/' + file.name, amentExecuted), 'actions');
        }
        next();
      });

      walker.on('end', () => {
        resolve(pkgMap);
      });

      walker.on('errors', (root, stats, next) => {
        reject(stats);
      });
    });
  });
}

let packages = {
  findPackagesInDirectory: findPackagesInDirectory
};

module.exports = packages;
