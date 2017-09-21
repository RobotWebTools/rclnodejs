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

const path = require('path');
const walk = require('walk');
const os = require('os');

const generatedRoot = path.join(__dirname, '../generated/');
let pkgMap = new Map();

function getPackageName(filePath) {
  if (os.type() === 'Windows_NT') {
    filePath = filePath.replace(/\\/g, '/');
  }
  return filePath.match(/\w+\/share\/(\w+)\//)[1];
};

function getSubFolder(filePath) {
  if (os.type() === 'Windows_NT') {
    filePath = filePath.replace(/\\/g, '/');
  }
  return filePath.match(/\w+\/share\/\w+\/(\w+)\//)[1];
}

function grabInterfaceInfo(filePath) {
  let pkgName = getPackageName(filePath);
  let interfaceName = path.parse(filePath).name;
  let subFolder = getSubFolder(filePath);
  return {pkgName, interfaceName, subFolder, filePath};
}

function addInterfaceInfo(info, type) {
  let pkgName = info.pkgName;
  if (!pkgMap.has(pkgName)) {
    pkgMap.set(pkgName, {messages: [], services: []});
  }
  let pkg = pkgMap.get(pkgName);
  pkg[type].push(info);
}

function findPackagesInDirectory(dir) {
  return new Promise((resolve, reject) => {
    let walker = walk.walk(dir + '/share', {followLinks: true});
    pkgMap.clear();

    walker.on('file', (root, file, next) => {
      if (path.extname(file.name) === '.msg') {
        addInterfaceInfo(grabInterfaceInfo(root + '/' + file.name), 'messages');
      } else if (path.extname(file.name) === '.srv') {
        addInterfaceInfo(grabInterfaceInfo(root + '/' + file.name), 'services');
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
}

let packages = {
  findPackagesInDirectory: findPackagesInDirectory
};

module.exports = packages;
