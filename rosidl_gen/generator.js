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

const fs = require('mz/fs');
const fse = require('fs-extra');
const mkdirp = require('mkdirp');
const path = require('path');
const parser = require('../rosidl_parser/rosidl_parser.js');
const packages = require('./packages.js');
const dot = require('dot');
const os = require('os');

dot.templateSettings.strip = false;
dot.log = process.env.RCLNODEJS_LOG_VERBOSE || false;
const dots = dot.process({path: path.join(__dirname, '../rosidl_gen/templates')});

const generatedRoot = path.join(__dirname, '../generated/');
const installedPackagesRoot = (os.type() === 'Windows_NT')
  ? process.env.AMENT_PREFIX_PATH.split(';')
  : process.env.AMENT_PREFIX_PATH.split(':');

function removeExtraSpaceLines(str) {
  return str.replace(/^\s*\n/gm, '');
}

function removeAllIntermediateFiles(path) {
  return fse.remove(path);
}

function writeGeneratedCode(dir, fileName, code) {
  mkdirp.sync(dir);
  return fs.writeFile(dir + fileName, code);
}

function generateServiceJSStruct(serviceInfo) {
  const dir = path.join(generatedRoot, `${serviceInfo.pkgName}/`);
  const fileName = serviceInfo.pkgName + '__' + serviceInfo.subFolder + '__' + serviceInfo.interfaceName + '.js';
  const generatedCode = removeExtraSpaceLines(dots.service({serviceInfo: serviceInfo}));
  return writeGeneratedCode(dir, fileName, generatedCode);
}

function generateMessageJSStruct(messageInfo) {
  return new Promise((resolve, reject) => {
    parser.parseMessageFile(messageInfo.pkgName, messageInfo.filePath).then((spec) => {
      const dir = path.join(generatedRoot, `${spec.baseType.pkgName}/`);
      const fileName = spec.baseType.pkgName + '__' + messageInfo.subFolder + '__' + spec.msgName + '.js';

      const generatedCode = removeExtraSpaceLines(dots.message({
        messageInfo: messageInfo,
        spec: spec,
        json: JSON.stringify(spec, null, '  '),
      }));
      return writeGeneratedCode(dir, fileName, generatedCode);
    }).then(() => {
      resolve();
    }).catch((e) => {
      reject(e);
    });
  });
}

function generateStructForPkg(pkg) {
  return new Promise((resolve, reject) => {
    let promises = [];
    pkg.messages.forEach((messageInfo) => {
      promises.push(generateMessageJSStruct(messageInfo));
    });
    pkg.services.forEach((serviceInfo) => {
      promises.push(generateServiceJSStruct(serviceInfo));
    });

    Promise.all(promises).then(() => {
      resolve();
    }).catch((e) => {
      reject(e);
    });
  });
}

function generateInPath(path) {
  return new Promise((resolve, reject) => {
    let promises = [];
    packages.findPackagesInDirectory(path).then((pkgs) => {
      pkgs.forEach((pkg) => {
        promises.push(generateStructForPkg(pkg));
      });

      Promise.all(promises).then(() => {
        resolve();
      }).catch((e) => {
        reject(e);
      });
    }).catch((e) => {
      reject(e);
    });
  });
}

function generateAll(forcedGenerating) {
  return new Promise((resolve, reject) => {
    if (forcedGenerating || !generator.isRootGererated()) {
      fse.copy(path.join(__dirname, 'generator.json'), path.join(generatedRoot, 'generator.json'), err => {
        if (err) reject(err);

        let promises = [];
        installedPackagesRoot.forEach((packageRoot) => {
          promises.push(generateInPath(packageRoot));
        });

        Promise.all(promises).then(() => {
          resolve();
        }).catch((e) => {
          reject(e);
        });
      });
    } else {
      resolve();
    }
  });
}

const generator = {
  generateAll: generateAll,

  generatedRoot: generatedRoot,

  generateForPackage(pkgName) {
    installedPackagesRoot.forEach((packageRoot) => {
      let packagePath = path.join(packageRoot, 'share', pkgName);

      // eslint-disable-next-line
      if (fs.existsSync(packagePath)) {
        findPackagesInDirectory(packagePath).then((pkg) => {
        });
      }
    });
  },

  isRootGererated() {
    // eslint-disable-next-line
    return fs.existsSync(generatedRoot);
  },

  version() {
    // eslint-disable-next-line
    return JSON.parse(fs.readFileSync(path.join(__dirname, 'generator.json'), 'utf8')).version;
  }
};

module.exports = generator;
