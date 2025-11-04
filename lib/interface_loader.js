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
const fs = require('fs');
const generator = require('../rosidl_gen/index.js');
const { TypeValidationError, ValidationError } = require('./errors.js');

let interfaceLoader = {
  loadInterfaceByObject(obj) {
    //
    // `obj` param structure
    //
    // {
    //   package: 'std_msgs',
    //   type: 'msg',
    //   name: 'String',
    // }
    //
    if (
      typeof obj !== 'object' ||
      !obj ||
      !obj.package ||
      !obj.type ||
      !obj.name
    ) {
      throw new TypeValidationError(
        'interfaceObject',
        obj,
        'object with package, type, and name properties',
        {
          entityType: 'interface loader',
        }
      );
    }
    return this.loadInterface(obj.package, obj.type, obj.name);
  },

  loadInterfaceByString(name) {
    if (typeof name !== 'string') {
      throw new TypeValidationError('name', name, 'string', {
        entityType: 'interface loader',
      });
    }

    // TODO(Kenny): more checks of the string argument
    if (name.indexOf('/') !== -1) {
      let [packageName, type, messageName] = name.split('/');
      return this.loadInterface(packageName, type, messageName);
    }

    // Suppose the name is a package, and traverse the path to collect the IDL files.
    let packagePath = path.join(generator.generatedRoot, name);

    let interfaces = fs.readdirSync(packagePath);
    if (interfaces.length > 0) {
      return this.loadInterfaceByPath(packagePath, interfaces);
    }

    throw new ValidationError(
      'A string argument in expected in "package/type/message" format',
      {
        code: 'INVALID_INTERFACE_FORMAT',
        argumentName: 'name',
        providedValue: name,
        expectedType: 'string in "package/type/message" format',
        entityType: 'interface loader',
      }
    );
  },

  loadInterfaceByPath(packagePath, interfaces) {
    let interfaceInfos = [];

    interfaces.forEach((file) => {
      let results = file.match(/\w+__(\w+)__(\w+).js$/);
      let type = results[1];
      let name = results[2];
      let filePath = path.join(packagePath, file).normalize();
      interfaceInfos.push({ name, type, filePath });
    });

    let pkg = { srv: {}, msg: {}, action: {} };
    interfaceInfos.forEach((info) => {
      Object.defineProperty(pkg[info.type], info.name, {
        value: require(info.filePath),
      });
    });

    return pkg;
  },

  _isRos2InstallationPath(pkgPath) {
    // Use "which ros2" to dynamically find the ROS2 installation root
    try {
      const whichResult = require('child_process').spawnSync(
        'which',
        ['ros2'],
        {
          encoding: 'utf8',
          timeout: 5000,
        }
      );

      if (whichResult.status === 0 && whichResult.stdout) {
        const ros2BinPath = whichResult.stdout.trim();
        // Get the ROS2 installation root (typically /opt/ros/<distro> or similar)
        const ros2Root = path.dirname(path.dirname(ros2BinPath));

        return pkgPath.includes(ros2Root);
      }
    } catch (err) {
      console.error('Error running which ros2:', err.message);
      // If "which ros2" fails, fall back to hardcoded check
      return pkgPath.includes('ros2-linux');
    }

    return false;
  },

  _searchAndGenerateInterface(packageName, type, messageName, filePath) {
    // Check if it's a valid package
    for (const pkgPath of generator.getInstalledPackagePaths()) {
      // We are going to ignore the path where ROS2 is installed.
      if (this._isRos2InstallationPath(pkgPath)) {
        continue;
      }

      // Recursively search for files named messageName.* under pkgPath/
      if (fs.existsSync(pkgPath)) {
        // Recursive function to search for files
        function searchForFile(dir) {
          try {
            const items = fs.readdirSync(dir, { withFileTypes: true });
            for (const item of items) {
              const fullPath = path.join(dir, item.name);

              if (item.isFile()) {
                const baseName = path.parse(item.name).name;
                // Check if the base filename matches messageName
                if (baseName === messageName) {
                  return fullPath;
                }
              } else if (item.isDirectory()) {
                // Recursively search subdirectories
                const result = searchForFile(fullPath);
                if (result) {
                  return result;
                }
              }
            }
          } catch (err) {
            // Skip directories we can't read
            console.error('Error reading directory:', dir, err.message);
          }
          return null;
        }

        const foundFilePath = searchForFile(
          path.join(pkgPath, 'share', packageName)
        );

        if (foundFilePath && foundFilePath.length > 0) {
          // Use worker thread to generate interfaces synchronously
          try {
            generator.generateInPathSyncWorker(pkgPath);
            // Now try to load the interface again from the generated files
            if (fs.existsSync(filePath)) {
              return require(filePath);
            }
          } catch (err) {
            console.error('Error in interface generation:', err);
          }
        }
      }
    }
    throw new ValidationError(
      `The message required does not exist: ${packageName}, ${type}, ${messageName} at ${generator.generatedRoot}`,
      {
        code: 'MESSAGE_NOT_FOUND',
        entityType: 'interface loader',
        details: {
          packageName: packageName,
          type: type,
          messageName: messageName,
          searchPath: generator.generatedRoot,
        },
      }
    );
  },

  loadInterface(packageName, type, messageName) {
    if (arguments.length === 1) {
      const type = arguments[0];
      if (typeof type === 'object') {
        return this.loadInterfaceByObject(type);
      } else if (typeof type === 'string') {
        return this.loadInterfaceByString(type);
      }
      throw new ValidationError(
        `The message required does not exist: ${type}`,
        {
          code: 'MESSAGE_NOT_FOUND',
          entityType: 'interface loader',
          details: { type: type },
        }
      );
    }
    if (packageName && type && messageName) {
      let filePath = path.join(
        generator.generatedRoot,
        packageName,
        packageName + '__' + type + '__' + messageName + '.js'
      );

      if (fs.existsSync(filePath)) {
        return require(filePath);
      } else {
        return this._searchAndGenerateInterface(
          packageName,
          type,
          messageName,
          filePath
        );
      }
    }
    // We cannot parse `packageName`, `type` and `messageName` from the string passed.
    throw new ValidationError(
      `The message required does not exist: ${packageName}, ${type}, ${messageName} at ${generator.generatedRoot}`,
      {
        code: 'MESSAGE_NOT_FOUND',
        entityType: 'interface loader',
        details: {
          packageName: packageName,
          type: type,
          messageName: messageName,
          searchPath: generator.generatedRoot,
        },
      }
    );
  },
};

module.exports = interfaceLoader;
