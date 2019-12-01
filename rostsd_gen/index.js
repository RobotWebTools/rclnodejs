/* eslint-disable camelcase */
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


// create messages.d.ts containing from each typeclass definition
// - write header txt into outfile
// - find each pkg
// --  create typescript namespace
// --  for each pkg msg file
// ---   create a typescript type definition
/* Example output for std_msgs_msg_String
declare module "rclnodejs" {
  namespace std_msgs {
	  namespace msg {
			export type String = {
				data: string
			}
		}
	}
}
*/

'use strict';

const path = require('path');
const fs = require('fs');
const rclnodejs = require('../index.js');
const loader = require('../lib/interface_loader.js');


function generateAll() {
  // load pkg and interface info (msgs and srvs)
  const generatedPath = path.join(__dirname, '../generated/');
  const pkgInfos = getPkgInfos(generatedPath);
  console.log('pkg count:', pkgInfos.length);

  // write message.d.ts file
  const messagesFilePath = path.join(__dirname, '../@types/messages.d.ts');
  const fd = fs.openSync(messagesFilePath, 'w');
  savePkgInfoAsTSD(pkgInfos, fd);
}

// scan generated files, collect pkg and msg info
function getPkgInfos(generatedRoot) {
  let pkgInfos = [];
  const rootDir = generatedRoot;
  let pkgs = fs.readdirSync(rootDir);

  for (let pkg of pkgs) {

    if (pkg.endsWith('.json')) continue;
    if (pkg.startsWith('actionlib')) continue;

    console.log('processing pkg', pkg);

    const pkgInfo = {
      name: pkg,
      messages: []
    };

    const pkgPath = path.join(rootDir, pkg);
    const files = fs.readdirSync(pkgPath);

    for (let filename of files) {
      const typeClass = fileName2Typeclass(filename);

      if (typeClass.type === 'srv') { // skip __srv__<action>
        if (!typeClass.name.endsWith('Request') || !typeClass.name.endsWith('Response')) {
          continue;
        }
      }

      const msg = createMessage(typeClass);
      const type = msg.constructor.type();
      const def = msg.constructor.ROSMessageDef;

      const msgInfo = {
        name: def.msgName,
        typeClass: typeClass,
        type: type,
        def: def
      };

      pkgInfo.messages.push(msgInfo);
    }

    pkgInfos.push(pkgInfo);
  };

  return pkgInfos;
}


function savePkgInfoAsTSD(pkgInfos, fd) {

  fs.writeSync(fd, '// DO NOT EDIT\n');
  fs.writeSync(fd, '// This file is generated by the rostsd_gen script\n\n');

  const moduleDeclare = 'declare module "rclnodejs" {\n';
  fs.writeSync(fd, moduleDeclare);

  for (const pkgInfo of pkgInfos) {
    // write namespaces heirarchy for package
    const pkgNamespaceTemplate =
      `  namespace ${pkgInfo.name} {
`;
    fs.writeSync(fd, pkgNamespaceTemplate);

    let curNS = null;
    for (const msgInfo of pkgInfo.messages) {

      // omit namespaces that are not 'msg' or 'srv'
      if (msgInfo.typeClass.type.startsWith('action')) continue;

      if (msgInfo.typeClass.type != curNS) {
        if (curNS) { // close current ns 
          fs.writeSync(fd, '    }\n');
        }

        curNS = msgInfo.typeClass.type;

        // write namespaces heirarchy for package
        const msgNamespaceTemplate =
          `    namespace ${curNS} {
`;
        fs.writeSync(fd, msgNamespaceTemplate);
      }

      saveMsgInfoAsTSD(msgInfo, fd);
    }

    if (curNS) {
      // close msg level namespace declare
      fs.writeSync(fd, '    }\n');
    }

    // close pkg level namespace declare
    fs.writeSync(fd, '  }\n\n');
  }

  // close module declare
  fs.writeSync(fd, '}\n');
  fs.closeSync(fd);
}


function saveMsgInfoAsTSD(msgInfo, fd) {

  // write type = xxxx {
  const typeTemplate =
    `      export type ${msgInfo.typeClass.name} = {\n`;

  fs.writeSync(fd, typeTemplate);

  // write constant definitions
  for (let i = 0; i < msgInfo.def.constants.length; i++) {
    const constant = msgInfo.def.constants[i];
    const constantType = primitiveType2JSName(constant.type);
    const tmpl = (constantType == 'string') ?
      `        ${constant.name}: '${constant.value}'` :
      `        ${constant.name}: ${constant.value}`;
    fs.writeSync(fd, tmpl);

    if (i != msgInfo.def.constants.length - 1) {
      fs.writeSync(fd, ',\n');
    } else if (msgInfo.def.fields.length > 0) {
      fs.writeSync(fd, ',\n');
    }
  }

  // write field definitions
  for (let i = 0; i < msgInfo.def.fields.length; i++) {
    const field = msgInfo.def.fields[i];
    const fieldType = fieldType2JSName(field);
    const tmpl = `        ${field.name}: ${fieldType}`;
    fs.writeSync(fd, tmpl);
    if (field.type.isArray) {
      fs.writeSync(fd, '[]');
    }
    if (i != msgInfo.def.fields.length - 1) {
      fs.writeSync(fd, ',');
    }
    fs.writeSync(fd, '\n');
  }

  // end of def
  fs.writeSync(fd, '      };\n');
}


function fieldType2JSName(fieldInfo) {
  return fieldInfo.type.isPrimitiveType ?
    primitiveType2JSName(fieldInfo.type.type) :
    fieldInfo.type.pkgName + '.msg.' + fieldInfo.type.type;
}


function primitiveType2JSName(type) {
  let jsName;

  switch (type) {
  case 'char':
  case 'byte':
  case 'uin8':
  case 'int8':
  case 'int16':
  case 'uint16':
  case 'int32':
  case 'uint32':
  case 'int64':
  case 'uint64':
  case 'float32':
  case 'float64':
  case 'double':
    jsName = 'number';
    break;
  case 'bool':
    jsName = 'boolean';
    break;
  case 'string':
  case 'wstring':
    jsName = 'string';
    break;
  }

  return jsName;
}


// example filename: std_msgs_msg_String, sensor_msgs_msg_LaserScan
// result {package: 'std_msgs', type: 'msg', name: 'String'}
function fileName2Typeclass(filename) {
  const regex = /(.+)__(\w+)__(\w+)\.js/;
  const array = filename.split(regex).filter(Boolean);

  if (!array || array.length != 3) {
    // todo: throw error 
    console.log('ERRORRROOROR', array);
    return;
  }

  return {
    package: array[0],
    type: array[1],
    name: array[2]
  };
}


function createMessage(type) {
  let typeClass = loader.loadInterface(type);
  return typeClass ?
    new typeClass() :
    undefined;
}


const tsd_generator = {
  // version() {
  //   // eslint-disable-next-line
  //   return fse.readJsonSync(path.join(__dirname, 'generator.json')).version;
  // },

  generateAll
};

module.exports = tsd_generator;

/*
dev notes -------------------------------
namespace sensor_msgs {

  namespace msg {

    type LaserScan = {
      header: std_msgs.msg.Header;
    }

  }
}


 msgInfo:
 {
  name: string,
  typeClass: typeClass, // {package, type, name}
  type: type,           // {pkgName: 'std_msgs', subFolder: 'msg', interfaceName: 'String'};
  def: def
 }

 pkgInfo:
 {
   name: string;
   messages: Array<msgInfo>
 }

 Example Spec object from parser

 {
  "constants": [
    {
      "type": "uint8",
      "name": "PRIMARY_STATE_UNKNOWN",
      "value": 0
    },
    {
      "type": "uint8",
      "name": "PRIMARY_STATE_UNCONFIGURED",
      "value": 1
    }
  ],
  "fields": [
    {
      "name": "header",
      "type": {
        "isArray": false,
        "arraySize": null,
        "isUpperBound": false,
        "isDynamicArray": false,
        "isFixedSizeArray": false,
        "pkgName": "std_msgs",
        "type": "Header",
        "stringUpperBound": null,
        "isPrimitiveType": false
      },
      "default_value": null
    },
    {
      "name": "angle_min",
      "type": {
        "isArray": false,
        "arraySize": null,
        "isUpperBound": false,
        "isDynamicArray": false,
        "isFixedSizeArray": false,
        "pkgName": null,
        "type": "float32",
        "stringUpperBound": null,
        "isPrimitiveType": true
      },
      "default_value": null
    },
    {
      "name": "angle_max",
      "type": {
        "isArray": false,
        "arraySize": null,
        "isUpperBound": false,
        "isDynamicArray": false,
        "isFixedSizeArray": false,
        "pkgName": null,
        "type": "float32",
        "stringUpperBound": null,
        "isPrimitiveType": true
      },
      "default_value": null
    },
    {
      "name": "angle_increment",
      "type": {
        "isArray": false,
        "arraySize": null,
        "isUpperBound": false,
        "isDynamicArray": false,
        "isFixedSizeArray": false,
        "pkgName": null,
        "type": "float32",
        "stringUpperBound": null,
        "isPrimitiveType": true
      },
      "default_value": null
    },
    {
      "name": "time_increment",
      "type": {
        "isArray": false,
        "arraySize": null,
        "isUpperBound": false,
        "isDynamicArray": false,
        "isFixedSizeArray": false,
        "pkgName": null,
        "type": "float32",
        "stringUpperBound": null,
        "isPrimitiveType": true
      },
      "default_value": null
    },
    {
      "name": "scan_time",
      "type": {
        "isArray": false,
        "arraySize": null,
        "isUpperBound": false,
        "isDynamicArray": false,
        "isFixedSizeArray": false,
        "pkgName": null,
        "type": "float32",
        "stringUpperBound": null,
        "isPrimitiveType": true
      },
      "default_value": null
    },
    {
      "name": "range_min",
      "type": {
        "isArray": false,
        "arraySize": null,
        "isUpperBound": false,
        "isDynamicArray": false,
        "isFixedSizeArray": false,
        "pkgName": null,
        "type": "float32",
        "stringUpperBound": null,
        "isPrimitiveType": true
      },
      "default_value": null
    },
    {
      "name": "range_max",
      "type": {
        "isArray": false,
        "arraySize": null,
        "isUpperBound": false,
        "isDynamicArray": false,
        "isFixedSizeArray": false,
        "pkgName": null,
        "type": "float32",
        "stringUpperBound": null,
        "isPrimitiveType": true
      },
      "default_value": null
    },
    {
      "name": "ranges",
      "type": {
        "isArray": true,
        "arraySize": null,
        "isUpperBound": false,
        "isDynamicArray": true,
        "isFixedSizeArray": null,
        "pkgName": null,
        "type": "float32",
        "stringUpperBound": null,
        "isPrimitiveType": true
      },
      "default_value": null
    },
    {
      "name": "intensities",
      "type": {
        "isArray": true,
        "arraySize": null,
        "isUpperBound": false,
        "isDynamicArray": true,
        "isFixedSizeArray": null,
        "pkgName": null,
        "type": "float32",
        "stringUpperBound": null,
        "isPrimitiveType": true
      },
      "default_value": null
    }
  ],
  "baseType": {
    "pkgName": "sensor_msgs",
    "type": "LaserScan",
    "stringUpperBound": null,
    "isPrimitiveType": false
  },
  "msgName": "LaserScan"
}
*/
