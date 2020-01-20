/* eslint-disable max-depth */
/* eslint-disable no-sync */
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


// create interfaces.d.ts containing from each typeclass definition
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
const loader = require('../lib/interface_loader.js');


function generateAll() {
  // load pkg and interface info (msgs and srvs)
  const generatedPath = path.join(__dirname, '../generated/');
  const pkgInfos = getPkgInfos(generatedPath);

  // write message.d.ts file
  const messagesFilePath = path.join(__dirname, '../types/interfaces.d.ts');
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

    const pkgInfo = {
      name: pkg,
      messages: []
    };

    const pkgPath = path.join(rootDir, pkg);
    const files = fs.readdirSync(pkgPath);

    for (let filename of files) {
      const typeClass = fileName2Typeclass(filename);

      if (typeClass.type === 'srv') { // skip __srv__<action>
        if (!typeClass.name.endsWith('Request') && !typeClass.name.endsWith('Response')) {
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

  let fullMessageNames = ['string'];

  fs.writeSync(fd, '/* eslint-disable camelcase */\n');
  fs.writeSync(fd, '// DO NOT EDIT\n');
  fs.writeSync(fd, '// This file is generated by the rostsd_gen script\n\n');

  const moduleDeclare = 'declare module \'rclnodejs\' {\n';
  fs.writeSync(fd, moduleDeclare);

  for (const pkgInfo of pkgInfos) {

    // write namespaces heirarchy for package
    const pkgNamespaceTemplate =
      `  namespace ${pkgInfo.name} {
`;
    fs.writeSync(fd, pkgNamespaceTemplate);

    let curNS = null;
    for (const msgInfo of pkgInfo.messages) {

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

      // full path to this msg
      const fullMessageName = `${pkgInfo.name}.${msgInfo.typeClass.type}.${msgInfo.typeClass.name}`;
      fullMessageNames.push(fullMessageName);
    }

    if (curNS) {
      // close msg level namespace declare
      fs.writeSync(fd, '    }\n');
    }

    // close pkg level namespace declare
    fs.writeSync(fd, '  }\n\n');
  }

  // write type alias for Message
  // e.g. type Message =
  //             string |
  //             std_msgs.msg.Bool |
  //             std_msgs.msg.Byte |
  //             ...
  fs.writeSync(fd, '  type Message = \n');
  for (let i=0; i < fullMessageNames.length; i++) {
    fs.writeSync(fd, '    ' +  fullMessageNames[i]);
    if (i != fullMessageNames.length-1) {
      fs.writeSync(fd, ' |\n');
    }
  }
  fs.writeSync(fd, ';\n\n');

  const fullMessageNameStrings = [];
  for (const messageName of fullMessageNames) {
    fullMessageNameStrings.push(messageName.replace(/\./g, '/'));
  }

  // write message type class string
  // e.g. type MessageTypeClassStr =
  //             'string' |
  //             'std_msgs/msg/Bool' |
  //             'std_msgs/msg/Byte' |
  //             ...
  fs.writeSync(fd, '  type MessageTypeClassStr = \n');
  for (let i=0; i < fullMessageNameStrings.length; i++) {
    fs.writeSync(fd, `    '${fullMessageNameStrings[i]}'`);
    if (i != fullMessageNames.length-1) {
      fs.writeSync(fd, ' |\n');
    }
  }
  fs.writeSync(fd, ';\n');

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
  // for (let i = 0; i < msgInfo.def.constants.length; i++) {
  //   const constant = msgInfo.def.constants[i];
  //   const constantType = primitiveType2JSName(constant.type);
  //   const tmpl = (constantType == 'string') ?
  //     `        readonly ${constant.name}?: '${constant.value}'` :
  //     `        readonly ${constant.name}?: ${constant.value}`;
  //   fs.writeSync(fd, tmpl);

  //   if (i != msgInfo.def.constants.length - 1) {
  //     fs.writeSync(fd, ',\n');
  //   } else if (msgInfo.def.fields.length > 0) {
  //     fs.writeSync(fd, ',\n');
  //   }
  // }

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

  // write wrapper interfaces
  if (msgInfo.def.constants.length) {
    fs.writeSync(fd, `      export interface ${msgInfo.typeClass.name}Wrapper {\n`);
    for (const constant of msgInfo.def.constants) {
      const constantType = primitiveType2JSName(constant.type);
      const tmpl = `        readonly ${constant.name}: ${constantType};\n`;
      fs.writeSync(fd, tmpl);
    }
    const ctorTmpl = `        new(other?: ${msgInfo.typeClass.name}): ${msgInfo.typeClass.name}\n`;
    fs.writeSync(fd, ctorTmpl);
    fs.writeSync(fd, '      }\n');
  }
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


const tsdGenerator = {
  generateAll
};

module.exports = tsdGenerator;
