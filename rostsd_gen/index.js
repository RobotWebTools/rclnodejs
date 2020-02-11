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
      messages: [],
      services: []
    };

    const pkgPath = path.join(rootDir, pkg);
    const files = fs.readdirSync(pkgPath).filter(fn => fn.endsWith('.js'));

    for (let filename of files) {
      const typeClass = fileName2Typeclass(filename);

      if (typeClass.type === 'srv') { // skip __srv__<action>
        if (!typeClass.name.endsWith('Request') && !typeClass.name.endsWith('Response')) {
          pkgInfo.services.push(typeClass);
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
  let messagesMap = {
    string: 'string',
  };

  fs.writeSync(fd, '/* eslint-disable camelcase */\n');
  fs.writeSync(fd, '/* eslint-disable max-len */\n');
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
      saveMsgWrapperAsTSD(msgInfo, fd);

      // full path to this msg
      const fullMessageName = `${pkgInfo.name}/${msgInfo.typeClass.type}/${msgInfo.typeClass.name}`;
      messagesMap[fullMessageName] = `${pkgInfo.name}.${msgInfo.typeClass.type}.${msgInfo.typeClass.name}`;
    }

    if (curNS) {
      // close msg level namespace declare
      fs.writeSync(fd, '    }\n');
    }

    // close pkg level namespace declare
    fs.writeSync(fd, '  }\n\n');
  }

  // write messages type mappings
  fs.writeSync(fd, '  type MessagesMap = {\n');
  for (const key in messagesMap) {
    fs.writeSync(fd, `    '${key}': ${messagesMap[key]},\n`);
  }
  fs.writeSync(fd, '  };\n');
  fs.writeSync(fd, '  type MessageTypeClassName = keyof MessagesMap;\n');
  fs.writeSync(fd, '  type Message = MessagesMap[MessageTypeClassName];\n');
  fs.writeSync(fd, '  type MessageType<T> = T extends MessageTypeClassName ? MessagesMap[T] : object;\n\n');

  // write message wrappers mappings
  fs.writeSync(fd, '  type MessageTypeClassWrappersMap = {\n');
  for (const key in messagesMap) {
    if (key === 'string') {
      fs.writeSync(fd, "    'string': never,\n");
      continue;
    }
    fs.writeSync(fd, `    '${key}': ${messagesMap[key]}_Wrapper,\n`);
  }
  fs.writeSync(fd, '  };\n');
  fs.writeSync(fd, '  type MessageWrapperType<T> = T extends MessageTypeClassName ? MessageTypeClassWrappersMap[T] : object;\n\n');

  // write service type class string
  const services = [];
  for (const pkg of pkgInfos) {
    services.push(...pkg.services);
  }
  if (!services.length) {
    fs.writeSync(fd, '  type ServiceTypeClassName = never;\n\n');
  } else {
    fs.writeSync(fd, '  type ServiceTypeClassName = \n');
    for (let i = 0; i < services.length; i++) {
      const srv = services[i];
      const srvTypeClassStr = `${srv.package}/${srv.type}/${srv.name}`;
      fs.writeSync(fd, `    '${srvTypeClassStr}'`);

      if (i !== services.length - 1) {
        fs.writeSync(fd, ' |\n');
      }
    }
    fs.writeSync(fd, ';\n\n');
  }

  fs.writeSync(fd, '  type TypeClassName = MessageTypeClassName | ServiceTypeClassName;\n');

  // close module declare
  fs.writeSync(fd, '}\n');

  fs.closeSync(fd);
}


function saveMsgWrapperAsTSD(msgInfo, fd) {
  const msgName = msgInfo.typeClass.name;
  fs.writeSync(fd, `      export type ${msgName}_Wrapper = {\n`);
  for (const constant of msgInfo.def.constants) {
    const constantType = primitiveType2JSName(constant.type);
    fs.writeSync(fd, `        ${constant.name}: ${constantType},\n`);
  }
  fs.writeSync(fd, `        new(other?: ${msgName}): ${msgName},\n`);
  fs.writeSync(fd, '      }\n');
}


function saveMsgConstantsAsTSD(msgInfo, fd) {
  if (!msgInfo.def.constants.length) {
    return;
  }
  fs.writeSync(fd, `      export const enum ${msgInfo.typeClass.name}_Constants {\n`);
  for (const constant of msgInfo.def.constants) {
    const constantType = primitiveType2JSName(constant.type);
    let value = undefined;
    switch (constantType) {
    case 'string':
      value = `'${constant.value}'`;
      break;
    case 'boolean':
      // true and false are not allowed in const enums for some reason
      value = constant.value ? 1 : 0;
      break;
    default:
      value = constant.value;
    }
    fs.writeSync(fd, `        ${constant.name} = ${value},\n`);
  }
  fs.writeSync(fd, '      }\n');
}


/**
 * Writes the message fields as typescript definitions.
 *
 * @param {*} msgInfo ros message info
 * @param {*} fd file descriptor
 * @param {string} indent The amount of indent, in spaces
 * @param {string} lineEnd The character to put at the end of each line, usually ','
 * or ';'
 * @param {string} typePrefix The prefix to put before the type name for
 * non-primitive types
 * @returns {undefined}
 */
function saveMsgFieldsAsTSD(msgInfo, fd, indent=0, lineEnd=',', typePrefix='') {
  const indentStr = ' '.repeat(indent);
  for (let i = 0; i < msgInfo.def.fields.length; i++) {
    const field = msgInfo.def.fields[i];
    let fieldType = fieldType2JSName(field);
    let tp = field.type.isPrimitiveType ? '' : typePrefix;
    if (typePrefix === 'rclnodejs.') {
      fieldType = 'any';
      tp = '';
    }
    const tmpl = `${indentStr}${field.name}: ${tp}${fieldType}`;
    fs.writeSync(fd, tmpl);
    if (field.type.isArray) {
      fs.writeSync(fd, '[]');
    }
    fs.writeSync(fd, lineEnd);
    fs.writeSync(fd, '\n');
  }
}


function saveMsgInfoAsTSD(msgInfo, fd) {
  saveMsgConstantsAsTSD(msgInfo, fd);

  // write type = xxxx {
  const typeTemplate =
    `      export type ${msgInfo.typeClass.name} = {\n`;

  fs.writeSync(fd, typeTemplate);

  // write field definitions
  saveMsgFieldsAsTSD(msgInfo, fd, 8);

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
  case 'uint8':
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
