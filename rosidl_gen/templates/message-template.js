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

'use strict';

/**
 * Generate JavaScript message class code using native template literals
 * This replaces the doT template engine with pure JavaScript
 */

// Helper functions (same as in message.dot template)
function getPrimitiveNameByType(type) {
  if (type.type === 'bool') {
    return 'Bool';
  } else if (type.type === 'int8') {
    return 'Int8';
  } else if (type.type === 'uint8') {
    return 'UInt8';
  } else if (type.type === 'int16') {
    return 'Int16';
  } else if (type.type === 'uint16') {
    return 'UInt16';
  } else if (type.type === 'int32') {
    return 'Int32';
  } else if (type.type === 'uint32') {
    return 'UInt32';
  } else if (type.type === 'int64') {
    return 'Int64';
  } else if (type.type === 'uint64') {
    return 'UInt64';
  } else if (type.type === 'float64') {
    return 'Float64';
  } else if (type.type === 'float32') {
    return 'Float32';
  } else if (type.type === 'char') {
    return 'Char';
  } else if (type.type === 'byte') {
    return 'Byte';
  } else if (type.type === 'string' || type.type === 'wstring') {
    return 'String';
  } else {
    return '';
  }
}

function getTypedArrayName(type) {
  const t = type.type.toLowerCase();
  let typedArrayName = null;

  switch (t) {
    case 'byte':
    case 'octet':
    case 'uint8':
      typedArrayName = 'Uint8Array';
      break;
    case 'char':
    case 'int8':
      typedArrayName = 'Int8Array';
      break;
    case 'int16':
    case 'short':
      typedArrayName = 'Int16Array';
      break;
    case 'uint16':
    case 'unsigned short':
      typedArrayName = 'Uint16Array';
      break;
    case 'int32':
    case 'long':
      typedArrayName = 'Int32Array';
      break;
    case 'uint32':
    case 'unsigned long':
      typedArrayName = 'Uint32Array';
      break;
    case 'float':
    case 'float32':
      typedArrayName = 'Float32Array';
      break;
    case 'double':
    case 'float64':
      typedArrayName = 'Float64Array';
      break;
    default:
      typedArrayName = '';
  }
  return typedArrayName;
}

function getTypedArrayElementName(type) {
  const t = type.type.toLowerCase();
  if (t === 'int8') {
    return 'int8';
  } else if (t === 'uint8') {
    return 'uint8';
  } else if (t === 'int16') {
    return 'int16';
  } else if (t === 'uint16') {
    return 'uint16';
  } else if (t === 'int32') {
    return 'int32';
  } else if (t === 'uint32') {
    return 'uint32';
  } else if (t === 'float64') {
    return 'double';
  } else if (t === 'float32') {
    return 'float';
  } else if (t === 'char') {
    return 'int8';
  } else if (t === 'byte') {
    return 'uint8';
  } else {
    return '';
  }
}

const primitiveBaseType = [
  'Bool',
  'Int8',
  'UInt8',
  'Int16',
  'UInt16',
  'Int32',
  'UInt32',
  'Int64',
  'UInt64',
  'Float64',
  'Float32',
  'Char',
  'Byte',
  'String',
];

const typedArrayType = [
  'int8',
  'uint8',
  'int16',
  'uint16',
  'int32',
  'uint32',
  'float64',
  'float32',
  'char',
  'byte',
];

function isPrimitivePackage(baseType) {
  return primitiveBaseType.indexOf(baseType.type) !== -1;
}

function isTypedArrayType(type) {
  return typedArrayType.indexOf(type.type.toLowerCase()) !== -1;
}

function isBigInt(type) {
  return ['int64', 'uint64'].indexOf(type.type.toLowerCase()) !== -1;
}

function getWrapperNameByType(type) {
  if (type.isPrimitiveType) {
    return getPrimitiveNameByType(type) + 'Wrapper';
  } else {
    return type.type + 'Wrapper';
  }
}

function getModulePathByType(type, messageInfo) {
  if (type.isPrimitiveType) {
    return 'std_msgs__msg__' + getPrimitiveNameByType(type) + '.js';
  }

  if (
    messageInfo &&
    messageInfo.subFolder === 'action' &&
    messageInfo.pkgName === type.pkgName &&
    (type.type.endsWith('_Goal') ||
      type.type.endsWith('_Result') ||
      type.type.endsWith('_Feedback'))
  ) {
    return type.pkgName + '__action__' + type.type + '.js';
  }

  /* We should use '__msg__' to require "service_msgs/msg/ServiceEventInfo.msg" for service event message. */
  if (
    messageInfo &&
    messageInfo.isServiceEvent &&
    type.type !== 'ServiceEventInfo'
  ) {
    return type.pkgName + `__${messageInfo.subFolder}__` + type.type + '.js';
  }
  return type.pkgName + '__msg__' + type.type + '.js';
}

function getPackageNameByType(type) {
  if (type.isPrimitiveType) {
    return 'std_msgs';
  } else {
    return type.pkgName;
  }
}

function extractMemberNames(fields) {
  let memberNames = [];
  fields.forEach((field) => {
    memberNames.push(field.name);
  });
  return JSON.stringify(memberNames);
}

/**
 * Main template generation function
 * @param {Object} data - Template data
 * @param {Object} data.spec - Message specification
 * @param {Object} data.messageInfo - Message information (pkgName, subFolder, interfaceName, isServiceEvent)
 * @param {string} data.json - JSON string of spec
 * @param {boolean} data.isDebug - Debug flag
 * @returns {string} Generated JavaScript code
 */
function generateMessage(data) {
  const spec = data.spec;
  const messageInfo = data.messageInfo;
  const json = data.json;
  const isDebug = data.isDebug;

  // Initialize variables
  const objectWrapper = spec.msgName + 'Wrapper';
  const arrayWrapper = spec.msgName + 'ArrayWrapper';
  const refObjectType = spec.msgName + 'RefStruct';
  const refObjectArrayType = spec.msgName + 'RefStructArray';
  const refArrayType = spec.msgName + 'RefArray';
  const compactMsgDefJSON = JSON.stringify(JSON.parse(json));

  // Add dummy field if empty (following rosidl_generator_c style)
  if (spec.fields.length === 0) {
    spec.isEmpty = true;
    spec.fields.push({
      type: {
        isArray: false,
        pkgName: null,
        type: 'bool',
        isDynamicArray: false,
        stringUpperBound: null,
        isUpperBound: false,
        isPrimitiveType: true,
        isFixedSizeArray: false,
        arraySize: null,
      },
      name: '_dummy',
    });
  }

  // Check if will use TypedArray
  const willUseTypedArray = isTypedArrayType(spec.baseType);
  const currentTypedArray = getTypedArrayName(spec.baseType);
  const currentTypedArrayElementType = getTypedArrayElementName(spec.baseType);

  // Track required modules
  let existedModules = [];
  function shouldRequire(baseType, fieldType) {
    let requiredModule = `${getPackageNameByType(fieldType)}/${getModulePathByType(fieldType, messageInfo)}`;
    let shouldReq =
      !isPrimitivePackage(baseType) &&
      (fieldType.isArray ||
        !fieldType.isPrimitiveType ||
        fieldType.type === 'string');

    if (shouldReq && existedModules.indexOf(requiredModule) === -1) {
      existedModules.push(requiredModule);
      return true;
    } else {
      return false;
    }
  }

  // Start generating the code
  return `// This file is automatically generated by rclnodejs
//
// *** DO NOT EDIT directly
//

'use strict';

${willUseTypedArray ? "const rclnodejs = require('../../lib/native_loader.js');" : ''}
const ref = require('../../third_party/ref-napi');
const StructType = require('@rclnodejs/ref-struct-di')(ref);
const ArrayType = require('@rclnodejs/ref-array-di')(ref);
const primitiveTypes = require('../../rosidl_gen/primitive_types.js');
const deallocator = require('../../rosidl_gen/deallocator.js');
const translator = require('../../rosidl_gen/message_translator.js');

${spec.fields
  .map((field) => {
    if (shouldRequire(spec.baseType, field.type)) {
      return `const ${getWrapperNameByType(field.type)} = require('../../generated/${getPackageNameByType(field.type)}/${getModulePathByType(field.type, messageInfo)}');`;
    }
    return '';
  })
  .filter((line) => line)
  .join('\n')}

${
  spec.msgName === 'String'
    ? `const ${refObjectType} = primitiveTypes.string;`
    : `const ${refObjectType} = StructType({
${spec.fields
  .map((field) => {
    if (field.type.isPrimitiveType && !field.type.isArray) {
      return `  ${field.name}: primitiveTypes.${field.type.type},`;
    } else if (
      field.type.isPrimitiveType &&
      field.type.isArray &&
      field.type.isFixedSizeArray
    ) {
      return `  ${field.name}: ArrayType(primitiveTypes.${field.type.type}, ${field.type.arraySize}),`;
    } else if (
      !field.type.isPrimitiveType &&
      field.type.isArray &&
      field.type.isFixedSizeArray
    ) {
      return `  ${field.name}: ArrayType(${getWrapperNameByType(field.type)}.refObjectType, ${field.type.arraySize}),`;
    } else if (field.type.isArray) {
      return `  ${field.name}: ${getWrapperNameByType(field.type)}.refObjectArrayType,`;
    } else {
      return `  ${field.name}: ${getWrapperNameByType(field.type)}.refObjectType,`;
    }
  })
  .join('\n')}
});`
}

const ${refArrayType} = ArrayType(${refObjectType});
const ${refObjectArrayType} = StructType({
${
  willUseTypedArray
    ? `  data: ref.refType(ref.types.${currentTypedArrayElementType}),`
    : `  data: ${refArrayType},`
}
  size: ref.types.size_t,
  capacity: ref.types.size_t
});

${generateWrapperClass()}

${generateArrayWrapperClass()}

${generateConstants()}

module.exports = ${objectWrapper};

${isDebug ? `/*\n * The following is the original spec object coming from parser:\n${json}\n*/` : ''}
`;

  // Helper function to generate the wrapper class
  function generateWrapperClass() {
    return `class ${objectWrapper} {
  constructor(other) {
    this._initialize();
    this._setDefaults();

${
  spec.fields
    .map(
      (field) => `    if (typeof other === 'object' && other._refObject) {
      this._refObject = new ${refObjectType}(other._refObject.toObject());
${
  field.type.isArray
    ? `      this._wrapperFields.${field.name} = ${getWrapperNameByType(field.type)}.createArray();
      this._wrapperFields.${field.name}.copy(other._wrapperFields.${field.name});
${field.type.isPrimitiveType && !isTypedArrayType(field.type) ? `      this.${field.name} = other.${field.name};` : ''}`
    : !field.type.isPrimitiveType ||
        (field.type.type === 'string' && spec.msgName !== 'String')
      ? `      this._wrapperFields.${field.name} =  new ${getWrapperNameByType(field.type)}(other._wrapperFields.${field.name});`
      : ''
}`
    )
    .filter((line) => line.trim())
    .join('\n')[0] || ''
}    if (typeof other === 'object' && other._refObject) {
      this._refObject = new ${refObjectType}(other._refObject.toObject());
${spec.fields
  .map((field) => {
    if (field.type.isArray) {
      return `      this._wrapperFields.${field.name} = ${getWrapperNameByType(field.type)}.createArray();
      this._wrapperFields.${field.name}.copy(other._wrapperFields.${field.name});
${field.type.isPrimitiveType && !isTypedArrayType(field.type) ? `      this.${field.name} = other.${field.name};` : ''}`;
    } else if (
      !field.type.isPrimitiveType ||
      (field.type.type === 'string' && spec.msgName !== 'String')
    ) {
      return `      this._wrapperFields.${field.name} =  new ${getWrapperNameByType(field.type)}(other._wrapperFields.${field.name});`;
    }
    return '';
  })
  .filter((line) => line)
  .join('\n')}
    } else if (typeof other !== 'undefined') {
      translator.constructFromPlanObject(this, other);
    }
    this.freeze(/*own=*/false);
  }

  _setDefaults() {
${spec.fields
  .map((field) => {
    if (
      field.type.isPrimitiveType &&
      !field.type.isArray &&
      field.default_value
    ) {
      if (field.type.type === 'string' || field.type.type === 'wstring') {
        return `    this._refObject.${field.name} = "${field.default_value.replace(/"/g, '\\"')}";`;
      } else {
        return `    this._refObject.${field.name} = ${field.default_value};`;
      }
    } else if (
      field.type.isPrimitiveType &&
      !isTypedArrayType(field.type) &&
      field.default_value
    ) {
      if (isBigInt(field.type)) {
        return `    this._${field.name}Array = ${JSON.stringify(field.default_value)}.map(num => BigInt(num));`;
      } else {
        return `    this._${field.name}Array = ${JSON.stringify(field.default_value)};`;
      }
    } else if (
      field.type.isPrimitiveType &&
      isTypedArrayType(field.type) &&
      field.default_value
    ) {
      return `    this._wrapperFields.${field.name}.fill(${getTypedArrayName(field.type)}.from(${JSON.stringify(field.default_value)}));`;
    }
    return '';
  })
  .filter((line) => line)
  .join('\n')}
  }

  _initialize() {
    this._wrapperFields = {};
    this._refObject = new ${refObjectType}();
${spec.fields
  .map((field) => {
    if (field.type.isArray) {
      let result = `    this._wrapperFields.${field.name} = ${getWrapperNameByType(field.type)}.createArray();`;
      if (field.type.type === 'string' && field.type.isFixedSizeArray) {
        result += `\n    for (let i = 0; i < ${field.type.arraySize}; i++) {
      primitiveTypes.initString(this._refObject.${field.name}[i]);
    }`;
      }
      if (
        field.type.isArray &&
        field.type.isPrimitiveType &&
        !isTypedArrayType(field.type)
      ) {
        result += `\n    this._${field.name}Array = [];`;
      }
      return result;
    } else if (
      !field.type.isPrimitiveType ||
      (field.type.type === 'string' && spec.msgName !== 'String')
    ) {
      return `    this._wrapperFields.${field.name} = new ${getWrapperNameByType(field.type)}();`;
    } else if (spec.msgName === 'String') {
      return `    primitiveTypes.initString(this._refObject);`;
    }
    return '';
  })
  .filter((line) => line)
  .join('\n')}
  }

  static createFromRefObject(refObject) {
    let self = new ${objectWrapper}();
    self.copyRefObject(refObject);
    return self;
  }

  static createArray() {
    return new ${arrayWrapper}();
  }

  static get ArrayType() {
    return ${arrayWrapper};
  }

  static get refObjectArrayType() {
    return ${refObjectArrayType};
  }

  static get refObjectType() {
    return ${refObjectType};
  }

  toRawROS() {
    this.freeze(/*own=*/true);
    return this._refObject.ref();
  }

  freeze(own) {
${generateFreezeMethod()}
  }

  serialize() {
    this.freeze(/*own=*/false);
    return this._refObject.ref();
  }

  deserialize(refObject) {
${spec.fields
  .map((field) => {
    if (
      field.type.isArray &&
      field.type.isPrimitiveType &&
      field.type.isFixedSizeArray &&
      isTypedArrayType(field.type)
    ) {
      return `    this._wrapperFields.${field.name}.fill(refObject.${field.name}.toArray());`;
    } else if (
      field.type.isArray &&
      field.type.isPrimitiveType &&
      field.type.isFixedSizeArray &&
      !isTypedArrayType(field.type)
    ) {
      if (field.type.type === 'string') {
        return `    for (let index = 0; index < ${field.type.arraySize}; index++) {
      this._${field.name}Array[index] = refObject.${field.name}[index].data;
    }`;
      } else {
        return `    this._${field.name}Array = refObject.${field.name}.toArray();`;
      }
    } else if (
      field.type.isArray &&
      field.type.isPrimitiveType &&
      !isTypedArrayType(field.type)
    ) {
      return `    refObject.${field.name}.data.length = refObject.${field.name}.size;
    for (let index = 0; index < refObject.${field.name}.size; index++) {
      this._${field.name}Array[index] = refObject.${field.name}.data[index].data;
    }`;
    } else if (
      field.type.isArray &&
      !field.type.isPrimitiveType &&
      field.type.isFixedSizeArray
    ) {
      return `    this._refObject.${field.name} = refObject.${field.name};
    this._wrapperFields.${field.name}.size = ${field.type.arraySize}
    for (let i = 0; i < ${field.type.arraySize}; i++) {
      this._wrapperFields.${field.name}.data[i].copyRefObject(refObject.${field.name}[i]);
    }`;
    } else if (field.type.isArray || !field.type.isPrimitiveType) {
      return `    this._wrapperFields.${field.name}.copyRefObject(refObject.${field.name});`;
    } else if (field.type.type === 'string' && spec.msgName !== 'String') {
      return `    this._wrapperFields.${field.name}.data = refObject.${field.name}.data;`;
    } else if (field.type.isPrimitiveType) {
      return `    this._refObject.${field.name} = refObject.${field.name};`;
    }
    return '';
  })
  .filter((line) => line)
  .join('\n')}
  }

  toPlainObject(enableTypedArray) {
    return translator.toPlainObject(this, enableTypedArray);
  }

  static freeStruct(refObject) {
${generateFreeStructMethod()}
  }

  static destroyRawROS(msg) {
    ${objectWrapper}.freeStruct(msg.refObject);
  }

  static type() {
    return {pkgName: '${messageInfo.pkgName}', subFolder: '${messageInfo.subFolder}', interfaceName: '${messageInfo.interfaceName}'};
  }

  static isPrimitive() {
    return ${isPrimitivePackage(spec.baseType)};
  }

  static get isROSArray() {
    return false;
  }

  get refObject() {
    return this._refObject;
  }

${generateGettersSetters()}

  copyRefObject(refObject) {
    this._refObject = new ${refObjectType}(refObject.toObject());

${spec.fields
  .map((field) => {
    if (
      field.type.isArray &&
      field.type.isPrimitiveType &&
      !isTypedArrayType(field.type)
    ) {
      return `    refObject.${field.name}.data.length = refObject.${field.name}.size;
    for (let index = 0; index < refObject.${field.name}.size; index++) {
      this._${field.name}Array[index] = refObject.${field.name}.data[index].data;
    }`;
    } else if (
      field.type.isArray &&
      field.type.isPrimitiveType &&
      field.type.isFixedSizeArray
    ) {
      return `    this._wrapperFields.${field.name}.fill(refObject.${field.name}.toArray());`;
    } else if (
      field.type.isArray &&
      !field.type.isPrimitiveType &&
      field.type.isFixedSizeArray
    ) {
      return `    this._refObject.${field.name} = refObject.${field.name};
    this._wrapperFields.${field.name}.size = ${field.type.arraySize}
    for (let i = 0; i < ${field.type.arraySize}; i++) {
      this._wrapperFields.${field.name}.data[i].copyRefObject(refObject.${field.name}[i]);
    }`;
    } else if (
      !field.type.isPrimitiveType ||
      field.type.isArray ||
      (field.type.type === 'string' && spec.msgName !== 'String')
    ) {
      return `    this._wrapperFields.${field.name}.copyRefObject(this._refObject.${field.name});`;
    }
    return '';
  })
  .filter((line) => line)
  .join('\n')}
  }

  copy(other) {
    this._refObject = new ${refObjectType}(other._refObject.toObject());

${spec.fields
  .map((field) => {
    if (
      field.type.isArray &&
      field.type.isPrimitiveType &&
      !isTypedArrayType(field.type)
    ) {
      return `    this._${field.name}Array = other._${field.name}Array.slice();`;
    } else if (
      !field.type.isPrimitiveType ||
      field.type.isArray ||
      (field.type.type === 'string' && spec.msgName !== 'String')
    ) {
      return `    this._wrapperFields.${field.name}.copy(other._wrapperFields.${field.name});`;
    }
    return '';
  })
  .filter((line) => line)
  .join('\n')}
  }

  static get classType() {
    return ${objectWrapper};
  }

  static get ROSMessageDef() {
    return ${compactMsgDefJSON};
  }

  hasMember(name) {
    let memberNames = ${extractMemberNames(spec.fields)};
    return memberNames.indexOf(name) !== -1;
  }
}`;
  }

  // Helper function to generate freeze method
  function generateFreezeMethod() {
    return spec.fields
      .map((field) => {
        if (
          field.type.isArray &&
          field.type.isPrimitiveType &&
          !isTypedArrayType(field.type) &&
          field.type.isFixedSizeArray
        ) {
          if (field.type.type === 'string') {
            return `    for (let i = 0; i < ${field.type.arraySize}; i++) {
      if (own) {
        primitiveTypes.initString(this._refObject.${field.name}[i].ref(), own);
      } else {
        if (this._${field.name}Array.length === ${field.type.arraySize}) {
          const value = this._${field.name}Array[i];
          this._refObject.${field.name}[i].data = value;
          this._refObject.${field.name}[i].size = Buffer.byteLength(value);
          this._refObject.${field.name}[i].capacity = Buffer.byteLength(value) + 1;
        }
      }
    }`;
          } else if (isBigInt(field.type)) {
            return `    this._refObject.${field.name} = this._${field.name}Array.map(num => num.toString());`;
          } else {
            return `    this._refObject.${field.name} = this._${field.name}Array;`;
          }
        } else if (
          field.type.isArray &&
          field.type.isPrimitiveType &&
          isTypedArrayType(field.type) &&
          field.type.isFixedSizeArray
        ) {
          return `    this._refObject.${field.name} = Array.from(this._wrapperFields.${field.name}.data);`;
        } else if (
          field.type.isArray &&
          field.type.isPrimitiveType &&
          !isTypedArrayType(field.type)
        ) {
          return `    if (own) {
      this._wrapperFields.${field.name}.fill([]);
    } else {
      this._wrapperFields.${field.name}.fill(this._${field.name}Array);
    }
    this._wrapperFields.${field.name}.freeze(own);
    this._refObject.${field.name} = this._wrapperFields.${field.name}.refObject;`;
        } else if (
          field.type.isArray &&
          field.type.isPrimitiveType &&
          isTypedArrayType(field.type)
        ) {
          return `    if (own) {
      this._wrapperFields.${field.name}.fill(${getTypedArrayName(field.type)}.from([]));
    }
    this._wrapperFields.${field.name}.freeze(own);
    this._refObject.${field.name} = this._wrapperFields.${field.name}.refObject;`;
        } else if (
          field.type.isArray &&
          !field.type.isPrimitiveType &&
          field.type.isFixedSizeArray
        ) {
          return `    for (let i = 0; i < ${field.type.arraySize}; i++) {
      if (this._wrapperFields.${field.name}.data[i]) {
        this._wrapperFields.${field.name}.data[i].freeze(own);
        this._refObject.${field.name}[i] = this._wrapperFields.${field.name}.data[i].refObject;
      }
    }`;
        } else if (field.type.isArray || !field.type.isPrimitiveType) {
          let result = '';
          if (field.type.isArray) {
            result += `    if (own) {
      this._wrapperFields.${field.name}.fill([]);
    }
`;
          }
          result += `    this._wrapperFields.${field.name}.freeze(own);
    this._refObject.${field.name} = this._wrapperFields.${field.name}.refObject;`;
          return result;
        } else if (field.type.type === 'string' && spec.msgName !== 'String') {
          return `    if (own) {
      this._wrapperFields.${field.name}.freeze(own);
    }
    this._refObject.${field.name} = this._wrapperFields.${field.name}.refObject;`;
        } else if (spec.msgName === 'String') {
          return `    if (own) {
      primitiveTypes.initString(this._refObject.ref(), own);
    }`;
        }
        return '';
      })
      .filter((line) => line)
      .join('\n');
  }

  // Helper function to generate freeStruct method
  function generateFreeStructMethod() {
    return spec.fields
      .map((field) => {
        if (field.type.isArray && !field.type.isFixedSizeArray) {
          return `    if (refObject.${field.name}.size != 0) {
      ${getWrapperNameByType(field.type)}.ArrayType.freeArray(refObject.${field.name});
      if (!${getWrapperNameByType(field.type)}.ArrayType.useTypedArray) {
        deallocator.freeStructMember(refObject.${field.name}, ${getWrapperNameByType(field.type)}.refObjectArrayType, 'data');
      }
    }`;
        } else if (
          field.type.isArray &&
          !field.type.isPrimitiveType &&
          field.type.isFixedSizeArray
        ) {
          return `    for (let i = 0; i < ${field.type.arraySize}; i++) {
      ${getWrapperNameByType(field.type)}.freeStruct(refObject.${field.name}[i]);
    }`;
        } else if (
          field.type.isArray &&
          field.type.type === 'string' &&
          field.type.isFixedSizeArray
        ) {
          return `    for (let i = 0; i < ${field.type.arraySize}; i++) {
      ${getWrapperNameByType(field.type)}.freeStruct(refObject.${field.name}[i]);
    }`;
        } else if (
          !field.type.isPrimitiveType ||
          (field.type.type === 'string' && spec.msgName !== 'String')
        ) {
          return `    ${getWrapperNameByType(field.type)}.freeStruct(refObject.${field.name});`;
        } else if (spec.msgName === 'String') {
          return `    deallocator.freeStructMember(refObject, ${getWrapperNameByType(field.type)}.refObjectType, '${field.name}');`;
        }
        return '';
      })
      .filter((line) => line)
      .join('\n');
  }

  // Helper function to generate getters and setters
  function generateGettersSetters() {
    return spec.fields
      .map(
        (field) => `  get ${field.name}() {
${generateGetter(field)}
  }

  set ${field.name}(value) {
${generateSetter(field)}
  }`
      )
      .join('\n\n');
  }

  function generateGetter(field) {
    if (field.type.isArray && field.type.type === 'Constants') {
      return `    return [];`;
    } else if (field.type.isArray && isTypedArrayType(field.type)) {
      return `    return this._wrapperFields['${field.name}'].data;`;
    } else if (field.type.isArray && field.type.isPrimitiveType) {
      return `    return this._${field.name}Array;`;
    } else if (field.type.isArray && !field.type.isPrimitiveType) {
      return `    return this._wrapperFields.${field.name};`;
    } else if (!field.type.isPrimitiveType && !field.type.isArray) {
      return `    return this._wrapperFields.${field.name};`;
    } else if (
      !field.type.isArray &&
      field.type.type === 'string' &&
      spec.msgName !== 'String'
    ) {
      return `    return this._wrapperFields.${field.name}.data;`;
    } else if (isBigInt(field.type)) {
      return `    return BigInt(this._refObject.${field.name});`;
    } else {
      return `    return this._refObject.${field.name};`;
    }
  }

  function generateSetter(field) {
    let result = '';

    if (field.type.isArray && field.type.isFixedSizeArray) {
      result += `    if (value.length !== ${field.type.arraySize}) {
      throw new RangeError('The length of the array must be ${field.type.arraySize}.');
    }

`;
    }
    if (field.type.isArray && field.type.isUpperBound) {
      result += `    if (value.length > ${field.type.arraySize}) {
      throw new RangeError('The length of array ${field.name} must be <= ${field.type.arraySize}.');
    }
`;
    }

    if (field.type.isArray && isTypedArrayType(field.type)) {
      result += `    this._wrapperFields['${field.name}'].fill(value);`;
    } else if (field.type.isArray && field.type.isPrimitiveType) {
      result += `    this._${field.name}Array = value;`;
    } else if (field.type.isArray && !field.type.isPrimitiveType) {
      result += `    this._wrapperFields.${field.name}.fill(value);`;
    } else if (!field.type.isPrimitiveType && !field.type.isArray) {
      result += `    if (value instanceof ${getWrapperNameByType(field.type)}) {
      this._wrapperFields.${field.name}.copy(value);
    } else {
      this._wrapperFields.${field.name}.copy(new ${getWrapperNameByType(field.type)}(value));
    }`;
    } else if (
      !field.type.isArray &&
      field.type.type === 'string' &&
      spec.msgName !== 'String'
    ) {
      result += `    this._wrapperFields.${field.name}.data = value;`;
    } else if (isBigInt(field.type)) {
      result += `    if (typeof value !== "bigint") {
      throw new TypeError('${field.name} must be type of bigint');
    }
    this._refObject.${field.name} = value.toString();`;
    } else {
      if (spec.msgName === 'String') {
        result += `    this._refObject.size = Buffer.byteLength(value);
    this._refObject.capacity = Buffer.byteLength(value) + 1;
`;
      }
      result += `    this._refObject.${field.name} = value;`;
    }

    return result;
  }

  // Helper function to generate array wrapper class
  function generateArrayWrapperClass() {
    return `class ${arrayWrapper} {
  constructor(size = 0) {
    this._resize(size);
  }

  toRawROS() {
    return this._refObject.ref();
  }

  fill(values) {
${
  willUseTypedArray
    ? `    if (Array.isArray(values)) {
      this._wrappers = new ${currentTypedArray}(values);
    } else {
      this._wrappers = values;
    }`
    : isPrimitivePackage(spec.baseType)
      ? `    const length = values.length;
    this._resize(length);
    for (let i = 0; i < length; ++i) {
      let wrapper = new ${objectWrapper}();
      wrapper.data = values[i];
      this._wrappers[i] = wrapper;
    }`
      : `    const length = values.length;
    this._resize(length);
    values.forEach((value, index) => {
      if (value instanceof ${objectWrapper}) {
        this._wrappers[index].copy(value);
      } else {
        this._wrappers[index] = new ${objectWrapper}(value);
      }
    });`
}
  }

  freeze(own) {
${
  !willUseTypedArray
    ? `    this._wrappers.forEach((wrapper, index) => {
      wrapper.freeze(own);
      this._refArray[index] = wrapper.refObject;
    });
`
    : ''
}
    this._refObject.size = this._wrappers.length;
    this._refObject.capacity = this._wrappers.length;

    if (this._refObject.capacity === 0) {
      this._refObject.data = null
    } else {
${
  willUseTypedArray
    ? `      const buffer = Buffer.from(new Uint8Array(this._wrappers.buffer));
      this._refObject.data = buffer;`
    : `      this._refObject.data = this._refArray.buffer;`
}
    }
  }

  get refObject() {
    return this._refObject;
  }

  get data() {
    return this._wrappers;
  }

  get size() {
    return this._wrappers.length;
  }

  set size(value) {
    if (typeof value != 'number') {
      throw new TypeError('Invalid argument: should provide a number to ${arrayWrapper}.size setter');
    }
    return this._resize(value);
  }

  get capacity() {
    return this._wrappers.length;
  }

  set capacity(value) {
    if (typeof value != 'number') {
      throw new TypeError('Invalid argument: should provide a number to ${arrayWrapper}.capacity setter');
    }
    return this._resize(value);
  }

  _resize(size) {
    if (size < 0) {
      throw new RangeError('Invalid argument: should provide a positive number');
    }
${
  willUseTypedArray
    ? `    this._refArray = undefined;`
    : `    this._refArray = new ${refArrayType}(size);`
}

    this._refObject = new ${refObjectArrayType}();
    this._refObject.size = size;
    this._refObject.capacity = size;

${
  willUseTypedArray
    ? `    this._wrappers = new ${currentTypedArray}(size);`
    : `    this._wrappers = new Array();
    for (let i = 0; i < size; i++) {
      this._wrappers.push(new ${objectWrapper}());
    }`
}
  }

  copyRefObject(refObject) {
    this._refObject = refObject;

${
  willUseTypedArray
    ? `    const byteLen = refObject.size * ref.types.${currentTypedArrayElementType}.size;
    const arrayBuffer = refObject.data.length !== 0 ?
        rclnodejs.createArrayBufferFromAddress(refObject.data.hexAddress(), byteLen) :
        Buffer.alloc(0);
    this._wrappers = new ${currentTypedArray}(arrayBuffer);`
    : `    let refObjectArray = this._refObject.data;
    refObjectArray.length = this._refObject.size;
    this._resize(this._refObject.size);
    for (let index = 0; index < this._refObject.size; index++) {
      this._wrappers[index].copyRefObject(refObjectArray[index]);
    }`
}
  }

  copy(other) {
    if (! (other instanceof ${arrayWrapper})) {
      throw new TypeError('Invalid argument: should provide "${arrayWrapper}".');
    }

    this._resize(other.size);
${
  willUseTypedArray
    ? `    this._wrappers = other._wrappers.slice();`
    : `    other._wrappers.forEach((wrapper, index) => {
      this._wrappers[index].copy(wrapper);
    });`
}
  }

  static freeArray(refObject) {
${
  !willUseTypedArray
    ? `    let refObjectArray = refObject.data;
    refObjectArray.length = refObject.size;
    for (let index = 0; index < refObject.size; index++) {
      ${objectWrapper}.freeStruct(refObjectArray[index]);
    }`
    : ''
}
  }

  static get elementType() {
    return ${objectWrapper};
  }

  static get isROSArray() {
    return true;
  }

  static get useTypedArray() {
    return ${willUseTypedArray};
  }

  get classType() {
    return ${arrayWrapper};
  }

  toPlainObject(enableTypedArray) {
    return translator.toPlainObject(this, enableTypedArray);
  }
}`;
  }

  // Helper function to generate constants
  function generateConstants() {
    if (spec.constants && spec.constants.length > 0) {
      return spec.constants
        .map((c) => {
          const value = c.type === 'string' ? `"${c.value}"` : c.value;
          return `Object.defineProperty(${objectWrapper}, "${c.name}", {value: ${value}, writable: false, enumerable: true, configurable: true});`;
        })
        .join('\n');
    }
    return '';
  }
}

module.exports = generateMessage;
