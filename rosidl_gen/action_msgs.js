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

/* eslint-disable camelcase */
const GOAL_ID_FIELD = {
  name: 'goal_id',
  type: {
    isArray: false,
    arraySize: null,
    isUpperBound: false,
    isDynamicArray: false,
    isFixedSizeArray: null,
    pkgName: 'unique_identifier_msgs',
    type: 'UUID',
    stringUpperBound: null,
    isPrimitiveType: false,
  },
  default_value: null,
};

function createSendGoalRequestSpec(pkgName, interfaceName) {
  return {
    constants: [],
    fields: [
      GOAL_ID_FIELD,
      {
        name: 'goal',
        type: {
          isArray: false,
          arraySize: null,
          isUpperBound: false,
          isDynamicArray: false,
          isFixedSizeArray: null,
          pkgName: pkgName,
          type: `${interfaceName}_Goal`,
          stringUpperBound: null,
          isPrimitiveType: false,
        },
        default_value: null,
      },
    ],
    baseType: {
      pkgName: pkgName,
      type: `${interfaceName}_SendGoal_Request`,
      stringUpperBound: null,
      isPrimitiveType: false,
    },
    msgName: `${interfaceName}_SendGoal_Request`,
  };
}

function createSendGoalResponseSpec(pkgName, interfaceName) {
  return {
    constants: [],
    fields: [
      {
        name: 'accepted',
        type: {
          isArray: false,
          arraySize: null,
          isUpperBound: false,
          isDynamicArray: false,
          isFixedSizeArray: null,
          pkgName: null,
          type: 'bool',
          stringUpperBound: null,
          isPrimitiveType: true,
        },
        default_value: null,
      },
      {
        name: 'stamp',
        type: {
          isArray: false,
          arraySize: null,
          isUpperBound: false,
          isDynamicArray: false,
          isFixedSizeArray: null,
          pkgName: 'builtin_interfaces',
          type: 'Time',
          stringUpperBound: null,
          isPrimitiveType: false,
        },
        default_value: null,
      },
    ],
    baseType: {
      pkgName: pkgName,
      type: `${interfaceName}_SendGoal_Response`,
      stringUpperBound: null,
      isPrimitiveType: false,
    },
    msgName: `${interfaceName}_SendGoal_Response`,
  };
}

function createGetResultRequestSpec(pkgName, interfaceName) {
  return {
    constants: [],
    fields: [GOAL_ID_FIELD],
    baseType: {
      pkgName: pkgName,
      type: `${interfaceName}_GetResult_Request`,
      stringUpperBound: null,
      isPrimitiveType: false,
    },
    msgName: `${interfaceName}_GetResult_Request`,
  };
}

function createGetResultResponseSpec(pkgName, interfaceName) {
  return {
    constants: [],
    fields: [
      {
        name: 'status',
        type: {
          isArray: false,
          arraySize: null,
          isUpperBound: false,
          isDynamicArray: false,
          isFixedSizeArray: null,
          pkgName: null,
          type: 'int8',
          stringUpperBound: null,
          isPrimitiveType: true,
        },
        default_value: null,
      },
      {
        name: 'result',
        type: {
          isArray: false,
          arraySize: null,
          isUpperBound: false,
          isDynamicArray: false,
          isFixedSizeArray: null,
          pkgName: pkgName,
          type: `${interfaceName}_Result`,
          stringUpperBound: null,
          isPrimitiveType: false,
        },
        default_value: null,
      },
    ],
    baseType: {
      pkgName: pkgName,
      type: `${interfaceName}_GetResult_Response`,
      stringUpperBound: null,
      isPrimitiveType: false,
    },
    msgName: `${interfaceName}_GetResult_Response`,
  };
}

function createFeedbackMessageSpec(pkgName, interfaceName) {
  return {
    constants: [],
    fields: [
      GOAL_ID_FIELD,
      {
        name: 'feedback',
        type: {
          isArray: false,
          arraySize: null,
          isUpperBound: false,
          isDynamicArray: false,
          isFixedSizeArray: null,
          pkgName: pkgName,
          type: `${interfaceName}_Feedback`,
          stringUpperBound: null,
          isPrimitiveType: false,
        },
        default_value: null,
      },
    ],
    baseType: {
      pkgName: pkgName,
      type: `${interfaceName}_FeedbackMessage`,
      stringUpperBound: null,
      isPrimitiveType: false,
    },
    msgName: `${interfaceName}_FeedbackMessage`,
  };
}

module.exports = {
  createSendGoalRequestSpec,
  createSendGoalResponseSpec,
  createGetResultRequestSpec,
  createGetResultResponseSpec,
  createFeedbackMessageSpec,
};
