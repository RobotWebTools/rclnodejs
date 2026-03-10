# Copyright (c) 2025, The Robot Web Tools Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
IDL file parser that uses rosidl_parser to directly parse .idl files and
produce the same JSON spec format as parser.py does for .msg files.

This eliminates the need for the intermediate .idl -> .msg conversion step.
"""

import sys
import json

from rosidl_parser.parser import parse_idl_string
from rosidl_parser.definition import (
    AbstractNestedType,
    Action,
    Array,
    BasicType,
    BoundedSequence,
    BoundedString,
    BoundedWString,
    Message,
    NamedType,
    NamespacedType,
    Service,
    UnboundedSequence,
    UnboundedString,
    UnboundedWString,
)

# Mapping from IDL basic type names to ROS2 message type names
IDL_TYPE_TO_ROS2 = {
    'boolean': 'bool',
    'octet': 'byte',
    'char': 'char',
    'wchar': 'wchar',
    'int8': 'int8',
    'uint8': 'uint8',
    'int16': 'int16',
    'uint16': 'uint16',
    'int32': 'int32',
    'uint32': 'uint32',
    'int64': 'int64',
    'uint64': 'uint64',
    'float': 'float32',
    'double': 'float64',
    'long double': 'float64',
    'short': 'int16',
    'long': 'int32',
    'long long': 'int64',
    'unsigned short': 'uint16',
    'unsigned long': 'uint32',
    'unsigned long long': 'uint64',
}


def _map_basic_type(typename):
    """Map an IDL basic type name to its ROS2 equivalent."""
    return IDL_TYPE_TO_ROS2.get(typename, typename)


def _get_nestable_type_info(type_obj):
    """
    Extract type info from a nestable type (the inner type for arrays/sequences,
    or the type itself for non-nested types).

    Returns a dict with: pkgName, type, stringUpperBound, isPrimitiveType
    """
    if isinstance(type_obj, BasicType):
        ros2_type = _map_basic_type(type_obj.typename)
        return {
            'pkgName': None,
            'type': ros2_type,
            'stringUpperBound': None,
            'isPrimitiveType': True,
        }
    elif isinstance(type_obj, (UnboundedString, BoundedString)):
        string_upper_bound = None
        if isinstance(type_obj, BoundedString):
            string_upper_bound = type_obj.maximum_size
        return {
            'pkgName': None,
            'type': 'string',
            'stringUpperBound': string_upper_bound,
            'isPrimitiveType': True,
        }
    elif isinstance(type_obj, (UnboundedWString, BoundedWString)):
        string_upper_bound = None
        if isinstance(type_obj, BoundedWString):
            string_upper_bound = type_obj.maximum_size
        return {
            'pkgName': None,
            'type': 'wstring',
            'stringUpperBound': string_upper_bound,
            'isPrimitiveType': True,
        }
    elif isinstance(type_obj, NamespacedType):
        # Convert Token objects to plain strings if needed
        namespaces = [str(ns) for ns in type_obj.namespaces]
        pkg_name = namespaces[0] if namespaces else None
        return {
            'pkgName': pkg_name,
            'type': str(type_obj.name),
            'stringUpperBound': None,
            'isPrimitiveType': False,
        }
    elif isinstance(type_obj, NamedType):
        # NamedType is typically resolved by typedefs, but handle it just in case
        return {
            'pkgName': None,
            'type': str(type_obj.name),
            'stringUpperBound': None,
            'isPrimitiveType': False,
        }
    else:
        raise ValueError(f'Unexpected nestable type: {type(type_obj).__name__}')


def _get_type_json(member_type):
    """
    Convert a rosidl_parser type object to the JSON format expected by
    the rclnodejs templates.

    Returns a dict matching the spec.fields[].type format:
    {isArray, arraySize, isUpperBound, isDynamicArray, isFixedSizeArray,
     pkgName, type, stringUpperBound, isPrimitiveType}
    """
    if isinstance(member_type, Array):
        # Fixed-size array: e.g., float64[9]
        inner_info = _get_nestable_type_info(member_type.value_type)
        return {
            'isArray': True,
            'arraySize': member_type.size,
            'isUpperBound': False,
            'isDynamicArray': False,
            'isFixedSizeArray': True,
            **inner_info,
        }
    elif isinstance(member_type, BoundedSequence):
        # Bounded dynamic array: e.g., sequence<float64, 3>
        inner_info = _get_nestable_type_info(member_type.value_type)
        return {
            'isArray': True,
            'arraySize': member_type.maximum_size,
            'isUpperBound': True,
            'isDynamicArray': True,
            'isFixedSizeArray': False,
            **inner_info,
        }
    elif isinstance(member_type, UnboundedSequence):
        # Unbounded dynamic array: e.g., sequence<float64>
        inner_info = _get_nestable_type_info(member_type.value_type)
        return {
            'isArray': True,
            'arraySize': None,
            'isUpperBound': False,
            'isDynamicArray': True,
            'isFixedSizeArray': False,
            **inner_info,
        }
    else:
        # Non-array/sequence type
        inner_info = _get_nestable_type_info(member_type)
        return {
            'isArray': False,
            'arraySize': None,
            'isUpperBound': False,
            'isDynamicArray': False,
            'isFixedSizeArray': False,
            **inner_info,
        }


def _get_default_value(member):
    """Extract default value from member annotations.

    rosidl_parser returns array/sequence defaults as tuple-like strings,
    e.g., "(0, 127, -128)" or "('', 'max value', 'min value')".
    The .msg parser returns these as Python lists [0, 127, -128].
    We need to convert the tuple-strings to proper lists.
    """
    try:
        default_annotation = member.get_annotation_value('default')
        if isinstance(default_annotation, dict) and 'value' in default_annotation:
            value = default_annotation['value']
        else:
            value = default_annotation

        # If the value is a string that looks like a tuple "(a, b, c)",
        # parse it into a Python list
        if isinstance(value, str) and value.startswith('(') and value.endswith(')'):
            value = _parse_tuple_string(value, member)

        return value
    except ValueError:
        return None


def _parse_tuple_string(s, member):
    """
    Parse a tuple-like string from rosidl_parser into a proper Python list.

    Handles formats like:
      "(0, 127, -128)" -> [0, 127, -128]
      "(1.125, 0.0, -1.125)" -> [1.125, 0.0, -1.125]
      "(False, True, False)" -> [False, True, False]
      "('', 'max value', 'min value')" -> ['', 'max value', 'min value']
    """
    inner = s[1:-1]  # Strip outer parentheses

    # Determine the element type from the member's type
    member_type = member.type
    if isinstance(member_type, AbstractNestedType):
        element_type = member_type.value_type
    else:
        element_type = member_type

    # Parse string arrays specially (may contain commas in values)
    if isinstance(element_type, (UnboundedString, BoundedString,
                                  UnboundedWString, BoundedWString)):
        return _parse_string_tuple(inner)

    # For boolean arrays
    if isinstance(element_type, BasicType) and element_type.typename == 'boolean':
        parts = [p.strip() for p in inner.split(',')]
        return [p == 'True' for p in parts]

    # For numeric arrays  
    parts = [p.strip() for p in inner.split(',')]
    result = []
    for part in parts:
        if not part:
            continue
        # Try int first, then float
        try:
            result.append(int(part))
        except ValueError:
            try:
                result.append(float(part))
            except ValueError:
                result.append(part)
    return result


def _parse_string_tuple(inner):
    """Parse the inside of a tuple containing quoted strings.

    e.g., "'', 'max value', 'min value'" -> ['', 'max value', 'min value']
    """
    result = []
    i = 0
    while i < len(inner):
        if inner[i] == "'":
            # Find matching closing quote
            j = i + 1
            while j < len(inner):
                if inner[j] == "'":
                    result.append(inner[i + 1:j])
                    i = j + 1
                    break
                j += 1
            else:
                break
        elif inner[i] == '"':
            j = i + 1
            while j < len(inner):
                if inner[j] == '"':
                    result.append(inner[i + 1:j])
                    i = j + 1
                    break
                j += 1
            else:
                break
        else:
            i += 1
    return result


def _get_constant_type_str(constant):
    """Get the ROS2 type string for a constant."""
    if isinstance(constant.type, BasicType):
        return _map_basic_type(constant.type.typename)
    elif isinstance(constant.type, (UnboundedString, BoundedString)):
        return 'string'
    elif isinstance(constant.type, (UnboundedWString, BoundedWString)):
        return 'wstring'
    return str(constant.type)


def get_json_from_message(msg, pkg_name=None):
    """
    Convert a rosidl_parser Message object to JSON spec matching
    what parser.py produces for .msg files.
    """
    namespaced_type = msg.structure.namespaced_type
    namespaces = [str(ns) for ns in namespaced_type.namespaces]
    msg_name = str(namespaced_type.name)

    if pkg_name is None:
        pkg_name = namespaces[0] if namespaces else ''

    # Build fields
    fields = []
    for member in msg.structure.members:
        field = {
            'name': member.name,
            'type': _get_type_json(member.type),
            'default_value': _get_default_value(member),
        }
        fields.append(field)

    # Build constants
    constants = []
    for constant in msg.constants:
        constants.append({
            'type': _get_constant_type_str(constant),
            'name': constant.name,
            'value': constant.value,
        })

    return {
        'constants': constants,
        'fields': fields,
        'baseType': {
            'pkgName': pkg_name,
            'type': msg_name,
            'stringUpperBound': None,
            'isPrimitiveType': False,
        },
        'msgName': msg_name,
    }


def get_json_from_service(srv):
    """
    Convert a rosidl_parser Service object to JSON spec matching
    what parser.py produces for .srv files.
    """
    namespaces = [str(ns) for ns in srv.namespaced_type.namespaces]
    pkg_name = namespaces[0] if namespaces else ''
    srv_name = str(srv.namespaced_type.name)

    return {
        'pkgName': pkg_name,
        'srvName': srv_name,
        'request': get_json_from_message(srv.request_message, pkg_name),
        'response': get_json_from_message(srv.response_message, pkg_name),
    }


def get_json_from_action(action):
    """
    Convert a rosidl_parser Action object to JSON spec matching
    what parser.py produces for .action files.
    """
    namespaces = [str(ns) for ns in action.namespaced_type.namespaces]
    pkg_name = namespaces[0] if namespaces else ''
    action_name = str(action.namespaced_type.name)

    return {
        'pkgName': pkg_name,
        'actionName': action_name,
        'goal': get_json_from_message(action.goal, pkg_name),
        'result': get_json_from_message(action.result, pkg_name),
        'feedback': get_json_from_message(action.feedback, pkg_name),
    }


def parse_idl_file(idl_file_path):
    """
    Parse an .idl file and return a JSON object in the same format
    as parser.py produces.

    The return value depends on the interface type:
    - Message: same format as parse_message_file
    - Service: same format as parse_service_file
    - Action: same format as parse_action_file

    Returns a dict with 'type' indicating the interface kind and 'spec'
    containing the JSON spec.
    """
    with open(idl_file_path, 'r') as f:
        content_str = f.read()

    content = parse_idl_string(content_str)

    # Find the main element (Message, Service, or Action)
    messages = content.get_elements_of_type(Message)
    services = content.get_elements_of_type(Service)
    actions = content.get_elements_of_type(Action)

    if actions:
        action = actions[0]
        return {
            'type': 'action',
            'spec': get_json_from_action(action),
        }
    elif services:
        service = services[0]
        return {
            'type': 'service',
            'spec': get_json_from_service(service),
        }
    elif messages:
        message = messages[0]
        return {
            'type': 'message',
            'spec': get_json_from_message(message),
        }
    else:
        raise ValueError(f'No Message, Service, or Action found in {idl_file_path}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: idl_parser.py <idl_file_path>', file=sys.stderr)
        sys.exit(1)

    try:
        result = parse_idl_file(sys.argv[1])
        print(json.dumps(result))
        sys.exit(0)
    except Exception as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)
