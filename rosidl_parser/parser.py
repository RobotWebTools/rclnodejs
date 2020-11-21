# Copyright (c) 2017 Intel Corporation. All rights reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import json

import rosidl_parser
from rosidl_adapter import parser

def get_json_object_from_base_type_object(base_type_obj):
     return {'pkgName': base_type_obj.pkg_name, 'type': base_type_obj.type,
         'stringUpperBound': base_type_obj.string_upper_bound, 'isPrimitiveType': base_type_obj.is_primitive_type()}

def get_json_object_from_type_object(type_obj):
    json_obj = {'isArray': type_obj.is_array,
        'arraySize': type_obj.array_size, 'isUpperBound': type_obj.is_upper_bound,
        'isDynamicArray': type_obj.is_dynamic_array(), 'isFixedSizeArray': type_obj.is_fixed_size_array()}
    json_obj.update(get_json_object_from_base_type_object(type_obj))
    return json_obj

def get_json_object_from_msg_spec_object(msg_spec_object):
    fields = []
    for field in msg_spec_object.fields:
        dict_field = {'name': field.name}
        dict_field['type'] = get_json_object_from_type_object(field.type)
        dict_field['default_value'] = field.default_value
        fields.append(dict_field)

    constants = []
    for constant in msg_spec_object.constants:
        constants.append({'type': constant.type, 'name': constant.name, 'value': constant.value})

    msg_name = {'msgName': msg_spec_object.msg_name}
    json_obj = {'constants': constants, 'fields': fields, 'baseType': get_json_object_from_base_type_object(msg_spec_object.base_type),
        'msgName': msg_spec_object.msg_name}

    return json_obj

if __name__ == '__main__':
    if len(sys.argv) < 3:
        print('Wrong number of argments')
        sys.exit(1)
    try:
        parser_method = getattr(parser, sys.argv[1])
        spec = parser_method(sys.argv[2], sys.argv[3])

        if sys.argv[1] == 'parse_message_file':
            json_obj = get_json_object_from_msg_spec_object(spec)
        elif sys.argv[1] == 'parse_service_file':
            json_obj = {'pkgName': spec.pkg_name, 'srvName': spec.srv_name,
                'request': get_json_object_from_msg_spec_object(spec.request),
                'response': get_json_object_from_msg_spec_object(spec.response)}
        elif sys.argv[1] == 'parse_action_file':
            json_obj = {'pkgName': spec.pkg_name, 'actionName': spec.action_name,
                'goal': get_json_object_from_msg_spec_object(spec.goal),
                'result': get_json_object_from_msg_spec_object(spec.result),
                'feedback': get_json_object_from_msg_spec_object(spec.feedback)}
        else:
            assert False, "unknown method '%s'" % sys.argv[1]

        print(json.dumps(json_obj))
        sys.exit(0)

    except Exception as e:
        print(str(e), file=sys.stderr)
        sys.exit(1)
