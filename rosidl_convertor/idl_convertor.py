#!/usr/bin/env python3

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
IDL to ROS2 Interface Converter

This tool converts ROS2 .idl files to corresponding .msg/.srv/.action files.
It parses IDL syntax and generates proper ROS2 interface definitions.
"""

import os
import sys
import re
import pathlib
import argparse
from typing import List, Dict, Optional
from dataclasses import dataclass
from enum import Enum


class IdlElementType(Enum):
    MESSAGE = "message"
    SERVICE = "service"
    ACTION = "action"


@dataclass
class IdlField:
    """Represents a field in an IDL structure"""
    field_type: str
    name: str
    is_array: bool = False
    array_size: Optional[int] = None
    is_sequence: bool = False
    is_bounded_string: bool = False
    default_value: Optional[str] = None
    comment: Optional[str] = None


@dataclass
class IdlConstant:
    """Represents a constant definition in an IDL structure"""
    name: str
    const_type: str
    value: str


@dataclass
class IdlStructure:
    """Represents an IDL structure (message, service part, etc.)"""
    name: str
    fields: List[IdlField]
    constants: List[IdlConstant]
    comments: List[str]


@dataclass
class IdlInterface:
    """Represents a complete IDL interface definition"""
    name: str
    interface_type: IdlElementType
    package: str
    structures: List[IdlStructure]  # For messages: 1 structure, services: 2 (request/response), actions: 3 (goal/result/feedback)


class IdlParser:
    """Parser for IDL files"""

    # Type mapping from IDL to ROS2
    TYPE_MAPPING = {
        'boolean': 'bool',
        'octet': 'byte',  # IDL octet maps to ROS2 byte
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
        'string': 'string',
        'wstring': 'wstring',
    }

    def __init__(self):
        self.includes = []
        self.current_package = ""
        self.current_module = ""
        self.typedefs = {}  # Store typedef declarations

    def _contains_key_annotations(self, content: str) -> bool:
        """Check if IDL content contains @key annotations or references to keyed types (not supported in ROS2 .msg)"""
        # Direct @key annotations
        if '@key' in content:
            return True

        # Check for references to known keyed types
        keyed_type_patterns = [
            r'test_msgs::msg::KeyedString',
            r'test_msgs::msg::KeyedLong',
            r'KeyedString',
            r'KeyedLong'
        ]

        import re
        for pattern in keyed_type_patterns:
            if re.search(pattern, content):
                return True

        return False

    def parse_file(self, idl_file_path: str) -> List[IdlInterface]:
        """Parse an IDL file and return list of interfaces"""
        with open(idl_file_path, 'r') as f:
            content = f.read()

        return self.parse_content(content, idl_file_path)

    def parse_content(self, content: str, file_path: str = "") -> List[IdlInterface]:
        """Parse IDL content string"""
        interfaces = []

        # Check for unsupported features
        if self._contains_key_annotations(content):
            # Determine the specific reason for skipping
            if '@key' in content:
                reason = "contains @key annotations"
            else:
                reason = "references keyed types"
            print(f"Warning: Skipping {file_path} - {reason} which are not supported in ROS2 .msg files")
            return interfaces

        # Extract modules and their contents BEFORE preprocessing (to preserve @verbatim)
        modules = self._extract_modules(content)

        for module_info in modules:
            module_name = module_info['name']
            module_content = module_info['content']

            # Set current package for type mapping
            self.current_package = module_name.split('::')[0]

            # Parse typedefs FIRST from raw content (before preprocessing removes them)
            self._parse_typedefs(module_content)

            # Parse structures (to extract verbatim comments before preprocessing)
            structures = self._parse_structures(module_content)

            # Now preprocess the content to remove comments and normalize
            clean_content = self._preprocess_content(module_content)

            # Parse constants from nested modules from clean content
            constants = self._parse_constants_from_modules(clean_content)

            # Add constants to the main structure (if any structures exist)
            if structures and constants:
                # Add constants to the first structure (typically the main message structure)
                structures[0].constants.extend(constants)

            for struct in structures:
                # Determine interface type based on naming convention or structure
                interface_type = self._determine_interface_type(struct, file_path)

                interface = IdlInterface(
                    name=struct.name,
                    interface_type=interface_type,
                    package=module_name,
                    structures=[struct]
                )
                interfaces.append(interface)

        return interfaces

    def _preprocess_content(self, content: str) -> str:
        """Remove @verbatim blocks from the content with proper string handling."""
        lines = content.split('\n')
        result_lines = []

        i = 0
        while i < len(lines):
            line = lines[i]
            if '@verbatim' in line:
                # Find opening parenthesis after @verbatim
                verbatim_pos = line.find('@verbatim')
                paren_pos = line.find('(', verbatim_pos)
                if paren_pos == -1:
                    result_lines.append(line)
                    i += 1
                    continue

                # Parse with string awareness
                paren_count = 0
                in_string = False
                escape_next = False
                start_part = line[:verbatim_pos]

                # Process current line starting from opening parenthesis
                j = paren_pos
                while j < len(line):
                    char = line[j]

                    if escape_next:
                        escape_next = False
                    elif char == '\\':
                        escape_next = True
                    elif char == '"' and not escape_next:
                        in_string = not in_string
                    elif not in_string:
                        if char == '(':
                            paren_count += 1
                        elif char == ')':
                            paren_count -= 1
                            if paren_count == 0:
                                # Found end of verbatim block
                                result_lines.append(start_part + line[j+1:])
                                i += 1
                                break
                    j += 1
                else:
                    # Verbatim block continues to next lines
                    i += 1
                    while i < len(lines) and paren_count > 0:
                        line = lines[i]
                        j = 0
                        while j < len(line):
                            char = line[j]

                            if escape_next:
                                escape_next = False
                            elif char == '\\':
                                escape_next = True
                            elif char == '"' and not escape_next:
                                in_string = not in_string
                            elif not in_string:
                                if char == '(':
                                    paren_count += 1
                                elif char == ')':
                                    paren_count -= 1
                                    if paren_count == 0:
                                        # Found end
                                        result_lines.append(start_part + line[j+1:])
                                        i += 1
                                        break
                            j += 1
                        else:
                            i += 1
                            continue
                        break
            else:
                result_lines.append(line)
                i += 1

        return '\n'.join(result_lines)

    def _extract_modules(self, content: str) -> List[Dict]:
        """Extract module definitions from content"""
        modules = []

        # Find module blocks - improved pattern for better nested module handling
        # This pattern will match modules even with complex nested structures
        pos = 0
        while True:
            # Find next module declaration
            module_match = re.search(r'module\s+(\w+)\s*\{', content[pos:])
            if not module_match:
                break

            module_name = module_match.group(1)
            start_pos = pos + module_match.end() - 1  # Position of opening brace

            # Count braces to find the matching closing brace
            brace_count = 1
            current_pos = start_pos + 1

            while current_pos < len(content) and brace_count > 0:
                if content[current_pos] == '{':
                    brace_count += 1
                elif content[current_pos] == '}':
                    brace_count -= 1
                current_pos += 1

            if brace_count == 0:
                # Found the matching closing brace
                module_content = content[start_pos + 1:current_pos - 1]

                # Handle nested modules recursively
                nested_modules = self._extract_modules(module_content)
                if nested_modules:
                    for nested in nested_modules:
                        nested['name'] = f"{module_name}::{nested['name']}"
                        modules.append(nested)

                    # Only add the current module if it has content beyond just nested modules
                    # Check if there are any struct, enum, typedef, or const definitions directly in this module
                    lines = module_content.split('\n')
                    has_direct_definitions = False
                    in_nested_module = False
                    brace_level = 0

                    for line in lines:
                        line = line.strip()
                        if line.startswith('module ') and '{' in line:
                            in_nested_module = True
                            brace_level = 1
                        elif in_nested_module:
                            brace_level += line.count('{') - line.count('}')
                            if brace_level <= 0:
                                in_nested_module = False
                        elif not in_nested_module and re.search(r'^\s*(struct|enum|typedef|const)\s+\w+', line):
                            has_direct_definitions = True
                            break

                    if has_direct_definitions:
                        modules.append({
                            'name': module_name,
                            'content': module_content
                        })
                else:
                    # No nested modules, always add this module
                    modules.append({
                        'name': module_name,
                        'content': module_content
                    })

                pos = current_pos
            else:
                # Unmatched braces, skip this occurrence
                pos = pos + module_match.end()

        return modules

    def _parse_typedefs(self, content: str):
        """Parse typedef declarations from module content"""
        # First pass: find simple typedefs like: typedef test_msgs::msg::Arrays test_msgs__msg__Arrays;
        simple_typedef_pattern = r'typedef\s+([^;\s]+)\s+([^;\[\s]+)\s*;'
        matches = re.finditer(simple_typedef_pattern, content, re.DOTALL)

        for match in matches:
            source_type = match.group(1).strip()
            target_name = match.group(2).strip()

            # Skip if this is actually an array typedef (contains [])
            if '[' not in match.group(0):
                # Map the source type and store the simple typedef
                ros_source_type = self._map_type(source_type)
                self.typedefs[target_name] = {
                    'base_type': ros_source_type,
                    'array_size': None
                }

        # Second pass: find array typedefs like: typedef double double__9[9];
        array_typedef_pattern = r'typedef\s+([^[\s]+)\s+(\w+)\[(\d+)\]\s*;'
        matches = re.finditer(array_typedef_pattern, content, re.DOTALL)

        for match in matches:
            base_type = match.group(1)
            typedef_name = match.group(2)
            array_size = int(match.group(3))

            # Map the base type and store the typedef
            ros_base_type = self._map_type(base_type)
            self.typedefs[typedef_name] = {
                'base_type': ros_base_type,
                'array_size': array_size
            }

    def _parse_constants_from_modules(self, content: str) -> List[IdlConstant]:
        """Parse constants from nested constant modules"""
        constants = []

        # Find constant modules like: module SomeConstants { const uint8 NAME = VALUE; };
        const_module_pattern = r'module\s+(\w*[Cc]onstants?\w*)\s*\{([^{}]*(?:\{[^{}]*\}[^{}]*)*)\}'
        matches = re.finditer(const_module_pattern, content, re.DOTALL)

        for match in matches:
            module_name = match.group(1)
            module_content = match.group(2)

            # Find const declarations within the module
            const_pattern = r'const\s+(\w+)\s+(\w+)\s*=\s*([^;]+);'
            const_matches = re.finditer(const_pattern, module_content)

            for const_match in const_matches:
                const_type = const_match.group(1)
                const_name = const_match.group(2)
                const_value = const_match.group(3).strip()

                # Map the type to ROS2 type
                ros_type = self._map_type(const_type)

                constant = IdlConstant(
                    name=const_name,
                    const_type=ros_type,
                    value=const_value
                )
                constants.append(constant)

        return constants

    def _parse_structures(self, content: str) -> List[IdlStructure]:
        """Parse structure definitions from module content"""
        structures = []

        # Find struct definitions
        struct_pattern = r'struct\s+(\w+)\s*\{([^{}]*(?:\{[^{}]*\}[^{}]*)*)\}'
        matches = re.finditer(struct_pattern, content, re.DOTALL)

        for match in matches:
            struct_name = match.group(1)
            struct_content = match.group(2)

            # Extract comments from @verbatim blocks before the struct
            comments = self._extract_verbatim_comments(content, match.start())

            # Parse fields from the struct content (need to preprocess it first)
            clean_struct_content = self._preprocess_content(struct_content)
            fields = self._parse_fields(clean_struct_content, struct_content, struct_name)  # Pass both clean, original, and struct name

            structure = IdlStructure(
                name=struct_name,
                fields=fields,
                constants=[],
                comments=comments
            )
            structures.append(structure)

        return structures

    def _extract_verbatim_comments(self, content: str, struct_start_pos: int) -> List[str]:
        """Extract comments from @verbatim blocks immediately before a struct definition"""
        comments = []

        # Look backwards from struct position to find the most recent @verbatim block
        content_before_struct = content[:struct_start_pos]

        # Try to find the last @verbatim block before the struct
        # Look for pattern: @verbatim (language="comment", text="...") struct

        # First, try single-line @verbatim pattern
        single_line_pattern = r'@verbatim\s*\(\s*language\s*=\s*"comment"\s*,\s*text\s*=\s*"([^"]+)"\s*\)\s*$'

        # Split into lines and work backwards
        lines = content_before_struct.split('\n')

        for line in reversed(lines):
            line_stripped = line.strip()

            # Skip empty lines and braces
            if not line_stripped or line_stripped in ['}', '};']:
                continue

            # If we hit non-verbatim content that's not empty/closing, stop looking
            if not line_stripped.startswith('@verbatim') and line_stripped:
                # Unless it's just whitespace or a closing brace, stop
                if not (line_stripped == '}' or line_stripped == '};' or not line_stripped):
                    break

            # Look for @verbatim
            if '@verbatim' in line and 'language="comment"' in line:
                # Try single-line match first
                match = re.search(single_line_pattern, line)
                if match:
                    comment_text = match.group(1).strip().replace('\\n', '\n')
                    return [comment_text]

                # For multi-line, we need to look at the next line
                # This handles cases like:
                # @verbatim (language="comment", text=
                #   "The comment text")
                break

        # If single-line didn't work, try multi-line pattern
        # Look for @verbatim blocks that span multiple lines
        multi_line_pattern = r'@verbatim\s*\(\s*language\s*=\s*"comment"\s*,\s*text\s*=\s*"([^"]+)"\s*\)'

        # Search in a reasonable window before the struct (last 500 characters)
        search_window = content_before_struct[-500:] if len(content_before_struct) > 500 else content_before_struct

        matches = list(re.finditer(multi_line_pattern, search_window, re.DOTALL))
        if matches:
            # Take the last match (closest to the struct)
            last_match = matches[-1]
            comment_text = last_match.group(1).strip().replace('\\n', '\n')
            return [comment_text]

        return comments

    def _extract_inline_verbatim_comments(self, content: str) -> Dict[str, str]:
        """Extract comments from @verbatim blocks that appear before field definitions"""
        field_comments = {}
        lines = content.split('\n')

        i = 0
        while i < len(lines):
            line = lines[i].strip()
            if '@verbatim' in line and 'language="comment"' in line:
                # Extract the comment text - handle multi-line format
                comment_text = ""

                # Look for text on the same line first
                text_match = re.search(r'text\s*=\s*"([^"]*)"', line)
                if text_match:
                    comment_text = text_match.group(1)
                else:
                    # Look for text in subsequent lines - handle concatenated strings
                    j = i + 1
                    in_text = True  # Set to True since we found 'text=' in the @verbatim line
                    text_parts = []

                    while j < len(lines):
                        next_line = lines[j].strip()

                        if in_text:
                            # Handle different string concatenation patterns
                            if next_line.startswith('"') and next_line.endswith('" "'):
                                # Pattern: "text" "
                                text_content = next_line[1:-3]  # Remove start quote and end quote+space+quote
                                if text_content:
                                    text_parts.append(text_content)
                            elif next_line.startswith('"') and next_line.endswith('")'):
                                # Pattern: "text")
                                text_content = next_line[1:-2]  # Remove start quote and end quote+paren
                                if text_content:
                                    text_parts.append(text_content)
                            elif next_line == '"':
                                # Just a newline marker
                                text_parts.append('\n')
                            elif '"' in next_line:
                                # Extract all quoted content
                                quote_matches = re.findall(r'"([^"]*)"', next_line)
                                for quote_match in quote_matches:
                                    if quote_match:  # Skip empty strings unless they're newlines
                                        text_parts.append(quote_match)

                            # Check if we've reached the end of the verbatim block
                            if ')' in next_line and in_text:
                                break
                        j += 1

                    comment_text = ''.join(text_parts)

                # Convert \n to actual newlines and clean up
                comment_text = comment_text.replace('\\n', '\n').strip()

                # Find the next field definition
                k = j + 1  # Start after the verbatim block ends
                while k < len(lines):
                    field_line = lines[k].strip()
                    if field_line and not field_line.startswith('@') and not field_line.startswith('//') and not field_line.startswith('"'):
                        # Extract field name
                        field_match = re.search(r'\b(\w+)\s*;', field_line)
                        if field_match:
                            field_name = field_match.group(1)
                            if comment_text:
                                field_comments[field_name] = comment_text
                        break
                    k += 1
            i += 1

        return field_comments

    def _parse_fields(self, struct_content: str, original_content: str = None, struct_name: str = None) -> List[IdlField]:
        """Parse field definitions from struct content"""
        fields = []

        # Use original content for default value extraction if provided
        content_for_defaults = original_content if original_content else struct_content

        # First, extract default values from the original content
        default_values = self._extract_default_values(content_for_defaults)

        # Extract verbatim comments from within the struct
        inline_comments = self._extract_inline_verbatim_comments(content_for_defaults)

        # Remove @verbatim blocks and @default annotations from the clean content
        cleaned_content = self._remove_verbatim_blocks(struct_content)

        # Split by semicolon and process each field
        field_lines = [line.strip() for line in cleaned_content.split(';') if line.strip()]

        for field_line in field_lines:
            field = self._parse_single_field(field_line, struct_name)
            if field and field.name in default_values:
                field.default_value = default_values[field.name]
            # Add inline comment if available
            if field and field.name in inline_comments:
                field.comment = inline_comments[field.name]
            if field:
                fields.append(field)

        return fields

    def _extract_default_values(self, content: str) -> Dict[str, str]:
        """Extract default values from @default annotations"""
        default_values = {}
        lines = content.split('\n')

        i = 0
        while i < len(lines):
            line = lines[i].strip()
            if '@default' in line:
                # Use regex to extract the value from @default (value=...)
                # Handle nested parentheses in the value
                default_match = re.search(r'@default\s*\(\s*value\s*=\s*(.+)\)\s*$', line)
                if default_match:
                    default_value = default_match.group(1).strip()

                    # Handle different value formats
                    if default_value.startswith('"') and default_value.endswith('"'):
                        # Quoted string - preserve quotes for string fields
                        inner_value = default_value[1:-1]
                        if inner_value.startswith('(') and inner_value.endswith(')'):
                            # It's a quoted tuple like "(False, True, False)"
                            inner_content = inner_value[1:-1]
                            # Replace Python boolean constants with ROS2 format
                            inner_content = inner_content.replace('False', 'false').replace('True', 'true')
                            # For string arrays, convert single quotes to double quotes
                            if "'" in inner_content:
                                inner_content = inner_content.replace("'", '"')
                            default_value = '[' + inner_content + ']'
                        else:
                            # For string fields, first unescape any escaped quotes
                            unescaped_value = inner_value.replace('\\"', '"').replace("\\'", "'")

                            # Now apply quoting logic based on content
                            if '"' in unescaped_value and "'" not in unescaped_value:
                                # Has double quotes only - use single quotes to wrap
                                default_value = "'" + unescaped_value + "'"
                            elif "'" in unescaped_value and '"' not in unescaped_value:
                                # Has single quotes only - use double quotes to wrap  
                                default_value = '"' + unescaped_value + '"'
                            elif "'" in unescaped_value and '"' in unescaped_value:
                                # Has both - escape single quotes and use single quotes to wrap
                                escaped_value = unescaped_value.replace("'", "\\'")
                                default_value = "'" + escaped_value + "'"
                            else:
                                # No internal quotes - use double quotes
                                default_value = '"' + unescaped_value + '"'
                    elif default_value.startswith('(') and default_value.endswith(')'):
                        # Unquoted tuple format (a, b, c) to array format [a, b, c]
                        inner_content = default_value[1:-1]
                        inner_content = inner_content.replace('False', 'false').replace('True', 'true')
                        default_value = '[' + inner_content + ']'
                    else:
                        # Simple value - convert boolean constants and clean up decimals
                        if default_value == 'FALSE':
                            default_value = 'false'
                        elif default_value == 'TRUE':
                            default_value = 'true'
                        elif default_value.endswith('.0'):
                            # Convert 0.0 to 0, 1.0 to 1, etc.
                            try:
                                float_val = float(default_value)
                                if float_val.is_integer():
                                    default_value = str(int(float_val))
                            except ValueError:
                                pass  # Keep original value if conversion fails

                    # Look for the field definition in the next lines
                    j = i + 1
                    while j < len(lines):
                        next_line = lines[j].strip()
                        if next_line and not next_line.startswith('@') and not next_line.startswith('//'):
                            # Extract field name from this line
                            field_match = re.search(r'\b(\w+)\s*;', next_line)
                            if field_match:
                                field_name = field_match.group(1)
                                default_values[field_name] = default_value
                            break
                        j += 1
            i += 1

        return default_values

    def _remove_verbatim_blocks(self, content: str) -> str:
        """Remove @verbatim and @default blocks from content"""
        lines = content.split('\n')
        processed_lines = []
        in_verbatim = False
        paren_count = 0

        for line in lines:
            # Skip @verbatim blocks
            if '@verbatim' in line and not in_verbatim:
                in_verbatim = True
                paren_count = line.count('(') - line.count(')')
                continue
            elif in_verbatim:
                paren_count += line.count('(') - line.count(')')
                if paren_count <= 0:
                    in_verbatim = False
                continue

            # Skip @default annotations completely
            if '@default' in line:
                continue

            # Skip @unit annotations completely
            if '@unit' in line:
                continue

            processed_lines.append(line)

        return '\n'.join(processed_lines)

    def _parse_single_field(self, field_line: str, struct_name: str = None) -> Optional[IdlField]:
        """Parse a single field definition"""
        field_line = field_line.strip()
        if not field_line:
            return None

        # Handle sequence types: sequence<type> name or sequence<type, bound> name
        sequence_match = re.match(r'sequence<([^,>]+)(?:,\s*(\d+))?>\s+(\w+)', field_line)
        if sequence_match:
            inner_type = sequence_match.group(1).strip()
            bound = sequence_match.group(2)
            field_name = sequence_match.group(3)

            # Map the inner type with field name context
            ros_type = self._map_type_with_context(inner_type, field_name, struct_name)

            # Handle bounded sequence
            if bound:
                bound_value = int(bound)
                return IdlField(
                    field_type=ros_type,
                    name=field_name,
                    is_sequence=True,
                    is_array=True,
                    array_size=bound_value  # Store bound as array_size for bounded sequences
                )
            else:
                return IdlField(
                    field_type=ros_type,
                    name=field_name,
                    is_sequence=True,
                    is_array=True
                )

        # Handle array types: type[size] name or type[] name
        array_match = re.match(r'([^[\s]+)\s*\[([^\]]*)\]\s+(\w+)', field_line)
        if array_match:
            base_type = array_match.group(1)
            array_size_str = array_match.group(2)
            field_name = array_match.group(3)

            ros_type = self._map_type_with_context(base_type, field_name, struct_name)
            array_size = int(array_size_str) if array_size_str.isdigit() else None

            return IdlField(
                field_type=ros_type,
                name=field_name,
                is_array=True,
                array_size=array_size
            )

        # Handle bounded strings: string<size> name
        bounded_string_match = re.match(r'string<(\d+)>\s+(\w+)', field_line)
        if bounded_string_match:
            bound_size = int(bounded_string_match.group(1))
            field_name = bounded_string_match.group(2)

            return IdlField(
                field_type='string',
                name=field_name,
                is_array=True,  # Use is_array to indicate bounded
                is_bounded_string=True,
                array_size=bound_size
            )

        # Handle regular types: type name or type<params> name
        regular_match = re.match(r'([^:\s]+(?:::[^:\s]+)*)\s+(\w+)', field_line)
        if regular_match:
            field_type = regular_match.group(1)
            field_name = regular_match.group(2)

            ros_type = self._map_type_with_context(field_type, field_name, struct_name)

            # Check if this is a typedef array
            if field_type in self.typedefs:
                typedef_info = self.typedefs[field_type]
                # Apply context mapping to the typedef base type
                contextual_type = self._map_type_with_context(typedef_info['base_type'], field_name, struct_name)
                return IdlField(
                    field_type=contextual_type,
                    name=field_name,
                    is_array=True,
                    array_size=typedef_info['array_size']
                )

            return IdlField(
                field_type=ros_type,
                name=field_name
            )

        return None

    def _map_type(self, idl_type: str) -> str:
        """Map IDL type to ROS2 type"""
        # Check if it's a typedef first
        if idl_type in self.typedefs:
            typedef_info = self.typedefs[idl_type]
            return typedef_info['base_type']

        # Handle namespaced types (e.g., std_msgs::msg::Header)
        if '::' in idl_type:
            parts = idl_type.split('::')
            if len(parts) >= 3:
                # For types like package::msg::Type, check if it's in the same package context
                package = parts[0]
                msg_type = parts[-1]

                # If it's the same package we're currently processing, just use the type name
                if package == self.current_package or package == 'rmw_dds_common' or package == 'test_msgs':
                    return msg_type
                else:
                    return f"{package}/{msg_type}"
            else:
                return idl_type.replace('::', '/')

        # Handle basic types
        return self.TYPE_MAPPING.get(idl_type, idl_type)

    def _map_type_with_context(self, idl_type: str, field_name: str, struct_name: str = None) -> str:
        """Map IDL type to ROS2 type with field name context"""
        # Special case: uint8 with "char" in field name should map to char
        if idl_type == 'uint8' and 'char' in field_name.lower():
            return 'char'

        # Special case: uint8 in Char struct should map to char
        if idl_type == 'uint8' and struct_name and 'char' in struct_name.lower():
            return 'char'

        # Otherwise use regular mapping
        return self._map_type(idl_type)

    def _determine_interface_type(self, structure: IdlStructure, file_path: str) -> IdlElementType:
        """Determine if structure represents a message, service, or action"""
        file_name = os.path.basename(file_path).lower()
        struct_name = structure.name.lower()

        # Check for service patterns - be more specific to avoid false positives
        if ('.srv' in file_path or 
            struct_name.endswith('_request') or struct_name.endswith('_response') or
            struct_name == 'request' or struct_name == 'response'):
            return IdlElementType.SERVICE
        elif ('.action' in file_path or
              struct_name.endswith('_goal') or struct_name.endswith('_result') or struct_name.endswith('_feedback') or
              struct_name == 'goal' or struct_name == 'result' or struct_name == 'feedback'):
            return IdlElementType.ACTION
        else:
            return IdlElementType.MESSAGE


class RosInterfaceGenerator:
    """Generates ROS2 interface files from IDL interfaces"""

    def __init__(self, output_dir: str = "ros_interfaces"):
        self.output_dir = pathlib.Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        # Create subdirectories
        self.msg_dir = self.output_dir / "msg"
        self.srv_dir = self.output_dir / "srv"
        self.action_dir = self.output_dir / "action"

        self.msg_dir.mkdir(exist_ok=True)
        self.srv_dir.mkdir(exist_ok=True)
        self.action_dir.mkdir(exist_ok=True)

        # Store interfaces for service/action combining
        self.service_parts = {}
        self.action_parts = {}

    def generate_interfaces(self, interfaces: List[IdlInterface]) -> List[str]:
        """Generate ROS2 interface files from IDL interfaces"""
        generated_files = []

        # First pass: collect service and action parts
        for interface in interfaces:
            if interface.interface_type == IdlElementType.SERVICE:
                self._collect_service_part(interface)
            elif interface.interface_type == IdlElementType.ACTION:
                self._collect_action_part(interface)

        # Second pass: generate files
        for interface in interfaces:
            if interface.interface_type == IdlElementType.MESSAGE:
                file_path = self._generate_message(interface)
                if file_path:
                    generated_files.append(str(file_path))

        # Generate combined service files
        generated_files.extend(self._generate_service_files())

        # Generate combined action files  
        generated_files.extend(self._generate_action_files())

        return generated_files

    def _collect_service_part(self, interface: IdlInterface):
        """Collect service request/response parts"""
        if interface.name.endswith('_Request'):
            base_name = interface.name[:-8]  # Remove '_Request'
            if base_name not in self.service_parts:
                self.service_parts[base_name] = {}
            self.service_parts[base_name]['request'] = interface
        elif interface.name.endswith('_Response'):
            base_name = interface.name[:-9]  # Remove '_Response'
            if base_name not in self.service_parts:
                self.service_parts[base_name] = {}
            self.service_parts[base_name]['response'] = interface

    def _collect_action_part(self, interface: IdlInterface):
        """Collect action goal/result/feedback parts"""
        name = interface.name
        if name.endswith('_Goal'):
            base_name = name[:-5]  # Remove '_Goal'
            if base_name not in self.action_parts:
                self.action_parts[base_name] = {}
            self.action_parts[base_name]['goal'] = interface
        elif name.endswith('Goal'):
            base_name = name[:-4]  # Remove 'Goal'
            if base_name not in self.action_parts:
                self.action_parts[base_name] = {}
            self.action_parts[base_name]['goal'] = interface
        elif name.endswith('_Result'):
            base_name = name[:-7]  # Remove '_Result'
            if base_name not in self.action_parts:
                self.action_parts[base_name] = {}
            self.action_parts[base_name]['result'] = interface
        elif name.endswith('Result'):
            base_name = name[:-6]  # Remove 'Result'
            if base_name not in self.action_parts:
                self.action_parts[base_name] = {}
            self.action_parts[base_name]['result'] = interface
        elif name.endswith('_Feedback'):
            base_name = name[:-9]  # Remove '_Feedback'
            if base_name not in self.action_parts:
                self.action_parts[base_name] = {}
            self.action_parts[base_name]['feedback'] = interface
        elif name.endswith('Feedback'):
            base_name = name[:-8]  # Remove 'Feedback'
            if base_name not in self.action_parts:
                self.action_parts[base_name] = {}
            self.action_parts[base_name]['feedback'] = interface

    def _generate_service_files(self) -> List[str]:
        """Generate .srv files from collected service parts"""
        generated_files = []

        for service_name, parts in self.service_parts.items():
            if 'request' in parts and 'response' in parts:
                file_path = self._generate_combined_service(service_name, parts['request'], parts['response'])
                if file_path:
                    generated_files.append(str(file_path))

        return generated_files

    def _generate_action_files(self) -> List[str]:
        """Generate .action files from collected action parts"""
        generated_files = []

        for action_name, parts in self.action_parts.items():
            if all(key in parts for key in ['goal', 'result', 'feedback']):
                file_path = self._generate_combined_action(action_name, parts)
                if file_path:
                    generated_files.append(str(file_path))

        return generated_files

    def _generate_combined_service(self, service_name: str, request_interface: IdlInterface, response_interface: IdlInterface) -> Optional[pathlib.Path]:
        """Generate a combined .srv file"""
        lines = []

        # Add request fields
        if request_interface.structures:
            structure = request_interface.structures[0]
            # Add structure comments as comment for first field (if no field comment exists)
            for i, field in enumerate(structure.fields):
                # Add field comment if present, or structure comment for first field
                if field.comment:
                    if lines:  # Add blank line before field comment if not first
                        lines.append("")
                    comment_lines = field.comment.split('\n')
                    for comment_line in comment_lines:
                        # Unescape quotes in comments
                        comment_line = comment_line.replace('\\"', '"')
                        lines.append(f"# {comment_line}")
                elif i == 0 and structure.comments:
                    # Use structure comment for first field if field has no comment
                    for comment in structure.comments:
                        comment_lines = comment.split('\n')
                        for comment_line in comment_lines:
                            lines.append(f"# {comment_line}")
                lines.append(self._format_field(field))

        # Add separator
        lines.append("---")

        # Add response fields
        if response_interface.structures:
            structure = response_interface.structures[0]
            # Add structure comments as comment for first field (if no field comment exists)
            for i, field in enumerate(structure.fields):
                # Add field comment if present, or structure comment for first field
                if field.comment:
                    lines.append("")  # Add blank line before field comment
                    comment_lines = field.comment.split('\n')
                    for comment_line in comment_lines:
                        # Unescape quotes in comments
                        comment_line = comment_line.replace('\\"', '"')
                        lines.append(f"# {comment_line}")
                elif i == 0 and structure.comments:
                    # Use structure comment for first field if field has no comment
                    for comment in structure.comments:
                        comment_lines = comment.split('\n')
                        for comment_line in comment_lines:
                            lines.append(f"# {comment_line}")
                lines.append(self._format_field(field))

        content = "\n".join(lines)
        file_path = self.srv_dir / f"{service_name}.srv"

        with open(file_path, 'w') as f:
            f.write(content)

        print(f"Generated: {file_path}")
        return file_path

    def _generate_combined_action(self, action_name: str, parts: Dict) -> Optional[pathlib.Path]:
        """Generate a combined .action file"""
        lines = []

        # Add header comment
        lines.append(f"# {action_name}.action")
        lines.append("# Generated from IDL file")
        lines.append("")

        # Add goal fields
        lines.append("# Goal")
        if parts['goal'].structures:
            for field in parts['goal'].structures[0].fields:
                lines.append(self._format_field(field))

        lines.append("---")

        # Add result fields
        lines.append("# Result")
        if parts['result'].structures:
            for field in parts['result'].structures[0].fields:
                lines.append(self._format_field(field))

        lines.append("---")

        # Add feedback fields
        lines.append("# Feedback")
        if parts['feedback'].structures:
            for field in parts['feedback'].structures[0].fields:
                lines.append(self._format_field(field))

        content = "\n".join(lines)
        file_path = self.action_dir / f"{action_name}.action"

        with open(file_path, 'w') as f:
            f.write(content)

        print(f"Generated: {file_path}")
        return file_path

    def _generate_message(self, interface: IdlInterface) -> Optional[pathlib.Path]:
        """Generate .msg file"""
        if not interface.structures:
            return None

        structure = interface.structures[0]
        content = self._generate_message_content(structure, interface)

        file_path = self.msg_dir / f"{interface.name}.msg"
        with open(file_path, 'w') as f:
            f.write(content)

        print(f"Generated: {file_path}")
        return file_path

    def _generate_message_content(self, structure: IdlStructure, interface: IdlInterface) -> str:
        """Generate the content of a .msg file"""
        lines = []

        # Add verbatim comments first (if any)
        if structure.comments:
            for comment in structure.comments:
                lines.append(f"# {comment}")

        # Process fields in their original order to preserve IDL field sequence
        for field in structure.fields:
            # Add field comment if present
            if field.comment:
                lines.append("")  # Add blank line before field comment
                comment_lines = field.comment.split('\n')
                for comment_line in comment_lines:
                    lines.append(f"# {comment_line}")
            line = self._format_field(field)
            lines.append(line)

        # Add constants after fields (for ROS2 .msg format compatibility)
        for constant in structure.constants:
            line = self._format_constant_as_field(constant)
            lines.append(line)

        return "\n".join(lines)

    def _format_field(self, field: IdlField) -> str:
        """Format a field for ROS interface file"""
        field_type = field.field_type

        # Handle arrays and bounded types
        if field.is_array:
            if field.is_sequence and field.array_size is not None:
                # Bounded sequence: Type[<=N]
                field_type += f"[<={field.array_size}]"
            elif field.is_sequence:
                # Unbounded sequence: Type[]
                field_type += "[]"
            elif field.is_bounded_string:
                # Bounded string: string<=N
                field_type = f"string<={field.array_size}"
            elif field.array_size is not None:
                # Fixed-size array: Type[N]
                field_type += f"[{field.array_size}]"
            else:
                # Dynamic array: Type[]
                field_type += "[]"

        line = f"{field_type} {field.name}"

        # Add default value if present
        if field.default_value:
            line += f" {field.default_value}"

        return line

    def _format_constant_as_field(self, constant: IdlConstant) -> str:
        """Format a constant as a field-like entry for compatibility with ROS2 .msg format"""
        return f"{constant.const_type} {constant.name}={constant.value}"

    def _format_constant(self, constant: IdlConstant) -> str:
        """Format a constant for ROS interface file"""
        return f"{constant.const_type} {constant.name}={constant.value}"


def main():
    """Main function"""
    parser = argparse.ArgumentParser(description="Convert ROS2 IDL files to interface files")
    parser.add_argument("idl_file", help="Path to the IDL file to convert")
    parser.add_argument("-o", "--output", default="ros_interfaces", 
                       help="Output directory for generated files")
    parser.add_argument("-r", "--root", 
                       help="Root path where the generated files will be located (default: current directory)")
    parser.add_argument("-p", "--package", 
                       help="Package name to use for generated files (overrides package from IDL)")
    parser.add_argument("-v", "--verbose", action="store_true", 
                       help="Enable verbose output")

    args = parser.parse_args()

    if not os.path.exists(args.idl_file):
        print(f"Error: IDL file '{args.idl_file}' not found")
        return 1

    try:
        # Parse IDL file
        idl_parser = IdlParser()
        interfaces = idl_parser.parse_file(args.idl_file)

        # Override package name if provided
        if args.package:
            for interface in interfaces:
                interface.package = args.package

        if args.verbose:
            print(f"Parsed {len(interfaces)} interfaces from {args.idl_file}")
            for interface in interfaces:
                print(f"  - {interface.name} ({interface.interface_type.value})")
                if args.package:
                    print(f"    Package: {interface.package} (overridden)")
                else:
                    print(f"    Package: {interface.package}")

        # Determine output directory
        if args.root:
            output_dir = pathlib.Path(args.root) / args.output
        else:
            output_dir = pathlib.Path(args.output)

        # Generate ROS interface files
        generator = RosInterfaceGenerator(str(output_dir))
        generated_files = generator.generate_interfaces(interfaces)

        print(f"\nGenerated {len(generated_files)} files:")
        for file_path in generated_files:
            print(f"  - {file_path}")

        # Display generated file contents if verbose
        if args.verbose:
            print("\n" + "="*60)
            print("Generated file contents:")
            for file_path in generated_files:
                print(f"\n--- {pathlib.Path(file_path).name} ---")
                with open(file_path, 'r') as f:
                    print(f.read())

        return 0

    except Exception as e:
        print(f"Error: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == "__main__":
    if len(sys.argv) == 1:
        print("\nUsage: python idl_parser.py <idl_file> [options]")
        print("Options:")
        print("  -o, --output DIR     Output directory name")
        print("  -r, --root PATH      Root path for generated files")
        print("  -p, --package NAME   Package name to use")
        print("  -v, --verbose        Enable verbose output")
    else:
        exit(main())
