# Copyright 2025 A Team
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Generates C++ conversion code for ROS2 messages from C structs."""

import os
import pathlib
import re
import sys

import clang.cindex


def generate_conversion_code(output_directory, header_file, struct_names):
    """Generate conversion functions."""
    include_root = pathlib.Path(header_file).parent
    index = clang.cindex.Index.create()
    header_tus = [
        (h, index.parse(str(h))) for h in sorted(include_root.rglob('*.h'))
    ]
    generate_header_file(output_directory, header_tus, struct_names, include_root)
    generate_implementation_file(output_directory, header_tus, struct_names)


def _nodes_in_file(translation_unit, header_path):
    """Yield only top-level nodes whose definition is in header_path."""
    path_str = str(header_path)
    for node in translation_unit.cursor.get_children():
        if node.location.file and node.location.file.name == path_str:
            yield node


def generate_header_file(output_directory, header_tus, struct_names, include_root):
    """Generate header file for conversion functions."""
    header_text = (
        '// Auto-generated conversion functions for ROS2 messages\n'
        '#ifndef CONVERSION_HPP_\n'
        '#define CONVERSION_HPP_\n\n'
    )
    for struct_name in struct_names:
        header_text += '#include <ateam_radio_msgs/msg/' \
            f'{camel_case_to_snake_case(struct_name)}.hpp>\n'
    _struct_like = (
        clang.cindex.CursorKind.STRUCT_DECL,
        clang.cindex.CursorKind.UNION_DECL,
    )
    for header_path, translation_unit in header_tus:
        for node in _nodes_in_file(translation_unit, header_path):
            if node.kind not in _struct_like:
                continue
            if node.spelling not in struct_names:
                continue
            relative_path = header_path.relative_to(include_root)
            header_text += f'#include <ateam_radio_msgs/packets/{relative_path}>\n'
    header_text += 'namespace ateam_radio_msgs {\n'
    for header_path, translation_unit in header_tus:
        for node in _nodes_in_file(translation_unit, header_path):
            if node.kind not in _struct_like:
                continue
            if node.spelling not in struct_names:
                continue
            header_text += generate_conversion_function_declaration(node) + '\n'
    header_text += '}  // namespace ateam_radio_msgs\n\n'
    header_text += '#endif  // CONVERSION_HPP_\n'
    os.makedirs(f'{output_directory}/include', exist_ok=True)
    file_path = f'{output_directory}/include/conversion.hpp'
    with open(file_path, 'w') as f:
        f.write(header_text)


def generate_implementation_file(output_directory, header_tus, struct_names):
    """Generate implementation file for conversion functions."""
    enums = []
    impl_text = '#include "conversion.hpp"\n\n' 'namespace ateam_radio_msgs {\n\n'
    for header_path, translation_unit in header_tus:
        for node in translation_unit.cursor.get_children():
            match node.kind:
                case clang.cindex.CursorKind.ENUM_DECL:
                    if node.location.file and node.location.file.name == str(header_path):
                        enums.append(collect_enum_details(node))
                case clang.cindex.CursorKind.STRUCT_DECL | clang.cindex.CursorKind.UNION_DECL:
                    if node.location.file is None \
                            or node.location.file.name != str(header_path):
                        continue
                    if node.spelling not in struct_names:
                        continue
                    impl_text += generate_conversion_function_implementation(
                        node, enums, struct_names
                    )
                case _:
                    continue
    impl_text += '}  // namespace ateam_radio_bridge\n'
    os.makedirs(f'{output_directory}/src', exist_ok=True)
    file_path = f'{output_directory}/src/conversion.cpp'
    with open(file_path, 'w') as f:
        f.write(impl_text)


def generate_conversion_function_declaration(struct_node):
    """Generate a conversion function declaration for the given struct."""
    param_name = re.sub(r'(?<!^)(?=[A-Z])', '_', struct_node.spelling).lower()
    return_type = f'ateam_radio_msgs::msg::{struct_node.spelling}'
    return f'{return_type} Convert(const {struct_node.spelling} & {param_name});'


def generate_conversion_function_implementation(struct_node, enums, struct_names):
    """Generate the implementation of a conversion function for the given struct."""
    param_name = camel_case_to_snake_case(struct_node.spelling)
    return_type = f'ateam_radio_msgs::msg::{struct_node.spelling}'
    impl_text = (
        f'{return_type} Convert(const {struct_node.spelling} & {param_name}) {{\n'
    )
    impl_text += f'    {return_type} msg;\n'
    for field in struct_node.get_children():
        if field.kind != clang.cindex.CursorKind.FIELD_DECL:
            continue
        if field.spelling.startswith('_'):
            continue
        impl_text += generate_field_copy_line(field, param_name, enums, struct_names)
    impl_text += '    return msg;\n'
    impl_text += '}\n'
    return impl_text


def generate_field_copy_line(field_node, param_name, enums, struct_names):
    """Generate a line of code to copy a field from the struct to the message."""
    field_name = field_node.spelling
    if field_node.type.kind == clang.cindex.TypeKind.CONSTANTARRAY:
        return f'    std::ranges::copy({param_name}.{field_node.spelling}, ' \
            f'std::back_inserter(msg.{field_name}));\n'
    elif field_node.type.kind == clang.cindex.TypeKind.ELABORATED:
        if field_node.type.spelling in [
            'uint8_t',
            'uint16_t',
            'uint32_t',
            'uint64_t',
            'int8_t',
            'int16_t',
            'int32_t',
            'int64_t',
        ]:
            return f'    msg.{field_name} = {param_name}.{field_node.spelling};\n'
        elif field_node.type.spelling in [e['type_name'] for e in enums]:
            return f'    msg.{field_name} = {param_name}.{field_node.spelling};\n'
        elif field_node.type.spelling in struct_names:
            return (
                f'    msg.{field_name} = Convert({param_name}.{field_node.spelling});\n'
            )
        else:
            return ''
    else:
        return f'    msg.{field_name} = {param_name}.{field_node.spelling};\n'


def collect_enum_details(enum_node):
    """Collect details of an enum AST node."""
    type_name = enum_node.spelling
    underlying_type = get_ros2_basic_type(enum_node.enum_type)
    values = []
    for child in enum_node.get_children():
        if child.kind == clang.cindex.CursorKind.ENUM_CONSTANT_DECL:
            values.append((child.spelling, child.enum_value))
    return {
        'type_name': type_name,
        'underlying_type': underlying_type,
        'values': values,
    }


def get_ros2_basic_type(field_type):
    """Map a basic C type to a ROS2 message type."""
    match field_type.kind:
        case clang.cindex.TypeKind.INT:
            return 'int32'
        case clang.cindex.TypeKind.UINT:
            return 'uint32'
        case clang.cindex.TypeKind.SHORT:
            return 'int16'
        case clang.cindex.TypeKind.USHORT:
            return 'uint16'
        case clang.cindex.TypeKind.UCHAR:
            return 'uint8'
        case clang.cindex.TypeKind.FLOAT:
            return 'float32'
        case clang.cindex.TypeKind.DOUBLE:
            return 'float64'
        case clang.cindex.TypeKind.CHAR_S:
            return 'string'
        case clang.cindex.TypeKind.BOOL:
            return 'bool'
        case clang.cindex.TypeKind.ELABORATED:
            match field_type.spelling:
                case 'uint8_t':
                    return 'uint8'
                case 'uint16_t':
                    return 'uint16'
                case 'uint32_t':
                    return 'uint32'
                case 'uint64_t':
                    return 'uint64'
                case 'int8_t':
                    return 'int8'
                case 'int16_t':
                    return 'int16'
                case 'int32_t':
                    return 'int32'
                case 'int64_t':
                    return 'int64'
                case _:
                    return 'ateam_radio_msgs/' + field_type.spelling
        case _:
            raise ValueError(f'Unsupported basic type: {field_type.spelling}')


def camel_case_to_snake_case(s):
    """Convert s (assumed to be CamelCase) to snake_case."""
    return re.sub(r'(?<!^)(?=[A-Z])', '_', s).lower()


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print(
            'Usage: generate_conversion_code.py <output_directory> <header_file> '
            '<struct_name> [<struct_name> ...]'
        )
        sys.exit(1)
    generate_conversion_code(sys.argv[1], sys.argv[2], sys.argv[3:])
