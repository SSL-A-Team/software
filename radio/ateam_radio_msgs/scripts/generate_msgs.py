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

"""Generates ROS2 message definitions from a C header file containing struct definitions."""

import pathlib
import sys

import clang.cindex


def generate_msgs_for_file(output_dir, file_path, struct_names):
    """Generate ROS2 message definitions from a C header file."""
    index = clang.cindex.Index.create()
    translation_unit = index.parse(file_path)

    enums = []

    for node in translation_unit.cursor.get_children():
        match node.kind:
            case clang.cindex.CursorKind.ENUM_DECL:
                enums.append(collect_enum_details(node))
            case clang.cindex.CursorKind.STRUCT_DECL:
                msg_name = node.spelling
                if msg_name not in struct_names:
                    continue
                declaration_file = pathlib.Path(
                    node.get_definition().location.file.name
                ).name
                msg = generate_msg_for_struct(node, enums)
                write_msg_to_file(output_dir, msg, declaration_file)
            case _:
                continue


def generate_msg_for_struct(struct_ast_node, enums):
    """Generate a ROS2 message definition for a single struct AST node."""
    type_name = struct_ast_node.spelling
    declarations = []
    for field in struct_ast_node.get_children():
        if field.kind != clang.cindex.CursorKind.FIELD_DECL:
            continue
        if field.spelling.startswith('_'):
            continue
        add_field_declarations(declarations, field, enums)
    return {'type_name': type_name, 'declarations': declarations}


def write_msg_to_file(output_dir, msg, input_file_path):
    """Write the generated ROS2 message definition to a file."""
    type_name = msg['type_name']
    declarations = msg['declarations']
    file_path = f'{output_dir}/{type_name}.msg'

    with open(file_path, 'w') as f:
        f.write(f'# Auto-generated from {input_file_path}\n')
        for declaration in declarations:
            f.write(f'{declaration}\n')


def add_field_declarations(declarations, field_node, enums):
    """Add declarations for a field node to the list of declarations."""
    if field_node.is_bitfield():
        add_bitfield_declaration(declarations, field_node)
    elif field_node.type.kind == clang.cindex.TypeKind.CONSTANTARRAY:
        add_array_declaration(declarations, field_node)
    elif field_node.type.spelling in [e['type_name'] for e in enums]:
        add_enum_declaration(declarations, field_node, enums)
    else:
        ros2_type = get_ros2_basic_type(field_node.type)
        declarations.append(f'{ros2_type} {field_node.spelling}')


def add_bitfield_declaration(declarations, field_node):
    """Add a declaration for a bitfield field node."""
    type_name = ''
    bitfield_width = field_node.get_bitfield_width()
    if bitfield_width == 1:
        type_name = 'bool'
    elif bitfield_width <= 8:
        type_name = 'uint8'
    elif bitfield_width <= 16:
        type_name = 'uint16'
    elif bitfield_width <= 32:
        type_name = 'uint32'
    elif bitfield_width <= 64:
        type_name = 'uint64'
    else:
        raise ValueError(
            f'Unsupported bitfield width: {bitfield_width} for field {field_node.spelling}'
        )
    declarations.append(f'{type_name} {field_node.spelling}')


def add_array_declaration(declarations, field_node):
    """Add a declaration for an array field node."""
    element_type = get_ros2_basic_type(field_node.type.get_array_element_type())
    declarations.append(f'{element_type}[] {field_node.spelling}')


def add_enum_declaration(declarations, field_node, enums):
    """Add a declaration for an enum field node."""
    enum_details = [e for e in enums if e['type_name'] == field_node.type.spelling][0]
    declarations.append(f'{enum_details["underlying_type"]} {field_node.spelling}')
    for value_name, value in enum_details['values']:
        declarations.append(
            f'{enum_details["underlying_type"]} {value_name.upper()} = {value}'
        )


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


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print(
            'Usage: python generate_msgs.py <output_dir> <path_to_c_header> '
            '<struct_name> [<struct_name> ...]'
        )
        sys.exit(1)
    generate_msgs_for_file(sys.argv[1], sys.argv[2], sys.argv[3:])
