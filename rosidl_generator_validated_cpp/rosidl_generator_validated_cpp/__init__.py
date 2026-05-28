# Copyright 2024 Hans
#
# Licensed under the Apache License, Version 2.0

"""Generate validated C++ type adapters from ROS interfaces."""

from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedString
from rosidl_parser.definition import NamespacedType
from rosidl_pycommon import generate_files


def generate_validated_cpp(generator_arguments_file):
    """Generate validated C++ type adapters.

    Args:
        generator_arguments_file: Path to JSON file with generator arguments.

    Returns:
        List of generated files.
    """
    mapping = {
        'validated_idl.hpp.em': 'validated/%s__validated.hpp',
    }

    additional_context = {
        'get_validation_info': get_validation_info,
        'get_cpp_type': get_cpp_type,
        'is_numeric_type': is_numeric_type,
        'is_array_type': is_array_type,
        'is_sequence_type': is_sequence_type,
        'is_string_type': is_string_type,
        'is_bounded_string': is_bounded_string,
        'get_string_bound': get_string_bound,
        'get_array_size': get_array_size,
    }

    return generate_files(
        generator_arguments_file, mapping,
        additional_context=additional_context,
        post_process_callback=prefix_with_bom_if_necessary)


def prefix_with_bom_if_necessary(content):
    """Add BOM if content contains non-ASCII characters."""
    try:
        content.encode('ASCII')
    except UnicodeError:
        prefix = '\ufeff' + \
            '// NOLINT: This file starts with a BOM ' + \
            'since it contain non-ASCII characters\n'
        content = prefix + content
    return content


# C++ type mapping (simplified, without allocator)
MSG_TYPE_TO_CPP = {
    'boolean': 'bool',
    'octet': 'unsigned char',
    'char': 'unsigned char',
    'wchar': 'char16_t',
    'float': 'float',
    'double': 'double',
    'long double': 'long double',
    'uint8': 'uint8_t',
    'int8': 'int8_t',
    'uint16': 'uint16_t',
    'int16': 'int16_t',
    'uint32': 'uint32_t',
    'int32': 'int32_t',
    'uint64': 'uint64_t',
    'int64': 'int64_t',
    'string': 'std::string',
    'wstring': 'std::u16string',
}


def get_cpp_type(type_):
    """Convert rosidl type to C++ type string."""
    if isinstance(type_, AbstractNestedType):
        value_type = type_.value_type
    else:
        value_type = type_

    if isinstance(value_type, BasicType):
        cpp_type = MSG_TYPE_TO_CPP[value_type.typename]
    elif isinstance(value_type, AbstractString):
        cpp_type = 'std::string'
    elif isinstance(value_type, AbstractWString):
        cpp_type = 'std::u16string'
    elif isinstance(value_type, NamespacedType):
        cpp_type = '::'.join(value_type.namespaced_name())
    else:
        cpp_type = 'unknown'

    if isinstance(type_, Array):
        return f'std::array<{cpp_type}, {type_.size}>'
    elif isinstance(type_, AbstractSequence):
        return f'std::vector<{cpp_type}>'
    else:
        return cpp_type


def is_numeric_type(type_):
    """Check if type is a numeric type that can have range validation."""
    if isinstance(type_, AbstractNestedType):
        type_ = type_.value_type
    if isinstance(type_, BasicType):
        return type_.typename not in ('boolean', 'octet', 'char', 'wchar')
    return False


def is_array_type(type_):
    """Check if type is a fixed-size array."""
    return isinstance(type_, Array)


def is_sequence_type(type_):
    """Check if type is a sequence (dynamic array)."""
    return isinstance(type_, AbstractSequence)


def is_string_type(type_):
    """Check if type is a string."""
    if isinstance(type_, AbstractNestedType):
        type_ = type_.value_type
    return isinstance(type_, (AbstractString, AbstractWString))


def is_bounded_string(type_):
    """Check if type is a bounded string."""
    if isinstance(type_, AbstractNestedType):
        type_ = type_.value_type
    return isinstance(type_, BoundedString)


def get_string_bound(type_):
    """Get the maximum length of a bounded string."""
    if isinstance(type_, AbstractNestedType):
        type_ = type_.value_type
    if isinstance(type_, BoundedString):
        return type_.maximum_size
    return None


def get_array_size(type_):
    """Get the size of a fixed-size array."""
    if isinstance(type_, Array):
        return type_.size
    return None


def get_validation_info(member):
    """Extract validation information from IDL annotations.

    Supported annotations:
        @range(min=X, max=Y) - Range validation
        @min(value=X) or @min(X) - Minimum value
        @max(value=X) or @max(X) - Maximum value
        @unit(value="X") or @unit("X") - Unit annotation

    Args:
        member: A rosidl_parser.definition.Member object.

    Returns:
        A dictionary with validation information.
    """
    info = {
        'has_range': False,
        'min_value': None,
        'max_value': None,
        'unit': None,
        'has_string_length': False,
        'max_string_length': None,
    }

    # @range(min=X, max=Y) annotation
    if member.has_annotation('range'):
        value = member.get_annotation_value('range')
        info['has_range'] = True
        if isinstance(value, dict):
            info['min_value'] = value.get('min')
            info['max_value'] = value.get('max')

    # @min(value=X) or @min(X) annotation
    if member.has_annotation('min'):
        info['has_range'] = True
        value = member.get_annotation_value('min')
        if isinstance(value, dict):
            info['min_value'] = value.get('value')
        else:
            info['min_value'] = value

    # @max(value=X) or @max(X) annotation
    if member.has_annotation('max'):
        info['has_range'] = True
        value = member.get_annotation_value('max')
        if isinstance(value, dict):
            info['max_value'] = value.get('value')
        else:
            info['max_value'] = value

    # @unit(value="X") or @unit("X") annotation
    if member.has_annotation('unit'):
        value = member.get_annotation_value('unit')
        if isinstance(value, dict):
            info['unit'] = value.get('value')
        else:
            info['unit'] = str(value)

    # Bounded string check
    if is_bounded_string(member.type):
        info['has_string_length'] = True
        info['max_string_length'] = get_string_bound(member.type)

    return info
