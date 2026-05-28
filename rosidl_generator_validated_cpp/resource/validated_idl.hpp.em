@# Generation template for validated C++ type adapters
@#
@# Context variables:
@#   - package_name (str): Name of the package
@#   - interface_path (Path): Path to the IDL file relative to package
@#   - content (IdlContent): Parsed IDL content
@#   - get_validation_info (function): Get validation info from member
@#   - get_cpp_type (function): Get C++ type string
@#   - is_numeric_type (function): Check if type is numeric
@#   - is_array_type (function): Check if type is array
@#   - is_sequence_type (function): Check if type is sequence
@#   - is_string_type (function): Check if type is string
@#   - is_bounded_string (function): Check if type is bounded string
@#   - get_string_bound (function): Get string bound
@#   - get_array_size (function): Get array size
@{
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore
from rosidl_parser.definition import Message

# Get message from content
messages = list(content.get_elements_of_type(Message))
}@
@[for message in messages]@
@{
struct_name = message.structure.namespaced_type.name
header_name = convert_camel_case_to_lower_case_underscore(struct_name)
include_guard = (package_name + '__MSG__VALIDATED__' + struct_name + '__VALIDATED_HPP_').upper()
}@
// Copyright 2024 Hans
// Licensed under the Apache License, Version 2.0

// Auto-generated validated type adapter for @(package_name)::msg::@(struct_name)
// DO NOT EDIT MANUALLY

#ifndef @(include_guard)
#define @(include_guard)

#include <array>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/type_adapter.hpp"
#include "@(package_name)/msg/@(header_name).hpp"

namespace @(package_name)
{
namespace msg
{
namespace validated
{

/// Exception thrown when validation fails
class ValidationError : public std::runtime_error
{
public:
  explicit ValidationError(const std::string & message)
  : std::runtime_error(message) {}
};

/// Validated wrapper for @(struct_name)
///
/// This class wraps the ROS message type and provides validation
/// based on IDL annotations (@range, @@min, @@max).
/// Validation is performed when converting to ROS message (e.g., on publish).
struct @(struct_name)Validated
{
  using ROSMessage = @(package_name)::msg::@(struct_name);

  // Member variables (same as ROS message)
@[for member in message.structure.members]@
@{
cpp_type = get_cpp_type(member.type)
validation = get_validation_info(member)
unit_comment = ''
if validation['unit']:
    unit_comment = '  // unit: ' + validation['unit']
}@
  @(cpp_type) @(member.name){};@(unit_comment)
@[end for]@

  /// Default constructor
  @(struct_name)Validated() = default;

  /// Constructor from ROS message
  explicit @(struct_name)Validated(const ROSMessage & msg)
  {
@[for member in message.structure.members]@
    @(member.name) = msg.@(member.name);
@[end for]@
  }

  /// Validate all fields and throw ValidationError if invalid
  ///
  /// @@throws ValidationError if any field is out of range
  void validate() const
  {
@[for member in message.structure.members]@
@{
validation = get_validation_info(member)
is_array = is_array_type(member.type)
is_seq = is_sequence_type(member.type)
is_numeric = is_numeric_type(member.type)
is_str = is_string_type(member.type)
}@
@[  if validation['has_range'] and is_numeric]@
@[    if is_array or is_seq]@
    // Validate @(member.name) (array/sequence elements)
    for (size_t i = 0; i < @(member.name).size(); ++i) {
@[      if validation['min_value'] is not None]@
      if (@(member.name)[i] < @(validation['min_value'])) {
        std::ostringstream oss;
        oss << "Field '@(member.name)[" << i << "]' value " << @(member.name)[i]
            << " is below minimum @(validation['min_value'])";
        throw ValidationError(oss.str());
      }
@[      end if]@
@[      if validation['max_value'] is not None]@
      if (@(member.name)[i] > @(validation['max_value'])) {
        std::ostringstream oss;
        oss << "Field '@(member.name)[" << i << "]' value " << @(member.name)[i]
            << " exceeds maximum @(validation['max_value'])";
        throw ValidationError(oss.str());
      }
@[      end if]@
    }
@[    else]@
    // Validate @(member.name)
@[      if validation['min_value'] is not None]@
    if (@(member.name) < @(validation['min_value'])) {
      std::ostringstream oss;
      oss << "Field '@(member.name)' value " << @(member.name)
          << " is below minimum @(validation['min_value'])";
      throw ValidationError(oss.str());
    }
@[      end if]@
@[      if validation['max_value'] is not None]@
    if (@(member.name) > @(validation['max_value'])) {
      std::ostringstream oss;
      oss << "Field '@(member.name)' value " << @(member.name)
          << " exceeds maximum @(validation['max_value'])";
      throw ValidationError(oss.str());
    }
@[      end if]@
@[    end if]@
@[  end if]@
@[  if validation['has_string_length']]@
    // Validate @(member.name) string length
    if (@(member.name).length() > @(validation['max_string_length'])) {
      std::ostringstream oss;
      oss << "Field '@(member.name)' length " << @(member.name).length()
          << " exceeds maximum @(validation['max_string_length'])";
      throw ValidationError(oss.str());
    }
@[  end if]@
@[end for]@
  }

  /// Check if all fields are valid (non-throwing)
  ///
  /// @@return true if all fields pass validation
  bool is_valid() const noexcept
  {
    try {
      validate();
      return true;
    } catch (const ValidationError &) {
      return false;
    }
  }

  /// Create from ROS message (no validation)
  ///
  /// @@param msg The ROS message to copy from
  /// @@return A new validated wrapper instance
  static @(struct_name)Validated from_ros_message(const ROSMessage & msg)
  {
    return @(struct_name)Validated(msg);
  }

  /// Convert to ROS message with validation
  ///
  /// @@return The ROS message
  /// @@throws ValidationError if any field is out of range
  ROSMessage to_ros_message() const
  {
    validate();  // Validate before conversion
    ROSMessage msg;
@[for member in message.structure.members]@
    msg.@(member.name) = @(member.name);
@[end for]@
    return msg;
  }

  /// Convert to ROS message without validation
  ///
  /// @@return The ROS message (may contain invalid values)
  ROSMessage to_ros_message_unchecked() const noexcept
  {
    ROSMessage msg;
@[for member in message.structure.members]@
    msg.@(member.name) = @(member.name);
@[end for]@
    return msg;
  }
};

}  // namespace validated
}  // namespace msg
}  // namespace @(package_name)

// Type Adapter specialization for rclcpp
template<>
struct rclcpp::TypeAdapter<
  @(package_name)::msg::validated::@(struct_name)Validated,
  @(package_name)::msg::@(struct_name)>
{
  using is_specialized = std::true_type;
  using custom_type = @(package_name)::msg::validated::@(struct_name)Validated;
  using ros_message_type = @(package_name)::msg::@(struct_name);

  /// Convert from custom type to ROS message (validation happens here)
  static void convert_to_ros_message(
    const custom_type & source,
    ros_message_type & destination)
  {
    destination = source.to_ros_message();  // Validation inside
  }

  /// Convert from ROS message to custom type (no validation)
  static void convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
  {
    destination = custom_type::from_ros_message(source);
  }
};

// Convenience type alias
using @(struct_name)Adapted = rclcpp::TypeAdapter<
  @(package_name)::msg::validated::@(struct_name)Validated,
  @(package_name)::msg::@(struct_name)>;

#endif  // @(include_guard)
@[end for]@
