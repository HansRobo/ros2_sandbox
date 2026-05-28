# Copyright 2024 Hans
#
# Licensed under the Apache License, Version 2.0

find_package(rosidl_cmake REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_validated_cpp/${PROJECT_NAME}")
set(_generated_headers "")

foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_idl_name}" _header_name)

  list(APPEND _generated_headers
    "${_output_path}/${_parent_folder}/validated/${_header_name}__validated.hpp"
  )
endforeach()

set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_IDL_FILES})
    rosidl_find_package_idl(_abs_idl_file "${_pkg_name}" "${_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_validated_cpp_BIN}"
  ${rosidl_generator_validated_cpp_GENERATOR_FILES}
  "${rosidl_generator_validated_cpp_TEMPLATE_DIR}/validated_idl.hpp.em"
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})

foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_validated_cpp__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_validated_cpp_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0094 NEW)
set(Python3_FIND_UNVERSIONED_NAMES FIRST)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
  OUTPUT ${_generated_headers}
  COMMAND Python3::Interpreter
  ARGS ${rosidl_generator_validated_cpp_BIN}
  --generator-arguments-file "${generator_arguments_file}"
  DEPENDS ${target_dependencies}
  COMMENT "Generating validated C++ type adapters"
  VERBATIM
)

set(_target_suffix "__rosidl_generator_validated_cpp")

add_custom_target(
  ${rosidl_generate_interfaces_TARGET}__validated_cpp
  DEPENDS
  ${_generated_headers}
)

# Depend on the standard C++ generator completing first
add_dependencies(
  ${rosidl_generate_interfaces_TARGET}__validated_cpp
  ${rosidl_generate_interfaces_TARGET}__rosidl_generator_cpp)

add_library(${rosidl_generate_interfaces_TARGET}${_target_suffix} INTERFACE)
target_compile_features(${rosidl_generate_interfaces_TARGET}${_target_suffix} INTERFACE cxx_std_17)
add_library(${PROJECT_NAME}::${rosidl_generate_interfaces_TARGET}${_target_suffix} ALIAS
  ${rosidl_generate_interfaces_TARGET}${_target_suffix})

target_include_directories(${rosidl_generate_interfaces_TARGET}${_target_suffix}
  INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_validated_cpp>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Link to the standard rosidl_generator_cpp target for the base message types
target_link_libraries(
  ${rosidl_generate_interfaces_TARGET}${_target_suffix} INTERFACE
  ${rosidl_generate_interfaces_TARGET}__rosidl_generator_cpp)

add_dependencies(
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ${rosidl_generate_interfaces_TARGET}__validated_cpp)

add_dependencies(
  ${rosidl_generate_interfaces_TARGET}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix})

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  install(
    DIRECTORY ${_output_path}/
    DESTINATION "include/${PROJECT_NAME}/${PROJECT_NAME}"
    PATTERN "*.hpp"
  )

  ament_export_targets(export_${rosidl_generate_interfaces_TARGET}${_target_suffix})
  rosidl_export_typesupport_targets(${_target_suffix}
    ${rosidl_generate_interfaces_TARGET}${_target_suffix})

  install(
    TARGETS ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    EXPORT export_${rosidl_generate_interfaces_TARGET}${_target_suffix}
  )
endif()
