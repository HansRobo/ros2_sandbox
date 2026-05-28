cmake_minimum_required(VERSION 3.14)
project(rosidl_generator_validated_cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_cmake REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install CLI script
install(PROGRAMS
  bin/rosidl_generator_validated_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install resource (templates)
install(DIRECTORY resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

# Install cmake files
install(FILES
  cmake/rosidl_generator_validated_cpp_generate_interfaces.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Configure and install extras file
set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
configure_file(
  cmake/rosidl_generator_validated_cpp-extras.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/rosidl_generator_validated_cpp-extras.cmake
  @ONLY
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/rosidl_generator_validated_cpp-extras.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_package(
  CONFIG_EXTRAS
    "cmake/rosidl_generator_validated_cpp-extras.cmake.in"
)
