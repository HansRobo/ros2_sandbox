cmake_minimum_required(VERSION 3.14)
project(validated_adapter_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Demo node using validated type adapter
ament_auto_add_executable(demo_node src/demo_node.cpp)

ament_auto_package()
