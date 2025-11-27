cmake_minimum_required(VERSION 3.12)
project(sample_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node src/node.cpp)
ament_auto_add_executable(${PROJECT_NAME}_node2 src/node.cpp)
ament_auto_add_executable(${PROJECT_NAME}_node_no_targets src/node_no_targets.cpp)

rosidl_auto_generate_interfaces(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_node2)

target_link_libraries(${PROJECT_NAME}_node_no_targets
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

ament_auto_package()
