cmake_minimum_required(VERSION 3.12)
project(sample_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_validated_cpp REQUIRED)
find_package(rclcpp REQUIRED)

# Generate interfaces from msg and idl files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Test.msg"
  "msg/TestA.msg"
  "msg/TestB.msg"
  "msg/SensorData.idl"
)

# Build executables
add_executable(${PROJECT_NAME}_node src/node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

add_executable(${PROJECT_NAME}_node2 src/node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node2 rclcpp)

add_executable(${PROJECT_NAME}_node_no_targets src/node_no_targets.cpp)
ament_target_dependencies(${PROJECT_NAME}_node_no_targets rclcpp)

# Link generated interfaces to executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_node2 ${cpp_typesupport_target})
target_link_libraries(${PROJECT_NAME}_node_no_targets ${cpp_typesupport_target})

# Install executables
install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_node2
  ${PROJECT_NAME}_node_no_targets
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
