cmake_minimum_required(VERSION 3.5)
project(sample_interfaces)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ament_auto_generate_code()には2つの使い方があります：
# 1. TARGETS引数を指定する場合：指定されたターゲットのみに型サポートライブラリをリンク
# 2. TARGETS引数を指定しない場合：全てのターゲットに自動的に型サポートライブラリをリンク

# パターン1: TARGETS引数を指定する場合（特定のターゲットのみにリンク）
ament_auto_add_executable(${PROJECT_NAME}_node src/node.cpp)
ament_auto_add_executable(${PROJECT_NAME}_node2 src/node.cpp)
ament_auto_add_executable(${PROJECT_NAME}_node_no_targets src/node_no_targets.cpp)

# TARGETS引数を使って、node と node2 のみに型サポートライブラリをリンク
# node_no_targets は含めない（手動リンクまたはTARGETS引数なしのパターンでテスト）
ament_auto_generate_code(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_node2)

# node_no_targets用に手動でリンク
# （本来は ament_auto_generate_code() で自動化されるが、TARGETS引数の違いを示すため）
target_link_libraries(${PROJECT_NAME}_node_no_targets
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)

# パターン2: TARGETS引数を指定しない場合（全ターゲットに自動リンク）
# 以下のようにすると、全てのament_auto_add_executable()で作成された
# ターゲットに自動的に型サポートライブラリがリンクされます：
# ament_auto_generate_code()

ament_auto_package()
