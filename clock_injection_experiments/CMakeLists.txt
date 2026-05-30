cmake_minimum_required(VERSION 3.8)
project(clock_injection_experiments)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# /clockを購読せず直接時刻注入のみでClockを駆動するカスタムTimeSource。
# include/配下のヘッダはament_cmake_autoが自動でインクルードパスへ加える。
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/injection_time_source.cpp
  src/node_internal_time_source_detacher.cpp)

# InjectionTimeSourceでrclcpp::Node::now()を駆動するデモ実行ファイル。
ament_auto_add_executable(node_clock_injection_example
  src/node_clock_injection_example.cpp)
target_link_libraries(node_clock_injection_example ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 著作権ヘッダを各ソースに付与していないためcopyright/cpplintリンタをスキップ
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # ament_auto_add_gtestはpackage.xmlの依存(rclcpp/rcl/rosgraph_msgs)を自動リンクするが、
  # ローカルライブラリ(${PROJECT_NAME})はtarget_link_librariesで明示リンクする必要がある。

  # --- シナリオA: InjectionTimeSourceによる直接時刻注入の基本動作 ---
  ament_auto_add_gtest(test_direct_clock_injection
    test/test_direct_clock_injection.cpp)
  target_link_libraries(test_direct_clock_injection ${PROJECT_NAME})

  # --- シナリオB: /clockを購読しないことによる構造的遮断 ---
  ament_auto_add_gtest(test_clock_isolation
    test/test_clock_isolation.cpp
    TIMEOUT 60)
  target_link_libraries(test_clock_isolation ${PROJECT_NAME})

  # --- シナリオC: rclcpp::Node::now()のClockへの注入 ---
  ament_auto_add_gtest(test_node_clock_injection
    test/test_node_clock_injection.cpp
    TIMEOUT 60)
  target_link_libraries(test_node_clock_injection ${PROJECT_NAME})

  # --- シナリオD: 内部TimeSourceからdetachして/clockを遮断(上級者向け) ---
  ament_auto_add_gtest(test_node_internal_detach
    test/test_node_internal_detach.cpp
    TIMEOUT 60)
  target_link_libraries(test_node_internal_detach ${PROJECT_NAME})
endif()

ament_auto_package()
