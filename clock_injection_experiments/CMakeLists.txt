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

# このパッケージはテストのみで構成され、実行ファイル/ライブラリは生成しない。

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 著作権ヘッダを各ソースに付与していないためcopyright/cpplintリンタをスキップ
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # ament_auto_add_gtestがpackage.xmlの依存(rclcpp/rcl/rosgraph_msgs)を自動リンクする。

  # --- シナリオA: rcl APIによる直接時刻注入の基本動作 ---
  ament_auto_add_gtest(test_direct_clock_injection
    test/test_direct_clock_injection.cpp)

  # --- シナリオB: TimeSourceからdetachClockし/clockを遮断 ---
  ament_auto_add_gtest(test_clock_isolation
    test/test_clock_isolation.cpp
    TIMEOUT 60)
endif()

ament_auto_package()
