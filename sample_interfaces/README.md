# sample_interfaces

ROS 2 インターフェース（メッセージ、サービス、アクション）の自動生成を示すサンプルパッケージです。

`rosidl_auto_generate_interfaces` マクロを使用して、従来の `rosidl_generate_interfaces` よりも簡潔にインターフェースを生成できます。

## Before/After 比較

### ケース1: インターフェース生成のみ（ノードなし）

他のパッケージから利用されるインターフェース専用パッケージの場合。

#### Before（従来方式）

```cmake
cmake_minimum_required(VERSION 3.12)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Test.msg"
  "msg/TestA.msg"
  "msg/TestB.msg"
  DEPENDENCIES std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

#### After（自動方式）

```cmake
cmake_minimum_required(VERSION 3.12)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_auto_generate_interfaces(ADD_LINTER_TESTS)

ament_package()
```

---

### ケース2: インターフェース + ノード（同一パッケージ）

インターフェースを定義し、同じパッケージ内のノードで使用する場合。

#### Before（従来方式）

```cmake
cmake_minimum_required(VERSION 3.12)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

# メッセージファイルを手動で列挙
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Test.msg"
  "msg/TestA.msg"
  "msg/TestB.msg"
)

# 実行ファイルを作成
add_executable(my_node src/node.cpp)
ament_target_dependencies(my_node rclcpp)

# 型サポートターゲットを手動で取得してリンク
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(my_node ${cpp_typesupport_target})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

#### After（自動方式）

```cmake
cmake_minimum_required(VERSION 3.12)
project(my_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

# 実行ファイルを作成
add_executable(my_node src/node.cpp)
ament_target_dependencies(my_node rclcpp)

# msg/, srv/, action/ を自動検出し、指定ターゲットへ自動リンク
rosidl_auto_generate_interfaces(TARGETS my_node)

ament_package()
```

---

## 主な違い

| 項目 | Before（従来方式） | After（自動方式） |
|------|-------------------|------------------|
| ファイル列挙 | 手動で全ファイル指定 | `msg/`, `srv/`, `action/` を自動検出 |
| 依存関係 | 手動で `find_package` | `ament_auto_find_build_dependencies()` |
| ターゲットリンク | `rosidl_get_typesupport_target` + `target_link_libraries` | `TARGETS` 引数で自動 |
| パッケージ処理 | `ament_package()` | `ament_auto_package()` |
| 行数 | 約20行 | 約10行 |

## package.xml の要件

`rosidl_auto_generate_interfaces` を使用するには、以下の依存関係が必須です：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_interfaces</name>
  <!-- ... -->

  <!-- 必須: ビルドツール依存 -->
  <buildtool_depend>ament_cmake_auto</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <!-- 必須: 実行時依存 -->
  <exec_depend>rosidl_default_runtime</exec_depend>

  <!-- 必須: インターフェースパッケージグループへの所属 -->
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## 使い方

### パターン1: TARGETS引数あり（推奨）

特定のターゲットに型サポートライブラリを自動リンク：

```cmake
ament_auto_add_executable(node1 src/node1.cpp)
ament_auto_add_executable(node2 src/node2.cpp)

# node1 と node2 に自動リンク
rosidl_auto_generate_interfaces(TARGETS node1 node2)
```

### パターン2: TARGETS引数なし

インターフェースのみ生成し、手動でリンク：

```cmake
# インターフェースのみ生成
rosidl_auto_generate_interfaces()

# 手動でリンク
target_link_libraries(my_target
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
```

### パターン3: 複合（一部手動リンク）

一部のターゲットは自動リンク、一部は手動リンク：

```cmake
ament_auto_add_executable(auto_linked_node src/node.cpp)
ament_auto_add_executable(manual_linked_node src/node.cpp)

# auto_linked_node のみ自動リンク
rosidl_auto_generate_interfaces(TARGETS auto_linked_node)

# manual_linked_node は手動リンク
target_link_libraries(manual_linked_node
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
```

## ディレクトリ構造

```
sample_interfaces/
├── CMakeLists.txt
├── package.xml
├── README.md
├── msg/
│   ├── Test.msg      # uint8 a, b, c, d
│   ├── TestA.msg     # string name, int32 value, float64 timestamp
│   └── TestB.msg     # bool flag, uint32[] data, string description
└── src/
    ├── node.cpp              # TARGETS引数でリンクされるノード
    └── node_no_targets.cpp   # 手動リンクのノード
```

## 自動検出されるファイル

`rosidl_auto_generate_interfaces()` は以下のパターンを自動検出：

- `msg/*.msg` - メッセージ定義
- `msg/*.idl` - IDL形式のメッセージ
- `srv/*.srv` - サービス定義
- `srv/*.idl` - IDL形式のサービス
- `action/*.action` - アクション定義
- `action/*.idl` - IDL形式のアクション

## 参考リンク

- [rosidl_cmake](https://github.com/ros2/rosidl) - ROS 2 インターフェース生成ツール
- [ament_cmake_auto](https://github.com/ament/ament_cmake) - CMake自動化マクロ
