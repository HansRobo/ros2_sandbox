# clock_injection_experiments

ロックステップなシミュレーションで時刻を制御するための **2つの操作** ——
`/clock` トピックの **遮断** と、rcl API による時刻の **直接注入** ——
が表裏一体で成立することを、カスタム TimeSource クラス `InjectionTimeSource` として実装し
gtest で検証する実験パッケージ。

## 背景

ロックステップなシミュレーションでは、`/clock` トピック経由で配信される時刻に遅延が乗る。
`rclcpp::Clock` の裏にある `rcl_clock_t` を直接操作すれば、トピックを介さず遅延ゼロで時刻を反映できる。
本パッケージはこれを `InjectionTimeSource` という再利用可能なクラスにカプセル化する。

## なぜ「遮断」と「注入」は表裏一体か

ロックステップで時刻を握るには、**外乱を断つ（遮断）** と **能動的に時刻を進める（注入）** の両方が要る。
どちらか片方だけでは成り立たない。

- **注入だけ**では不十分 — Clock が `/clock` に繋がったままだと、後から届いたトピック値が
  注入値を上書きし、競合する。
- **遮断だけ**では不十分 — `/clock` から切り離しても、時刻を進める手段が無ければ Clock は止まったまま。

| | 担うこと | 手段 |
| --- | --- | --- |
| **遮断** | 注入値を上書きする外乱（`/clock`）を断つ | `InjectionTimeSource` は `/clock` を **そもそも購読しない** |
| **注入** | 時刻を遅延ゼロで能動的に進める | `attachClock()` で override を有効化 → `injectTime()` |

`InjectionTimeSource` は標準の `rclcpp::TimeSource` と異なり `/clock` を一切購読しない。
そのため「遮断」は購読しないこと自体で構造的に達成され、`attachClock()` した Clock は
`injectTime()` による注入だけで駆動される。

## カスタム TimeSource クラス: `InjectionTimeSource`

`include/clock_injection_experiments/injection_time_source.hpp` / `src/injection_time_source.cpp`

`rclcpp::TimeSource` の attach/detach インターフェースに倣いつつ、`/clock` 購読の代わりに
`injectTime()` で時刻を直接注入する。

```cpp
#include "clock_injection_experiments/injection_time_source.hpp"

auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

clock_injection_experiments::InjectionTimeSource time_source;
time_source.attachClock(clock);      // override を有効化（RCL_ROS_TIME 以外は例外）

time_source.injectTime(time_ns);     // 時刻を遅延ゼロで直接注入
clock->now();                        // 注入値がそのまま返る
```

### API

| メソッド | 役割 |
| --- | --- |
| `attachNode(node)` | `node->get_clock()`（`node->now()` が使う Clock）を attach する便利メソッド。内部で `attachClock()` へ委譲。 |
| `attachClock(clock)` | `RCL_ROS_TIME` の Clock を管理対象に追加し override を有効化。`RCL_ROS_TIME` 以外は `std::invalid_argument`。既に注入済みなら最後の注入値を即適用。 |
| `detachClock(clock)` | 管理対象から外す。override は **無効化しない**（`rclcpp::TimeSource` と同じ挙動）。 |
| `injectTime(ns)` / `injectTime(rclcpp::Time)` | attach 中の全 Clock へ時刻を遅延ゼロで注入。 |

### 設計上のポイント

- `/clock` を購読しないため、attach した Clock は構造的に `/clock` の外乱を受けない。
- override は一度有効化されると `detachClock()` では無効化されない（時刻が止まるだけで巻き戻らない）。
- attach/detach/注入のリスト操作はミューテックスで保護。
- `RCL_ROS_TIME` 以外は注入できないため、`attachClock()` で弾いて落とし穴を不変条件として固定。

### `rclcpp::Node::now()` を注入で駆動する（ロックステップ本番）

ロックステップでは、ノード全体（タイマー・TF 等）を sim time で動かすため **`use_sim_time=true`** で
ノードを生成する。この状態でノード内部の `TimeSource` が `node->get_clock()` の override を
既に有効化しているので、`attachNode(node)` して `injectTime()` すれば、`/clock` を経由せず
遅延ゼロで `node->now()` を駆動できる。

```cpp
rclcpp::NodeOptions options;
options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
// /clock を未使用トピックへリマップして内部 TimeSource の購読を空振りさせる（下記参照）
options.arguments({"my_node", "--ros-args", "-r", "/clock:=/unused_clock_sink"});
auto node = std::make_shared<rclcpp::Node>("my_node", options);

clock_injection_experiments::InjectionTimeSource time_source;
time_source.attachNode(node);     // node->get_clock() を握る

time_source.injectTime(time_ns);
node->now();   // 注入値がそのまま返る
```

#### `/clock` の外乱を遮断する

`use_sim_time=true` のノードでは、ノード内部の `TimeSource` が `/clock` を購読しているため、
本物の `/clock` が発行されると `node->now()` が上書きされてしまう。これを断つ手段は2つある。

**この内部 `TimeSource` は公開 API では取得できない**（`NodeTimeSourceInterface` は空、
保持する `TimeSource` は private、rcl の clock API にも購読を止める手段がない）。
そのため遮断には以下のいずれかを使う。

##### 方法1: `/clock` をリマップする（推奨・堅牢）

`/clock` を未使用トピックへリマップすると、内部 `TimeSource` の購読が空振りになり、本物の
`/clock` が流れても `node->now()` は注入値のまま保たれる。公開機能のみで実装非依存。

```bash
ros2 run <pkg> <node> --ros-args -p use_sim_time:=true -r /clock:=/unused_clock_sink
```

動作するデモは `src/node_clock_injection_example.cpp`
（`ros2 run clock_injection_experiments node_clock_injection_example`。example 側でリマップを
フォールバック設定済み）。

##### 方法2: 内部 `TimeSource` から detach する（上級者向け・実装依存）

起動引数を使わずコード内で完結させたい場合は、`detachNodeInternalTimeSource(node)` で
内部 `TimeSource` からノードの Clock を引き剥がせる。private メンバアクセス手法で
`NodeTimeSource::time_source_` に到達して `detachClock()` を呼ぶ。

```cpp
#include "clock_injection_experiments/node_internal_time_source_detacher.hpp"

clock_injection_experiments::detachNodeInternalTimeSource(node);  // /clock購読から切り離す
time_source.attachNode(node);                                     // InjectionTimeSourceで握り直す
time_source.injectTime(time_ns);                                  // 以降は注入値が保たれる
```

**注意**: rclcpp 内部の `NodeTimeSource` 実装（private メンバ `time_source_`）に依存するため、
ROS バージョン更新で壊れる可能性がある。通常は方法1（リマップ）を推奨する。

## テスト

| テスト | 役割 | 内容 |
| --- | --- | --- |
| `test_direct_clock_injection` | 注入の検証 | `attachClock` で override が有効化されること、`injectTime` した値が `now()` と完全一致（遅延ゼロ）で返ること、複数回注入・巻き戻しが都度反映されること、注入後に attach した別 Clock へ最後の注入値が即反映されること、`RCL_ROS_TIME` 以外は `attachClock` で例外になることを確認。 |
| `test_clock_isolation` | 構造的遮断の検証 | `InjectionTimeSource` に attach した Clock へ `injectTime` した後、背景で `/clock` に別の値を発行し続けても `now()` が注入値のまま不変であること（購読しない設計による構造的遮断）、`detachClock` 後も override が有効なままであることを確認。 |
| `test_node_clock_injection` | Node への注入検証 | `use_sim_time=true` のノードを生成時に override が有効化されること、`attachNode(node)` し `injectTime` した値が `node->now()` に返ること（複数回・巻き戻し含む）、`/clock` を未使用トピックへリマップすると本物の `/clock` を発行しても `node->now()` が注入値のまま遮断されること（方法1）を確認。 |
| `test_node_internal_detach` | 内部 detach の検証 | `detachNodeInternalTimeSource(node)` で内部 `TimeSource` からノードの Clock を引き剥がした後、本物の `/clock` を発行しても `node->now()` が注入値のまま遮断されること（方法2）を確認。 |

## 確認できた知見

- `InjectionTimeSource` は `/clock` を購読しないため、attach した Clock は構造的に外乱を受けず、`injectTime()` だけで駆動できる（ロックステップ用途の本命挙動）。
- `detachClock()` は override を無効化しない。detach 後も `ros_time_is_active()` は `true` のままで、時刻は最後の注入値に保たれる。
- `attachClock()` を `RCL_ROS_TIME` 限定にすることで、注入不可能な Clock を早期に弾ける。
- `use_sim_time=true` のノードの内部 `TimeSource` は公開 API でも rcl API でも取得・detach できない（`NodeTimeSourceInterface` は空、`time_source_` は private、clock ハンドルに override 上書きを拒否する機構はない）。
- それでも `/clock` 外乱を断つには、(1) `/clock` のリマップ（堅牢）か、(2) private メンバアクセスによる内部 `TimeSource` の `detachClock`（実装依存・上級者向け）の2通りが有効。いずれも実機で遮断を確認済み。

## ビルドとテスト

```bash
colcon build --packages-select clock_injection_experiments
colcon test --packages-select clock_injection_experiments
colcon test-result --all --verbose
```
