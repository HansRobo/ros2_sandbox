# clock_injection_experiments

ロックステップなシミュレーションで時刻を制御するための **2つの操作** ——
`/clock` トピックの **遮断** と、rcl API による時刻の **直接注入** ——
が表裏一体で成立することを gtest で検証する実験パッケージ。

## 背景

ロックステップなシミュレーションでは、`/clock` トピック経由で配信される時刻に遅延が乗る。
`rclcpp::Clock` の裏にある `rcl_clock_t` を直接操作すれば、トピックを介さず遅延ゼロで時刻を反映できる。
本パッケージはその実現可能性を確認する。

## なぜ「遮断」と「注入」は表裏一体か

ロックステップで時刻を握るには、**外乱を断つ（遮断）** と **能動的に時刻を進める（注入）** の両方が要る。
どちらか片方だけでは成り立たない。

- **注入だけ**では不十分 — Clock が `/clock` に繋がったままだと、後から届いたトピック値が
  注入値を上書きし、競合する。
- **遮断だけ**では不十分 — `/clock` から切り離しても、時刻を進める手段が無ければ Clock は止まったまま。

| | 担うこと | 手段 |
| --- | --- | --- |
| **遮断** | 注入値を上書きする外乱（`/clock`）を断つ | `TimeSource` に attach しない / `detachClock()` |
| **注入** | 時刻を遅延ゼロで能動的に進める | `rcl_enable_ros_time_override` + `rcl_set_ros_time_override` |

両者をつなぐ鍵は、**override は一度有効化されると `detachClock()` では無効化されない**という挙動。
これにより「`/clock` から引き剥がして遮断 → そのまま rcl API で注入し続ける」が破綻なく両立する。

## 仕組み

### (a) 時刻注入

`rclcpp::Clock(RCL_ROS_TIME)` から `get_clock_handle()` で `rcl_clock_t*` を取り出し、
rcl の公開 API で時刻を注入する。

```cpp
auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
rcl_clock_t * handle = clock->get_clock_handle();

rcl_enable_ros_time_override(handle);          // override を有効化（必須）
rcl_set_ros_time_override(handle, time_ns);    // 時刻を直接注入

clock->now();  // 注入値がそのまま返る（遅延ゼロ）
```

### (b) `/clock` 遮断

注入値が `/clock` に上書きされないよう、Clock を `/clock` の購読から切り離す。
そもそも `TimeSource` に attach しなければ `/clock` の影響は受けない。
すでに attach 済みなら `detachClock()` で引き剥がす（override は無効化されない）。

```cpp
// use_clock_thread=false で内部スレッドを無効化し、自前 executor の spin に直列化することで
// 直接注入とのスレッド競合を避ける。
rclcpp::TimeSource time_source(node, rclcpp::ClockQoS(), /*use_clock_thread=*/false);
time_source.attachClock(ros_clock);
// ...
time_source.detachClock(ros_clock);  // /clock から遮断。ros_time_is_active() は true のまま
```

## テスト

| テスト | 役割 | 内容 |
| --- | --- | --- |
| `test_direct_clock_injection` | 注入の検証 | rcl API による直接注入の基本動作。`now()` が注入値と完全一致（遅延ゼロ）で返ること、複数回注入・巻き戻しが都度反映されること、override 有効化が必須であること、`RCL_ROS_TIME` 以外は注入不可なことを確認。 |
| `test_clock_isolation` | 遮断＋注入の両立検証 | `TimeSource` に attach した Clock を `detachClock()` で `/clock` から引き剥がす。`/clock` 経由更新が効くことを確認後、detach（遮断）→ 直接注入し、その後 `/clock` を別の値で発行しても `now()` が注入値のまま影響を受けないことを確認。 |

## 確認できた知見

- `detachClock()` は override を無効化しない。detach 後も `ros_time_is_active()` は `true` のままで、直接注入が即有効。これが「遮断」と「注入」を両立させる根拠。
- `TimeSource` に attach していない `RCL_ROS_TIME` Clock は `/clock` の影響を受けず、注入だけで動く（ロックステップ用途の本命挙動）。遮断は「attach しない」だけでも達成できる。
- `TimeSource` は `use_clock_thread=false` にして自前 executor の spin に直列化すると、直接注入とのスレッド競合を避けられる。

## ビルドとテスト

```bash
colcon build --packages-select clock_injection_experiments
colcon test --packages-select clock_injection_experiments
colcon test-result --all --verbose
```
