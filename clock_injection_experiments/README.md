# clock_injection_experiments

`/clock` トピックを経由せず、rcl API で `rclcpp::Clock` に時刻を**直接注入**できることを検証する実験パッケージ。

## 背景

ロックステップなシミュレーションでは、`/clock` トピック経由で配信される時刻に遅延が乗る。
`rclcpp::Clock` の裏にある `rcl_clock_t` を直接操作すれば、トピックを介さず遅延ゼロで時刻を反映できる。
本パッケージはその実現可能性を gtest で確認する。

## 仕組み

`rclcpp::Clock(RCL_ROS_TIME)` から `get_clock_handle()` で `rcl_clock_t*` を取り出し、
rcl の公開 API で時刻を注入する。

```cpp
auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
rcl_clock_t * handle = clock->get_clock_handle();

rcl_enable_ros_time_override(handle);          // override を有効化（必須）
rcl_set_ros_time_override(handle, time_ns);    // 時刻を直接注入

clock->now();  // 注入値がそのまま返る（遅延ゼロ）
```

## テスト

| テスト | 内容 |
| --- | --- |
| `test_direct_clock_injection` | rcl API による直接注入の基本動作。`now()` が注入値と完全一致（遅延ゼロ）で返ること、複数回注入・巻き戻しが都度反映されること、override 有効化が必須であること、`RCL_ROS_TIME` 以外は注入不可なことを確認。 |
| `test_clock_isolation` | `TimeSource` に attach した Clock を `detachClock()` で `/clock` から引き剥がす。`/clock` 経由更新が効くことを確認後、detach → 直接注入し、その後 `/clock` を別の値で発行しても `now()` が注入値のまま影響を受けないことを確認。 |

## 確認できた知見

- `detachClock()` は override を無効化しない。detach 後も `ros_time_is_active()` は `true` のままで、直接注入が即有効。
- `TimeSource` に attach していない `RCL_ROS_TIME` Clock は `/clock` の影響を受けず、注入だけで動く（ロックステップ用途の本命挙動）。
- `TimeSource` は `use_clock_thread=false` にして自前 executor の spin に直列化すると、直接注入とのスレッド競合を避けられる。

## ビルドとテスト

```bash
colcon build --packages-select clock_injection_experiments
colcon test --packages-select clock_injection_experiments
colcon test-result --all --verbose
```
