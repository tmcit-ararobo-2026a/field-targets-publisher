# base-ros2

2026NHKロボコンAチームのために作成された、フィールド上のターゲット位置および自機位置を管理・パブリッシュするROS 2パッケージ。

## 目次

1. [概要](#1-概要)
2. [コントリビューション](#2-コントリビューション)
3. [ビルド・使い方](#3-ビルド使い方)
4. [システム構成](#4-システム構成)
5. [ライセンス](#5-ライセンス)

---

## 1. 概要

本ノード（`field_target_node`）は、ロボットの現在位置（領域A / 領域Bの自動判定）を基に、自機の視野角40度（前方 $\pm20^\circ$ 以内）に入るフィールド上のオブジェクトを検出し、それぞれの相対角度を計算して ROS 2 トピックとしてパブリッシュする。

---

## 2. コントリビューション

[CONTRIBUTING.md](./CONTRIBUTING.md) を参照。

---

## 3. ビルド・使い方

### 前提条件

- ROS 2 (Humble / Iron / Jazzy 等)
- `rclcpp`, `tf2_ros`, `tf2_geometry_msgs`, `geometry_msgs`, `std_msgs`

### クローン

```bash
cd ~/your_ws/src
git clone https://github.com/tmcit-ararobo-2026a/field-targets-publisher.git
```

### ビルド

```bash
cd ~/your_ws
colcon build --symlink-install --packages-select field-target-pub
source install/setup.bash
```

### 実行

```bash
ros2 run field-target-pub field_target_node
```

### パブリッシュされるトピック一覧

※オブジェクトが視野内に存在しない場合、Float32系トピックには `99.0` が送信される。`target_angles` では範囲外・未検出の要素に `INT8_MIN` (-128) が設定される。

| トピック名 | 型 | 説明 |
| :--- | :--- | :--- |
| `/robot/operation/target_angles` | `std_msgs/msg/Int8MultiArray` | 水平角6要素（移動バケツ、バケツ1〜3、旗、机。-20〜20degを-127〜127に対応、範囲外はINT8_MIN） |
| `/robot/operation/fixed_buckets_angle` | `std_msgs/msg/Float32MultiArray` | バケツ（`bucket_1`, `2`, `3`）の相対角度（rad） |
| `robot/operation/move_bucket_angle` | `std_msgs/msg/Float32` | 相手ロボット（`opponent_robot`）の相対角度（rad） |
| `/robot/operation/flag_angle` | `std_msgs/msg/Float32` | 旗（`flag_base`）の相対角度（rad） |
| `robot/operation/desk_angle` | `std_msgs/msg/Float32` | 机（`desk_1`〜`4`）の相対角度（rad） |

---

## 4. システム構成

- **自動エリア判定**: ロボットのY座標（`map` から見た `base_link`）を監視し、自陣が領域Aか領域Bかを自動で切り替える。
- **Static TF の配信**: フィールド上の各種目標物（旗、机、バケツなど）の座標を `map` フレーム基準で静的に配信する。
- **視野角フィルタリング**: 自機から見た各ターゲットの方位角度を計算し、前方40度以内にあるターゲットのみを抽出する。
- **トピック配信**: カテゴリごとに分類し、視界にない場合はデフォルト値として `99.0` をパブリッシュする。

---

## 5. ライセンス

本リポジトリは [MITライセンス](./LICENSE) のもとで公開されている。
