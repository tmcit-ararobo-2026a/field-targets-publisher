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
colcon build --symlink-install --packages-select field_target_pub
source install/setup.bash
```

### 実行

```bash
ros2 run field_target_pub field_target_node
```

### パブリッシュされるトピック一覧

※オブジェクトが視野内に存在しない場合、すべてのトピックに `99.0` が送信される。

| トピック名 | 型 | 説明 |
| :--- | :--- | :--- |
| `bucket_target` | `std_msgs/msg/Float32` | バケツ（`bucket_1`, `2`, `3`）の相対角度（rad） |
| `move_bucket_target` | `std_msgs/msg/Float32` | 相手ロボット（`opponent_robot`）の相対角度（rad） |
| `flag_target` | `std_msgs/msg/Float32` | 旗（`flag_base`）の相対角度（rad） |
| `desk_target` | `std_msgs/msg/Float32` | 机（`desk_1`〜`4`）の相対角度（rad） |

---

## 4. システム構成

- **自動エリア判定**: ロボットのY座標（`map` から見た `base_link`）を監視し、自陣が領域Aか領域Bかを自動で切り替える。
- **Static TF の配信**: フィールド上の各種目標物（旗、机、バケツなど）の座標を `map` フレーム基準で静的に配信する。
- **視野角フィルタリング**: 自機から見た各ターゲットの方位角度を計算し、前方40度以内にあるターゲットのみを抽出する。
- **トピック配信**: カテゴリごとに分類し、視界にない場合はデフォルト値として `99.0` をパブリッシュする。

---

## 5. ライセンス

本リポジトリは [MITライセンス](./LICENSE) のもとで公開されている。

## STM32新仕様向け型生成（T01・T02b）

ROSパッケージ名は `field_target_pub`。リポジトリ名と既存 `field-target-pub/` ディレクトリは維持する。
`-DBUILD_RUNTIME_NODES=OFF` で型生成を実行ノードから分離できる。
新しい型付き対象配信・承認済み停止PoseはT09で追加する。既存の座標・旧角度出力は
承認済み停止位置として扱わず、managerの制御入力へ接続しない。

## FieldTarget・TargetSet（T02b）

FieldTargetは照準点・停止Poseを分離し、各validと角度validを持つ。
TargetSetは明示チーム・地図ID・revision・全体validを持つ。未承認値はvalidとして扱わない。
T02bは型定義までで、新 `/field/targets` 配信・承認設定の導入はT09の残作業。
旧ノードの自機Yによるチーム推測や99radは新型の有効値へ変換しない。
