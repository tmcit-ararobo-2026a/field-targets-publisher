# field_target_pub

2026NHKロボコンAチームのために作成された、フィールド上の固定対象の照準点・停止位置・中心座標を配信するROS 2パッケージです。

## 目次

1. [概要](#1-概要)
2. [コントリビューション](#2-コントリビューション)
3. [ビルド・使い方](#3-ビルド使い方)
4. [システム構成](#4-システム構成)
5. [ライセンス](#5-ライセンス)

---

## 1. 概要

`field_target_node`は設定ファイルで指定したチーム・地図の対象情報を`TargetSet`として配信します。照準点、停止Pose、物体中心を分けて管理し、自機のTFから各対象への方位角を計算します。

---

## 2. コントリビューション

[CONTRIBUTING.md](CONTRIBUTING.md)を参照してください。

---

## 3. ビルド・使い方

### 前提条件

ROS 2 Humble、`rclcpp`、`tf2_ros`、`geometry_msgs`、`std_msgs`、`rosidl_default_generators`、`nlohmann_json`を使用します。

### クローンとビルド

```bash
cd ~/ros2_ws/src
git clone https://github.com/tmcit-ararobo-2026a/field-targets-publisher.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/field-targets-publisher --ignore-src -y --rosdistro humble
colcon build --symlink-install --packages-select field_target_pub
source install/setup.bash
```

ROSパッケージ名は`field_target_pub`です。リポジトリ内のパッケージは`field-target-pub/`にあります。

### 実行

```bash
ros2 launch field_target_pub field-target-pub.launch.py config_file:=/absolute/path/targets.json
```

既定の[config/targets.json](field-target-pub/config/targets.json)は未承認・座標なしのテンプレートです。ファイルをコピーし、実測値と識別情報を設定してください。設定は起動時に読み込み、変更時はrevisionを更新して再起動します。bag再生には`use_sim_time:=true`を指定できます。

型のみをビルドする場合は`--cmake-args -DBUILD_RUNTIME_NODES=OFF`を指定します。

### パブリッシュされるトピック

| トピック | 型 | 内容 |
| :--- | :--- | :--- |
| `/field/targets` | `field_target_pub/msg/TargetSet` | チーム・地図・revisionと各対象の座標・有効性 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 承認済みの固定物中心TF |

---

## 4. システム構成

### 対象の設定

承認済み設定には次の項目が必要です。

| 項目 | 内容 |
| :--- | :--- |
| `schema_version` / `approved` | `1` / `true` |
| `team` | `red`または`blue`を明示 |
| `map_id` / `map_frame` | 使用する地図の識別子・座標系 |
| `revision` | 0より大きい設定の版番号 |
| `targets` | 固定対象ID 0〜6を重複・欠落なく指定 |

各targetには`label`と`stop_pose=[x,y,z,yaw]`を指定します。停止Poseは2Dで`z=0`、yawはradです。ID 1〜6には`aim_point=[x,y,z]`と`object_center=[x,y,z]`も必要です。ID 0のStartZoneは停止Poseだけを持ちます。位置の単位はmです。

非有限値、重複・欠落ID、不正な設定を拒否します。未承認設定では有効な対象や静的TFを配信しません。`expected_map_id`パラメータで地図IDの一致を検査できます。利用側でもチーム・地図・revisionを照合してください。

### 固定TFと方位角

承認済みの物体中心から、`bucket_`、`desk_`、`flag_`で始まるcenterフレームを生成します。対象IDを持たない固定物は`static_objects=[{frame_id,object_center:[x,y,z]},...]`で追加できます。frame_idはこれらの接頭辞を持ち、一意である必要があります。既知物体のTFはLiDAR認識側の除外に利用できます。

`map → base_link` TFから数学的な`bearing_rad`を計算し、TF断・期限切れ・時計停止では`bearing_valid=false`にします。通信向け角度の符号・丸め・無効値は未確定のため、`angle_valid`はfalseです。

固定TFを変更する場合は利用ノードも再起動し、古い静的情報を別の地図へ持ち越さないでください。実機の停止位置・照準点・中心座標は実測による確認が必要です。

---

## 5. ライセンス

本リポジトリは[MITライセンス](LICENSE)のもとで公開されています。
