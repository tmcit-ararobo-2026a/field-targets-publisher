# field_target_pub

2026NHKロボコンAチームのために作成された、フィールド上のターゲット位置および自機位置を管理・パブリッシュするROS 2パッケージ。

## 目次

1. [概要](#1-概要)
2. [コントリビューション](#2-コントリビューション)
3. [ビルド・使い方](#3-ビルド使い方)
4. [システム構成](#4-システム構成)
5. [ライセンス](#5-ライセンス)

---

## 1. 概要

本ノード（`field_target_node`）は、明示した承認チーム・地図プロファイルから固定照準点・停止Pose・中心TFを配信する。自機位置からチームを推測せず、STM32の角度変換が未確定の間はwire角度を有効にしない。

---

## 2. コントリビューション

[CONTRIBUTING.md](./CONTRIBUTING.md) を参照。

---

## 3. ビルド・使い方

### 前提条件

- ROS 2 Humble（本変更の検証対象）
- `rclcpp`, `tf2_ros`, `geometry_msgs`, `std_msgs`, `rosidl_default_generators`, `nlohmann_json`

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

| トピック名 | 型 | 説明 |
| :--- | :--- | :--- |
| `/field/targets` | `field_target_pub/msg/TargetSet` | チーム・地図・revision・照準点・停止Pose・有効性 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 承認された固定物中心（bucket_/desk_/flag_） |

## 4. システム構成

- チームはred/blueを明示し、全固定ID 0..6を設定で解決する。
- 中心TF、照準点、停止Poseを分離する。デフォルト設定は未承認で座標を持たない。
- 最新のbase_link TFから数学的bearingを計算し、TF断・期限切れではbearing_valid=false。
- 旧角度Topicと99radは配信しない。STM32用angle_validは未確定契約のためfalse。

---

## 5. ライセンス

本リポジトリは [MITライセンス](./LICENSE) のもとで公開されている。

## STM32新仕様向け型生成（T01・T02b）

ROSパッケージ名は `field_target_pub`。リポジトリ名と既存 `field-target-pub/` ディレクトリは維持する。
`-DBUILD_RUNTIME_NODES=OFF` で型生成を実行ノードから分離できる。
型付き対象配信・停止Pose設定をT09で実装した。既存ハードコード座標を承認済み停止位置として流用しない。

## FieldTarget・TargetSet（T02b）

FieldTargetは照準点・停止Poseを分離し、各validと角度validを持つ。
TargetSetは明示チーム・地図ID・revision・全体validを持つ。未承認値はvalidとして扱わない。
型定義（T02b）に加えT09の配信・承認設定を実装済み。実機プロファイルの確定は残っている。
旧ノードの自機Yによるチーム推測や99radは新型の有効値へ変換しない。

## 明示チーム・照準点・停止位置（T09）

旧Y座標によるチーム推測、ハードコード対象、旧角度Topic・99radを廃止し、`/field/targets` を配信する。
`config/targets.json` はapproved=false・座標なしのテンプレート。
承認プロファイルはschema_version=1、approved=true、team=red/blue、map_id、map_frame、revision>0と
全固定ID 0..6のtargetsを持つ。各targetはlabel、stop_pose=[x,y,z,yaw]（2Dのz=0）、
ID1..6はaim_point=[x,y,z]とobject_center=[x,y,z]が必要。
中心・照準点・停止位置を流用しない。StartZoneは停止Poseのみで射出照準点を持たない。
不正・重複・欠落ID・非有限値を拒否し、未承認設定では対象や静的TFを有効配信しない。

`ros2 launch field_target_pub field-target-pub.launch.py config_file:=/approved/profile.json`。
expected_map_idで地図IDの不一致も拒否できる。manager側でも実際のチーム・地図識別を照合する。
設定はread-onlyで、変更の反映には再起動と新revisionを使う。
固定TFは承認されたobject_centerからbucket_/desk_/flag_のcenterフレームを生成し、LPS除外へ接続する。
有効なbase_link TFから数学的bearing_radを求めるが、STM32の角度符号・丸め・無効値は未確定なので
angle_validはfalse。承認停止位置・校正・角度契約・実機での地図一致は未完了。

対象IDを持たない固定物もstatic_objects=[{frame_id,object_center:[x,y,z]},...]で明示できる。
frame_idは一意のbucket_/desk_/flag_接頭辞を持ち、別チーム側や非照準物もLPS除外へ供給する。
設定変更時はTF利用ノードも再起動し、古い静的除外情報を新地図へ流用しない。
TFの進行・受信からの単調期限も検査し、ROS時計停止で古いbearingを延長しない。
