# 固定対象設定の検証（T09）

実施日: 2026-10-14（Asia/Tokyo）。作業ディレクトリ: `/home/lambda/ros2_ws/src`。
対象: `feat/1-field-target-interfaces`、基準 `500dd69a39713f4aff4a9d5c20999d6da8a4504f`からT01・T02b・T09の差分。

`source /opt/ros/humble/setup.bash` 後、colcon build --base-paths field-targets-publisher --build-base /tmp/stm32-ros2/build --install-base /tmp/stm32-ros2/install --packages-select field_target_pub。
ビルド: 成功／終了コード0。source /tmp/stm32-ros2/install/setup.bash 後、ctest --test-dir /tmp/stm32-ros2/build/field_target_pub --output-on-failure。

試験: 成功／終了コード0。合成赤青プロファイルの全固定ID、照準点と停止Poseの分離、Yawによるbearing変化、TF断、未承認・地図不一致・重複／欠落ID・不正vectorを検証。
ROS公式schema検査とPython launch構文: 成功。旧ROSパッケージ名の実行・manifest・CMake参照なし。
静的TFは対象中心と追加固定物から生成。real座標・チーム・地図・角度の契約は未確定。

ログ: /tmp/stm32-ros2/field-t09-static-build.log、field-t09-static-final-tests.log。
試験はlocalhost専用domain・合成geometryのみ。GPUでの固定物除外・実機干渉・停止位置・精度・HIL: 未実施。
初回T01 schema取得はsandboxでROS公式XSDを取得できず失敗。ネットワーク許可下で再実行して成功。
既定テンプレートはapproved=false・座標なし。STM32角度の符号・丸め・無効値は未確定なのでangle_valid=false。
