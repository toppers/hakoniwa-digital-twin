# hakoniwa-digital-twin

本リポジトリでは、バーチャル・ドローンとリアル・ロボット(TB3)が、箱庭を通して相互運用するデジタルツインのデモ環境を提供します。

![image](images/real-virtual-fusion.png)

## デモ

### 登場物

* バーチャル側
  * バーチャル・ドローン
    * バーチャルな世界にだけ存在するドローンです
    * 荷物を運搬し、バーチャルロボットに渡します
  * 荷物
    * バーチャル世界にだけ存在する荷物です
  * バーチャル・ロボット(Twin)
    * リアル世界のロボットのTwinです
    * バーチャル世界のイベントをセンサデータとしてリアル世界のロボットに通知します
  * バーチャル信号
    * バーチャル世界の信号です
    * 赤、黄、青の状態を持ちます
* リアル側
  * インフラセンサ(LiDAR)
    * リアルロボットの位置推定を行います
  * リアル・ロボット(TB3)
    * インフラセンサの情報とバーチャル側のセンサ情報を入力として荷物運搬の制御を行います
* [箱庭ブリッジ](https://github.com/toppers/hakoniwa-bridge/tree/main)
  * ShmProxy
    * 箱庭PDUデータをリアルとバーチャルとで共有します
  * RosProxy
    * ROSメッセージをリアルとバーチャルとで共有します

### デモ内容

* バーチャル・ドローンのシナリオ
  * バーチャル・ドローンは、バーチャル世界の荷物集積場に移動します
  * バーチャル・ドローンは、荷物集積場で荷物を受け取ります
  * バーチャル・ドローンは、リアルロボットの位置を受信し、目標移動ポイントを設定します
  * バーチャル・ドローンは、リアルロボットの位置まで移動します
  * バーチャル・ドローンは、目標位置に到着したら高度をリアルロボット上の50cmあたりで止まり、荷物を下ろします
  * 荷物を下ろした後、バーチャル・ドローンは、荷物集積場に戻ります
* バーチャル・ロボットのシナリオ
  * バーチャル・ロボットは、リアル・ロボットの位置を受信し、バーチャル空間上の同じ位置に移動します
  * バーチャル・ロボットは、荷物が置かれたことを検出するためのタッチセンサを装備しており、荷物の配置有無の情報を箱庭PDUデータで共有します
* バーチャル信号のシナリオ
  * バーチャル信号は、決められたルールに従って、信号の状態を変更します
    * ルール1. 赤 -> 黄 -> 青 の順番で遷移します
    * ルール2. 赤の時間：10秒
    * ルール3. 黄の時間：5秒
    * ルール4. 青の時間：15秒
    * ルール5. 青の時間が10秒経過したら、残り5秒間は点滅状態とする
* リアル・ロボットのシナリオ
  * リアル・ロボットは、荷物を受け取る場所で待機状態として、停止しています
  * リアル・ロボットは、荷物が置かれたことを検出した場合、前進します
  * バーチャル・ロボットは、バーチャル信号のデータを検出し、その値に応じた回避行動を行います
    * 赤信号：停止します
    * 黄信号：停止します
    * 青信号：前進します
  * リアル・ロボットは、前進中にバーチャル・ロボットから自己位置を取得し、目標位置に到達したことを検出すると停止します
  * リアル・ロボットは、目標地点に到着したことをバーチャル側に通知します
  * バーチャル側でその通知を受け取ると、バーチャル側の操作で荷物を下ろします
  * リアル・ロボットは、荷物が下ろされたことを検出すると、元の位置に後進し、待機状態に戻ります
* インフラセンサのシナリオ
  * インフラセンサは、リアルロボットの位置を推定し、推定結果をリアルとバーチャル側に配信します

## アーキテクチャ

![architecture](images/digital-twin-demo-arch.png)

なお、アーキテクチャ上、リアル側は仮想テスト向けに差し替えることが可能です。

## 動作環境

* バーチャル側
  * MacPC
    * MacBook Pro(2)
    * チップ：Apple M2 Pro
    * Memory: 32GB
    * ソフトウェア
      * Unity Editor
      * 箱庭ドローンシミュレータ
      * ShmProxy
* リアル側（実機向け）
  * Linux PC
    * Ubuntu 22.04.4 LTS
    * CPU: Intel(R) Core(TM) i7-10610U CPU @ 1.80GHz
    * Memory: 16GB
    * ソフトウェア：
      * RosProxy: Ubuntu(ROS2)
      * infra_sensor_urg: ロボットの位置推定プログラム
      * tb3_controller: ロボット制御プログラム
      * urg_node2：URGセンサドライバ
  * LiDAR
    * urg_node2: HOKUYO URG-04LX-UG01
  * リアルロボット
    * TurtleBot3
    * Raspberry Pi3(ROS2)
      * TB3パッチ適用が必要
* AR側
  * QUEST3
    * QUEST3向け箱庭ARアプリ
* Wi-Fiルーター
* 有線LAN（３本）
* PS4コントローラおよび接続USBケーブル
* デモフィールド：ダンボールで構築(1m x 2.5m 0.3m)

なお、リアル側をシミュレータとしたい場合は、Ubuntu側の構成は以下の通りとなります。

* リアル側（仮想テスト向け）
  * RosProxy: Ubuntu(ROS2)
  * infra_sensor_urg: ロボットの位置推定プログラム
  * tb3_controller: ロボット制御プログラム
  * virtual urg sensor: Unityで実現
  * Virtual TB3 Robot: Unityで実現

## 構成

## 物理構成

![image](images/physical-diagram.png)

## ソフトウェア構成

![image](images/software-diagram.png)

## インストール手順

### コンフィグファイルの作成

箱庭のコンフィグファイルである custom.jsonは、リアルとバーチャルとで共有しますので、それぞれにコピー配置してください。
共有対象モジュールは、ShmProxy と HakoRosProxy です。

custom.json は、以下の２パターンあります。

*  リアル側（実機向け）
*  リアル側（仮想テスト向け）

custom.jsonは、箱庭ドローンシミュレータのUnityエディタ上で、`Generate`を実行することで作成できますが、以下の理由から、そのまま使用することはできません。

1. リアル側に配信するデータとしては、必要最小限のもので良いため、不要なものは削除する必要がある
2. リアル側をシミュレータとする場合は、リアル側のTB3のROSトピック情報を追加する必要がある

### リアル側

リアル側では、以下の対応が必要となります。

* [Raspberry Pi on TB3 にパッチ適用する](real/robot/tb3/README.md)
* [Ubuntu PC に URG センサドライバをインストール](real/sensors/drivers/Hokuyo/urg/README.md)
* [Ubuntu PC に RosProxyをインストール](#UbuntuPCにRosProxyをインストール)
* [Ubuntu PC にインフラセンサモジュールをインストール](#UbuntuPCにインフラセンサモジュールをインストール)
* [Ubuntu PC にロボット制御プログラムをインストール](#UbuntuPCにロボット制御プログラムをインストール)

なお、リアル側のテスト用のUnityアプリは以下にあります。

* https://github.com/toppers/hakoniwa-digital-twin/releases/edit/digital-twin-real-model

hakoniwa-unity-drone-model 直下で、`TwinReal.zip` を解凍してください。


#### UbuntuPCにRosProxyをインストール

リポジトリのクローン：
```
git clone --recursive https://github.com/toppers/hakoniwa-digital-twin.git
```

ディレクトリの移動：
```
cd hakoniwa-digital-twin/bridge/third-party/hakoniwa-ros2pdu
```

RosProxyのインストール：custom.jsonは、digital/config 配下のものを利用してください。

https://github.com/toppers/hakoniwa-bridge?tab=readme-ov-file#installation-instructions

#### UbuntuPCにインフラセンサモジュールをインストール

リポジトリのクローン：
```
git clone --recursive https://github.com/toppers/hakoniwa-digital-twin.git
```

ディレクトリの移動：
```
cd hakoniwa-digital-twin/real/sensors/workspace
```

ビルド：
```
colcon build --packages-select infra_sensor_urg
```

成功するとこうなります。
```
Starting >>> infra_sensor_urg
Finished <<< infra_sensor_urg [4.47s]   
```

# UbuntuPCにロボット制御プログラムをインストール

リポジトリのクローン：
```
git clone --recursive https://github.com/toppers/hakoniwa-digital-twin.git
```

ディレクトリの移動：
```
cd hakoniwa-digital-twin/real/robot/workspace/
```

ビルド：
```
colcon build --packages-select tb3_controller
```

成功するとこうなります。
```
Starting >>> tb3_controller
Finished <<< tb3_controller [4.47s]   
```

### バーチャル側

バーチャル側では、以下の対応が必要となります。

* [箱庭ドローンシミュレータのインストール](https://github.com/toppers/hakoniwa-px4sim)
* [ShmProxyのインストール](#ShmProxyのインストール)


#### ShmProxyのインストール

リポジトリのクローン：
```
git clone --recursive https://github.com/toppers/hakoniwa-digital-twin.git
```

ディレクトリの移動：
```
cd hakoniwa-digital-twin/bridge/third-party/hakoniwa-ros2pdu
```

ShmProxyのインストール：custom.jsonは、digital/config 配下のものを利用してください。

https://github.com/toppers/hakoniwa-bridge?tab=readme-ov-file#installation-instructions


## 箱庭ARアプリ側

リポジトリのクローン：
```
git clone --recursive https://github.com/toppers/hakoniwa-unity-drone-model.git
```

### ARアプリの作成

#### Hakoniwa Scene の設定

Hierarchyビューの `Assets/Scenes/DigitalTwin/Hakoniwa` を選択します。

#### ARアプリのビルド

箱庭ARアプリの[ビルド手順](https://github.com/toppers/hakoniwa-unity-drone-model/blob/main/README-quest3.md)を参照ください。

なお、ビルド済みのもを以下で公開しています（最新版をご利用ください）。

https://github.com/toppers/hakoniwa-unity-drone-model/releases

対象ファイル：model.apk

#### ARアプリのインストール

箱庭ARアプリの[インストール手順](https://github.com/toppers/hakoniwa-unity-drone-model/blob/main/README-quest3.md#quest3%E5%90%91%E3%81%91%E3%82%A2%E3%83%97%E3%83%AA%E3%81%AE%E3%83%93%E3%83%AB%E3%83%89)を参照ください。

Unityシーンは、`Scenes/DigitalTwin/Quest3` を使います。

#### ARアプリの位置調整

以下の解説資料を参照ください。

- https://www.docswell.com/s/kanetugu2015/K4QXM1-hakoniwa-unity-ar
- https://github.com/toppers/hakoniwa-unity-drone-model/blob/main/README-quest3-drone.md
- https://www.docswell.com/s/kanetugu2015/K4VVJD-hakoniwa-drone-ar-op


## 実行手順（自動ツール利用編）

デモ向けに簡単にオペレーション実行したい場合は、以下の手順で実施できます。

### TB3

TB3の電源を起動し、TB3のトピックが出力されるのを待ちます。

```
ros2 topic list
```

`tb3_cmd_vel` のトピックが見えたら成功です。

TB3の頭に被せる帽子を外します。

### Linuxマシン

#### Linux 端末A

URGとインフラセンサを起動します。

```bash
hakoniwa-digital-twin/real/sensors
```

```
bash run_nodes.bash
```

センサが環境認識完了した、TB3の頭に帽子を被せて、`s` ボタンを押下します。

#### Linux 端末B

ROSProxyを起動します。

```
cd hakoniwa-digital-twin/bridge/third-party/hakoniwa-ros2pdu/workspace
```

```
ros2 run hako_ros_proxy hako_ros_proxy_node
```

#### Linux 端末C

TB3ロボット制御プログラムを起動します。

```
cd hakoniwa-digital-twin/real/robot
```

```
bash run.bash
```

### QUEST3

箱庭ARアプリを起動します。

箱庭ARブリッジがあるので、独立して起動停止可能です。

### MacPC

```
cd hakoniwa-digital-twin
```

```
bash run.bash
```

位置調整モードになるので、PS4コントローラでドローンの初期位置を決めます。

初期位置が決まったら、○ボタンを押下してください。

１秒後くらいで、×ボタンを押すと、PS4コントローラでドローンを操作できます。

ドローンを５０cmほど浮上させます。


### PS4コントローラ

荷物をロボットの上に置いてください

操縦している際に、位置ずれが発生する場合は、PS4コントローラの箱庭の未サポートボタンを押下して、位置調整からやり直しできます。



## 実行手順（個別編）

個々の機能を手作業で実行する場合は、以下の手順で実施できます。

0. ARアプリを起動する
1. 箱庭ドローンシミュレータを起動する
2. [ShmProxyを起動する](https://github.com/toppers/hakoniwa-bridge?tab=readme-ov-file#shmproxy)
3. [RosProxyを起動する](https://github.com/toppers/hakoniwa-bridge?tab=readme-ov-file#rosproxy)
4. [TB3のROSノードを起動する](#TB3のROSノードを起動する)
5. [URGセンサを起動する](real/sensors/drivers/Hokuyo/urg/README.md#ros2ノードを起動する)
5. [Infra Sensorを起動する](#InfraSensorを起動する)
6. [ロボット制御プログラムを起動する](#ロボット制御プログラムを起動する)
7. [箱庭Webサーバーを起動する](#箱庭Webサーバーを起動する)
7. [箱庭ARブリッジを起動する](#箱庭ARブリッジを起動する)

### TB3のROSノードを起動する

ディレクトリ移動：
```
cd turtlebot3_ws
```

ROSノード起動：
```
source install/setup.bash 
```

```
ros2 launch turtlebot3_bringup robot.launch.py
```

### InfraSensorを起動する

```
cd hakoniwa-digital-twin/real/sensors/workspace
```

```
source install/setup.bash 
```

```
ros2 run infra_sensor_urg lidar_subscriber --ros-args -p act_mode:=real
```

※テスト向けの場合は、act_mode:=simを指定してください。


成功するとこうなります。
```
act_mode:  real
[INFO] [1720398553.685691839] [lidar_subscriber]: InfraSensor UP
Now scanning environments..., please wait.
```

成功すると以下の画面が出力されます。

![image](images/infra-sensor-start.png)

環境データを認識するために３０秒程度お待ちください。成功すると以下のように環境を認識し、セグメント化したデータ表示されます。

![image](images/infra-sensor-env.png)

この状態で、ロボットを配置してください。

配置完了したら、以下の端末上で、`s` を入力してエンターキーを押下しましょう。

```
Enter command (s: scan mode, p: processing mode): Environment scan completed and data averaged.
```

成功すると、ロボットのセグメントが表示されます。

![image](images/infra-sensor-trace.png)


ロボットの位置推定結果は、`RobotAvator_cmd_pos`で配信されます。以下の要領で確認できます。

```
ros2 topic echo /TB3RoboAvatar_cmd_pos
```

```
linear:
  x: 0.2749041557341515
  y: 0.02060769323128149
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

### ロボット制御プログラムを起動する

```
cd hakoniwa-digital-twin/real/robot/workspace
```

```
ros2 run tb3_controller tb3_controller_node --ros-args -p act_mode:=real
```

※テスト向けの場合は、act_mode:=simを指定してください。

成功すると、こうなります。

```
[INFO] [1720399525.556355826] [tb3_controller_node]: START: tb3_controller_node: real
```

### 箱庭Webサーバーを起動する

```bash
 python3.12 server/main.py --asset_name WebServer --config_path config/twin-custom.json --delta_time_usec 20000
```

### 箱庭ARブリッジを起動する

```bash
python3.12 asset_lib/main.py --config asset_lib/config/ar_bridge_config.json
```

## プログラム構成

本リポジトリのプログラム構成は以下の通り。

* real
  * リアル側のソースコード一式を管理
* digital
  * バーチャル側のソースコード一式を管理
* bridge
  * 箱庭ブリッジ（サブモジュールとして管理）
