# インストール手順

TG15のROS2ドライバのインストール手順は以下の通りです。
本手順は、[こちら](https://www.switch-science.com/blogs/magazine/ydlidar-tg15-with-ros2-humble)を参考にしたものです。

- パッチを適用する
- ydlidar_ros2_driver をビルドする
- YDLidar-SDK をビルドする
- LiDARセンサの設定をする

# パッチを適用する

```
cd hakoniwa-digital-twin/real/sensors/drivers/TG15
```

```
bash install.bash
```

# ydlidar_ros2_driver をビルドする

```
source /opt/ros/foxy/setup.bash
```

```
colcon build --packages-select ydlidar_ros2_driver
```

```
source install/setup.bash
```

# YDLidar-SDK をビルドする

事前インストール：
```
sudo apt install cmake pkg-config
sudo apt-get install swig
sudo apt-get install python3-pip
```


```
cd hakoniwa-digital-twin/third-party/YDLIDAR/YDLidar-SDK
```

```
mkdir build
cd build
```

```
cmake ..
make
sudo make install
cd YDLidar-SDK
pip install .
```

# LiDARセンサの設定をする

1. TG15本体とUSBケーブルをUSBアダプターに接続します。
2. LiDARセンサを接続するUSBポートに権限を付与します。
3. テスト


## LiDARセンサを接続するUSBポートに権限を付与

例：
```
sudo chmod 777 /deb/tty/USB0
```

## テスト

```
cd YDLidar-SDK/build
./tri_test
```

* port番号 ： LiDARセンサのものを選択
* Baudrate： 512000 bps
* one-way scan：no
* frequency：10 Hz
