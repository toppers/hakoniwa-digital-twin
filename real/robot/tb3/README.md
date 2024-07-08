# TB3 プログラムにパッチを適用する

以下のファイルを、[real/robot/tb3/robot.launch.py](robot.launch.py)に置き換えてください。

* turtlebot3_ws/src/turtlebot3/turtlebot3_bringup/launch/robot.launch.py 

TB3のROSトピック名がインフラセンサーのものと重複するため、接頭辞 `tb3_` をつける修正です。

# Ubuntu側のファイアウォールの設定

```
状態: アクティブ

To                         Action      From
--                         ------      ----
3389/tcp                   ALLOW       Anywhere                  
22/tcp                     ALLOW       Anywhere                  
7400:7500/udp              ALLOW       Anywhere                  
7400:7500/tcp              ALLOW       Anywhere                  
53/udp                     ALLOW       Anywhere                  
53/tcp                     ALLOW       Anywhere                  
3389/tcp (v6)              ALLOW       Anywhere (v6)             
22/tcp (v6)                ALLOW       Anywhere (v6)             
7400:7500/udp (v6)         ALLOW       Anywhere (v6)             
7400:7500/tcp (v6)         ALLOW       Anywhere (v6)             
53/udp (v6)                ALLOW       Anywhere (v6)             
53/tcp (v6)                ALLOW       Anywhere (v6)   
```

コマンド：
```
sudo ufw status
sudo ufw allow 22/tcp  # SSHのポートを開放
sudo ufw allow 7400:7500/udp  # ROS2のデフォルトポートを開放
sudo ufw allow 7400:7500/tcp  # 必要に応じてTCPポートも開放
sudo ufw allow 53/udp  # DNSサービスのためのポートを開放
sudo ufw allow 53/tcp  # 必要に応じてTCPも開放
````

