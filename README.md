# maestro_driver

Pololu Maestro サーボコントローラ用の ROS2 ブリッジノードです。
`/rc_pwm` トピックで受信した RC PWM 値を USB シリアル経由で Maestro に送信します。

## 動作環境

- ROS2 Jazzy
- Python 3
- pyserial (`pip install pyserial`)
- Pololu Maestro (USB Dual Port モード)

## インストール

```bash
cd ~/ros2_ws/src
git clone <repository_url> maestro_driver
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select maestro_driver
source install/setup.bash
```

## 使い方

### 基本起動

```bash
ros2 run maestro_driver maestro_driver_node
```

### パラメータ指定

```bash
ros2 run maestro_driver maestro_driver_node --ros-args \
  -p serial_port:=/dev/ttyACM1 \
  -p ch1_channel:=0 \
  -p ch2_channel:=1 \
  -p failsafe_timeout:=1.0
```

## トピック

| トピック名 | 型 | 方向 | 説明 |
|---|---|---|---|
| `/rc_pwm` | `std_msgs/msg/UInt16MultiArray` | Subscribe | PWM パルス幅 (data[0]=ch1, data[1]=ch2) |

- PWM 値はマイクロ秒単位 (1000~2000, 中央値 1500)
- QoS: BestEffort, depth=1
- 想定配信レート: 約 10 Hz

## パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `serial_port` | string | `/dev/ttyACM0` | Maestro の Command Port デバイスパス |
| `ch1_channel` | int | `0` | Maestro のチャンネル番号 (ch1 用) |
| `ch2_channel` | int | `1` | Maestro のチャンネル番号 (ch2 用) |
| `failsafe_timeout` | double | `0.5` | フェールセーフ発動までの秒数 |
| `failsafe_pwm` | int | `1500` | フェールセーフ時の PWM 値 (us) |
| `pwm_min` | int | `1000` | 許容 PWM 最小値 (us) |
| `pwm_max` | int | `2000` | 許容 PWM 最大値 (us) |

## フェールセーフ

`/rc_pwm` トピックが `failsafe_timeout` 秒以上途絶えると、両チャンネルに `failsafe_pwm` (デフォルト 1500us = ニュートラル) を自動送信します。

- ノード起動直後は最初のメッセージ受信までフェールセーフ状態
- トピック受信が再開すると自動復帰
- ノードシャットダウン時にもフェールセーフ値を送信してから終了

## シリアル通信

Maestro の Compact Protocol (Set Target コマンド `0x84`) を使用します。

- USB Dual Port モードのため、ボーレート設定は不要
- ポートが見つからない場合は 1 秒間隔で自動リトライ
- 通信中のエラー発生時はポートを閉じて自動再接続

## Maestro 側の設定

1. Pololu Maestro Control Center でシリアルモードを **USB Dual Port** に設定
2. 使用するチャンネルのモードを **Servo** に設定
3. Linux では `/dev/ttyACM0` (Command Port) として認識される

## ライセンス

MIT
