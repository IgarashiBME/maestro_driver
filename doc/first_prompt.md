以下の仕様に基づいて、ROS2 (Jazzy) の Python ノード `maestro_driver_node.py` を実装してください。

## 概要
`/rc_pwm` トピック (UInt16MultiArray) をサブスクライブし、
USB接続された Pololu Maestro サーボコントローラにシリアル通信で
RC PWM値を送信するブリッジノードです。

## 入力トピック
- トピック名: `/rc_pwm`
- 型: `std_msgs/msg/UInt16MultiArray`
- data[0] = ch1 の PWM パルス幅 (μs単位, 1000〜2000, 中央値1500)
- data[1] = ch2 の PWM パルス幅 (μs単位, 同上)
- 配信レート: 約 10 Hz

## Maestro シリアルプロトコル仕様
- USB接続時、Maestroは "USB Dual Port" モードで使用する
- Linuxでは `/dev/ttyACM0` 等の Command Port として認識される
- Compact Protocol の **Set Target** コマンドを使う:
  - バイト列: `0x84, channel, target_low, target_high`
  - target = パルス幅(μs) × 4 (quarter-microseconds単位)
  - target_low = target & 0x7F (下位7ビット)
  - target_high = (target >> 7) & 0x7F (上位7ビット)
- 例: 1500μs → target = 6000 → low = 0x70, high = 0x2E
  → 送信バイト: [0x84, 0x00, 0x70, 0x2E]
- ボーレート設定は不要 (USB Dual Port では OS が直接通信する)
- Mini Maestro の場合は **Set Multiple Targets** も利用可能:
  - バイト列: `0x9F, num_targets, first_channel, target1_low, target1_high, target2_low, target2_high, ...`
  - 2チャンネル同時設定で効率的

## ROS2 パラメータ (すべて declare_parameter で宣言)
- `serial_port` (string, default: "/dev/ttyACM0") — Maestro の Command Port デバイスパス
- `ch1_channel` (int, default: 0) — Maestro のチャンネル番号 (ch1用)
- `ch2_channel` (int, default: 1) — Maestro のチャンネル番号 (ch2用)
- `failsafe_timeout` (double, default: 0.5) — /rc_pwm が途絶えてからフェールセーフ発動までの秒数
- `failsafe_pwm` (int, default: 1500) — フェールセーフ時に送信する PWM 値 (μs)。中央値=ニュートラル
- `pwm_min` (int, default: 1000) — 許容する PWM 最小値 (μs)
- `pwm_max` (int, default: 2000) — 許容する PWM 最大値 (μs)

## フェールセーフ機能 (重要)
- タイマーを使い、`failsafe_timeout` 秒ごとにトピックの受信状態をチェックする
- 最後のメッセージ受信時刻と現在時刻の差が `failsafe_timeout` を超えた場合:
  1. `failsafe_pwm` の値を両チャンネルに送信する (ニュートラル停止)
  2. WARN レベルのログを出力する (throttle 付き、1秒に1回程度)
- `/rc_pwm` が再び受信されたら通常動作に復帰し、INFO ログを出力する
- ノード起動時、最初のメッセージ受信前もフェールセーフ状態とする
- ノードシャットダウン時 (on_shutdown / destroy_node) にも `failsafe_pwm` を送信してからポートを閉じる

## シリアル通信の堅牢性
- pyserial を使用する (`import serial`)
- ノード起動時にポートが見つからない場合は ERROR ログを出して再試行する
  (create_timer で定期的にリトライ、接続成功したら INFO ログ)
- 通信中に serial.SerialException が発生した場合:
  - ポートを閉じて再接続を試みる
  - 再接続中もフェールセーフ状態として扱う
- ポートが開いていない状態でメッセージを受信した場合はバッファせずに破棄する

## コード構造
```python
class MaestroDriverNode(Node):
    def __init__(self):
        # パラメータ宣言
        # シリアルポート初期化 (失敗時はリトライタイマー設定)
        # サブスクライバ作成 (/rc_pwm)
        # フェールセーフタイマー作成
        # 状態変数: last_msg_time, is_failsafe, ser (serial.Serial or None)

    def rc_pwm_callback(self, msg):
        # msg.data[0], msg.data[1] を取得
        # pwm_min/pwm_max でクランプ
        # Maestro に Set Target 送信
        # last_msg_time を更新
        # フェールセーフ復帰処理

    def failsafe_check(self):
        # タイマーコールバック
        # タイムアウト判定 → フェールセーフ PWM 送信

    def send_target(self, channel: int, pwm_us: int):
        # PWM (μs) → quarter-microseconds 変換
        # Compact protocol Set Target バイト列生成・送信
        # SerialException ハンドリング

    def send_failsafe(self):
        # 両チャンネルに failsafe_pwm を送信

    def connect_serial(self):
        # serial.Serial() でポートオープン
        # 成功/失敗のログ

    def on_shutdown(self):
        # フェールセーフ送信 → ポートクローズ
```

## その他
- QoSProfile は depth=1 の BestEffort を使用 (リアルタイム制御向け)
- ログは get_logger() を使用し、適切な throttle_duration_sec を設定
- シリアル通信の書き込みはブロッキングだが、10Hz程度なら問題ない
  (Set Target 2ch分で8バイト、USB経由なので実質的に瞬時)
- setup.py の console_scripts エントリポイントも忘れずに設定する
- 型ヒント (type hints) を付ける
- docstring をクラスと各メソッドに記述する

パッケージ名: maestro_driver
ノードファイル: maestro_driver/maestro_driver_node.py
