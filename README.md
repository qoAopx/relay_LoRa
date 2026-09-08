# ESP32_RELAY_LoRa

ESP32 で動作する LoRa 受信 relay 用スケッチです。E220-900T22S(JP) モジュールから受信したデータを、BLE 経由でスマートフォン側へそのまま転送します。

本プロジェクトは、RC Lap Timer のような計測データを LoRa で受け取り、BLE 通知として送る用途を想定しています。受信したペイロードは文字列変換せず、元のバイト列をそのまま BLE の characteristic に設定して送信します。

---

## 概要

- MCU: ESP32
- LoRa モジュール: E220-900T22S(JP)
- 通信方式: LoRa を受信し、BLE で通知
- 想定用途: ラップタイムやセンサ値の中継
- 動作方針: 「LoRa 受信 → そのまま BLE へ転送」

このスケッチでは、LoRa で受信したデータを `RC_LAPTIMER_RELAY` という BLE デバイス名で広告し、接続中の phone に `notify()` で送ります。

---

## 主な機能

### 1. LoRa 受信と BLE 転送

- `lora.ReceiveFrame()` で E220 から受信データを取得
- 受信した `recv_data` を `forwardToBLE()` に渡す
- `pRelayCharacteristic->setValue(data, length);`
- `pRelayCharacteristic->notify();` により BLE クライアントへ送信

### 2. 受信データの内容保持

元データを破壊せずにそのまま送信する設計です。たとえば `Lap: 12.34` のような文字列形式を保持したまま、BLE 側に通知します。

### 3. ラップタイム文字列の抽出

受信データの中に `:` を含む場合、`updateLapDisplay()` が最後の lap 表示用文字列を抽出し、`lastLapStr` に格納します。

- 文字列の前後空白を除去
- `:` の右側の数値部分だけを保存
- 画面表示やデバッグ用途に利用可能

### 4. 受信成功時のブザー通知

- AUX が LOW の状態で受信成功した場合、ブザーを鳴らす
- `tone(BUZZER_PIN, 4000);` により短い音を出力
- 一定時間経過後に `noTone()` で停止

### 5. デバッグ出力

- `Serial.printf()` と `Serial.println()` により LoRa の受信内容や BLE 接続状態をログ出力
- 受信フレーム数、転送成功数、破棄数、エラー数をカウント

---

## ハードウェア接続

デフォルトのライブラリ定義をそのまま使う前提です。

### ピン割り当て

| E220-900T22S(JP) | ESP32 |
|---|---|
| VCC | 5V |
| AUX | GPIO23 |
| TXD | GPIO16 |
| RXD | GPIO17 |
| M1 | GPIO33 |
| M0 | GPIO32 |
| GND | GND |

### キー定義

```cpp
#define LoRa_ModeSettingPin_M0 32
#define LoRa_ModeSettingPin_M1 33
#define LoRa_RxPin 16
#define LoRa_TxPin 17
#define LoRa_AUXPin 23
#define LoRa_BaudRate 9600
```

---

## Arduino / ESP32 側の設定

### 必要ファイル

同じフォルダ内に以下を置いて使用します。

- `ESP32_RELAY_LoRa.ino`
- `esp32_e220900t22s_jp_lib_v2.h`
- `esp32_e220900t22s_jp_lib_v2.cpp`
- `FIRMWARE.H`

### ファームウェア設定

`FIRMWARE.H` では以下のバージョンを使用します。

```cpp
#define FIRMWARE_VERSION 0x10
```

このプロジェクトは "E220-900T22S(JP) V1 firmware" を前提にしています。`FIRMWARE_VERSION` を変更する場合は、ライブラリ側の設定と整合性を確認してください。

---

## BLE 設定

このスケッチは BLE サーバーとして動作します。

```cpp
#define BLE_NAME "RC_LAPTIMER_RELAY"
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHAR_UUID "abcdefab-1234-5678-1234-abcdefabcdef"
```

- BLE 名: `RC_LAPTIMER_RELAY`
- サービス UUID: 互換性重視の固定 UUID
- キャラクタリスティック UUID: 通知用

接続された Phone 側でこの UUID を受信し、LoRa データを受け取る構成です。

---

## 動作フロー

1. ESP32 の起動時に `setup()` が実行される
2. BLE サーバーを作成して広告開始
3. LoRa モジュールの設定ファイルを読み込む
4. `InitLoRaModule()` で E220 の初期化を実行
5. `loop()` で AUX の状態を監視
6. AUX が LOW になった時に `ReceiveFrame()` を実行
7. 受信成功時に BLE に `notify()`
8. 成功した受信に対してブザーを鳴らす

---

## 受信時の挙動

- `ReceiveFrame()` が正常終了すると、`frame.recv_data` と `frame.recv_data_len` を得る
- `rssi` も取得可能
- 空のフレームは無視
- 受信成功時に `forwardToBLE()` が呼ばれる

`rssi_byte_flag = 1` としているため、受信時に RSSI を付加して受信しています。

---

## ファイル構成

```text
ESP32_RELAY_LoRa/
├── ESP32_RELAY_LoRa.ino
├── esp32_e220900t22s_jp_lib_v2.h
├── esp32_e220900t22s_jp_lib_v2.cpp
├── FIRMWARE.H
└── README.md
```

---

## 注意事項

- `FIRMWARE.H` の `FIRMWARE_VERSION` と実際のモジュールファームウェア対応を合わせてください。
- `SPIFFS` を使って設定ファイルを読み込むため、必要に応じて設定ファイルを ESP32 内に保存しておく必要があります。
- `AUX` は受信状態の判定に使うため、配線やピン設定がずれると受信が安定しません。
- BLE 接続が切れているときは、payload は通知されず、カウンタとして破棄数が増えます。

---

## まとめ

このスケッチは、ESP32 と E220-900T22S(JP) を組み合わせ、LoRa で送られてくるラップタイムや計測データを、BLE 経由でスマホ側へそのまま配信するための relay 実装です。

元データの整形をせず、そのまま転送する設計のため、既存の RC Lap Timer 系アプリとの互換性を保ちやすい構成になっています。

---

## ライセンス / 著作権

- MIT-License
- LoRaライブラリの著作権は `CLEALINK TECHNOLOGY Co.,Ltd.` です。
