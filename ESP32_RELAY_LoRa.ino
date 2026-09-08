/*
 * RC Lap Timer -- LoRa -> BLE relay (ESP32 / E220-900T22S(JP), firmware V1)
 *
 * Place this sketch, FIRMWARE.H, esp32_e220900t22s_jp_lib_v2.h, and
 * esp32_e220900t22s_jp_lib_v2.cpp in the same Arduino sketch folder.
 * The supplied library's standard LoRa pins are used:
 *   M0=GPIO32, M1=GPIO33, E220 TXD->ESP32 GPIO16, E220 RXD->GPIO17, AUX=GPIO23.
 *
 * LoRa payload bytes are relayed unchanged in one BLE notification.  This
 * preserves the original BLE message format (for example "Lap: 12.34").
 * 【配線（ライブラリヘッダのデフォルト値をそのまま使用）】
 *   E220-900T22S(JP)   ESP32
 *   VCC                5V
 *   AUX                GPIO23
 *   TXD                GPIO16
 *   RXD                GPIO17
 *   M1                 GPIO33
 *   M0                 GPIO32
 *   GND                GND
 */

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Wire.h>

#include "esp32_e220900t22s_jp_lib_v2.h"

// --- ピン定義 ---
const int BUZZER_PIN = 19;

// Keep the service and characteristic format used by the original relay.
#define BLE_NAME "RC_LAPTIMER_RELAY"
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHAR_UUID "abcdefab-1234-5678-1234-abcdefabcdef"

static BLEServer* pServer = nullptr;
static BLECharacteristic* pRelayCharacteristic = nullptr;
static volatile bool phoneConnected = false;
static volatile uint32_t bleConnectionGeneration = 0;

static CLoRa lora;
static LoRaConfigItem_t loraConfig;
static bool loraReady = false;
static uint32_t receivedFrames = 0;
static uint32_t forwardedFrames = 0;
static uint32_t droppedFrames = 0;
static char lastLapStr[16] = "--:--.--";
static uint32_t receiveErrors = 0;
static uint32_t emptyFrames = 0;
static bool lastAuxLow = false;
static uint32_t lastReceiveAttemptMs = 0;
static const uint32_t RECEIVE_RETRY_GUARD_MS = 2;
static unsigned long lastLapFlashUntilMs = 0;
const unsigned long LAP_FLASH_DURATION_MS = 800;
const int NOTE_FREQ = 4000;              // ブザー音の高さ（Hz）：4000Hz
const unsigned long TONE_DURATION = 50;  // ブザー鳴らす時間（ミリ秒）
// --- ブザー状態管理 ---
bool isBuzzerRinging = false;  // ブザーが鳴っているかどうか
int64_t buzzerStartTime = 0;   // ブザーを鳴らし始めた時刻

class PhoneServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    phoneConnected = true;
    ++bleConnectionGeneration;
    Serial.println(F("[BLE] Phone connected"));
  }

  void onDisconnect(BLEServer* server) override {
    phoneConnected = false;
    ++bleConnectionGeneration;
    Serial.println(F("[BLE] Phone disconnected; restarting advertising"));
    // Do not delay inside the BLE callback. A blocking delay here can stall
    // the BLE host task and make reconnects appear unreliable.
    BLEDevice::startAdvertising();
  }
};
static PhoneServerCallbacks phoneServerCallbacks;

void printPayload(const uint8_t* data, uint16_t length) {
  Serial.print(F("[LoRa] HEX : "));
  for (uint16_t i = 0; i < length; ++i) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    Serial.print(' ');
  }
  Serial.println();

  Serial.print(F("[LoRa] TEXT: "));
  for (uint16_t i = 0; i < length; ++i) {
    const uint8_t c = data[i];
    if (c >= 0x20 && c <= 0x7E) {
      Serial.write(c);
    } else {
      Serial.print(F("\\x"));
      if (c < 0x10) Serial.print('0');
      Serial.print(c, HEX);
    }
  }
  Serial.println();
}

// ゼロアロケーション版 updateLapDisplay
void updateLapDisplay(const uint8_t* data, uint16_t length) {
  if (length == 0) return;

  // ':' の位置をメモリ検索（String非使用）
  const uint8_t* colonPtr = static_cast<const uint8_t*>(memchr(data, ':', length));
  if (!colonPtr) return;

  size_t colonIdx = colonPtr - data;
  const char* valStart = reinterpret_cast<const char*>(colonPtr + 1);
  size_t valLen = length - (colonIdx + 1);

  // 前後の空白文字をスキップ (trim相当)
  while (valLen > 0 && isspace(static_cast<unsigned char>(*valStart))) {
    valStart++;
    valLen--;
  }
  while (valLen > 0 && isspace(static_cast<unsigned char>(valStart[valLen - 1]))) {
    valLen--;
  }

  if (valLen == 0) return;

  // 結果を固定長バッファへ安全にコピー
  size_t copyLen = (valLen < sizeof(lastLapStr) - 1) ? valLen : sizeof(lastLapStr) - 1;
  memcpy(lastLapStr, valStart, copyLen);
  lastLapStr[copyLen] = '\0';

  lastLapFlashUntilMs = millis() + LAP_FLASH_DURATION_MS;
}

void forwardToBLE(const uint8_t* data, uint16_t length, int rssi) {
  ++receivedFrames;
  Serial.printf("[Relay] LoRa frame #%lu: %u byte(s), RSSI %d dBm\n", static_cast<unsigned long>(receivedFrames),
                length, rssi);
  printPayload(data, length);
  updateLapDisplay(data, length);

  // Snapshot the connection state. The BLE callback can change it asynchronously.
  const uint32_t connectionGeneration = bleConnectionGeneration;
  if (!phoneConnected || pRelayCharacteristic == nullptr) {
    ++droppedFrames;
    Serial.println(F("[Relay] BLE phone is not connected; payload was not notified"));
    return;
  }

  // No text conversion or framing is done here: the original BLE payload is retained.
  pRelayCharacteristic->setValue(data, length);

  // If the phone disconnected while setValue() was executing, do not notify a
  // stale connection. This does not alter the packet format.
  if (!phoneConnected || connectionGeneration != bleConnectionGeneration) {
    ++droppedFrames;
    Serial.println(F("[Relay] BLE connection changed before notify; payload dropped"));
    return;
  }

  pRelayCharacteristic->notify();
  ++forwardedFrames;
  Serial.printf("[Relay] BLE notify sent: frame #%lu, %u byte(s) -> phone\n",
                static_cast<unsigned long>(forwardedFrames), length);
}

void setupBLE() {
  BLEDevice::init(BLE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(&phoneServerCallbacks);

  BLEService* service = pServer->createService(SERVICE_UUID);
  pRelayCharacteristic =
      service->createCharacteristic(CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY |
                                                   BLECharacteristic::PROPERTY_INDICATE);
  pRelayCharacteristic->addDescriptor(new BLE2902());
  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();
  Serial.println(F("[BLE] Advertising as RC_LAPTIMER_RELAY"));
}

bool setupLoRa() {
  pinMode(LoRa_AUXPin, INPUT_PULLUP);

  const int configResult = lora.LoadConfigSetting(CONFIG_FILENAME, loraConfig);
  if (configResult == 0) {
    Serial.printf("[LoRa] Loaded %s from SPIFFS\n", CONFIG_FILENAME);
  } else {
    Serial.println(F("[LoRa] Configuration file not found; using library defaults"));
  }

  // Enable the V1 RSSI byte. ReceiveFrame() removes it from recv_data_len and
  // exposes the value in RecvFrameE220900T22SJP_t::rssi.
  loraConfig.rssi_byte_flag = 1;
  const int initResult = lora.InitLoRaModule(loraConfig);
  lora.SwitchToNormalMode();
  delay(100);

  if (initResult != 0) {
    Serial.printf("[LoRa] Module initialization failed (code %d)\n", initResult);
    return false;
  }

  Serial.printf("[LoRa] Ready: addr=0x%04X, channel=%u, UART RX=%d TX=%d, M0=%d M1=%d AUX=%d\n", loraConfig.own_address,
                loraConfig.own_channel, LoRa_RxPin, LoRa_TxPin, LoRa_ModeSettingPin_M0, LoRa_ModeSettingPin_M1,
                LoRa_AUXPin);
  Serial.println(F("[LoRa] Receive mode active; waiting for E220 AUX LOW"));
  return true;
}

bool receiveAndRelay() {
  RecvFrameE220900T22SJP_t frame;
  memset(&frame, 0, sizeof(frame));

  Serial.println(F("[LoRa] AUX LOW: receiving frame..."));
  const int result = lora.ReceiveFrame(&frame);
  lastReceiveAttemptMs = millis();

  if (result != 0) {
    ++receiveErrors;
    Serial.printf("[LoRa] ReceiveFrame failed (code %d), errors=%lu\n",
                  result, static_cast<unsigned long>(receiveErrors));
    return false;
  }
  if (frame.recv_data_len == 0) {
    ++emptyFrames;
    Serial.printf("[LoRa] Empty frame ignored (empty=%lu)\n",
                  static_cast<unsigned long>(emptyFrames));
    return false;
  }

  forwardToBLE(frame.recv_data, frame.recv_data_len, frame.rssi);
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.println(F("\n=== RC LAP TIMER: LoRa -> BLE RELAY (E220 V1) ==="));
  setupBLE();
  for (int countLora = 0; countLora < 3; ++countLora) {
    loraReady = setupLoRa();
    if (loraReady) break;
    delay(500);  // 試行間の安定化待ち時間を確保
  }
}

void loop() {
  // The E220 holds AUX LOW while radio/UART receive data is pending.
  // Use the AUX level as a receive gate, but prevent the same LOW condition
  // from immediately retriggering the buzzer after a receive error.
  const bool auxLow = (digitalRead(LoRa_AUXPin) == LOW);

  if (loraReady && auxLow) {
    // If AUX has just gone LOW, receive immediately. If it remains LOW after
    // ReceiveFrame(), allow the library to drain another pending frame, but
    // avoid a tight error loop when the library returns an error.
    const uint32_t now = millis();
    const bool newAuxEvent = !lastAuxLow;
    const bool retryAllowed = (now - lastReceiveAttemptMs >= RECEIVE_RETRY_GUARD_MS);

    if (newAuxEvent || retryAllowed) {
      const bool received = receiveAndRelay();

      // Only a successfully received non-empty packet should produce the
      // one-shot relay buzzer. This prevents a stuck/low AUX signal from
      // causing continuous tone restarts.
      if (received && !isBuzzerRinging) {
        tone(BUZZER_PIN, NOTE_FREQ);
        buzzerStartTime = esp_timer_get_time();
        isBuzzerRinging = true;
      }
    }
  }

  lastAuxLow = auxLow;

  // ブザーが鳴っていたら停止
  if (isBuzzerRinging) {
    const uint64_t elapsedBuzzerMs =
        (static_cast<uint64_t>(esp_timer_get_time()) - static_cast<uint64_t>(buzzerStartTime)) / 1000ULL;
    if (elapsedBuzzerMs >= TONE_DURATION) {
      noTone(BUZZER_PIN);
      isBuzzerRinging = false;
    }
  }

  delay(5);
}
