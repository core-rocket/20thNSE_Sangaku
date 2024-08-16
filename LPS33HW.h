#include <SPI.h>                 // SPI通信を使用するためのlibraryをinclude
#include <string.h>              // 文字列操作用のlibraryをinclude
#include <CCP_MCP2515.h>         // CAN通信libraryをinclude
#include <Adafruit_LPS35HW.h>    // LPS35HWPressure-sensor-libraryをinclude

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();  // LPS35HWsensorのインスタンスを作成

#define CAN_INT D6              // CAN通信の割り込みピンをD6に設定
#define CAN_CS D7               // CAN通信のチップセレクトピンをD7に設定

// CAN通信のinstanceを作成
CCP_MCP2515 CCP(CAN_CS, CAN_INT);
// char msgString[128];           // output-message用の文字列バッファ
// char str_buf[7];              // 文字列データのbuffer（最大6文字+終端文字）

void setup() {
  delay(500);                  // 起動後に500msの遅延を挿入
  Serial.begin(115200);       // serial通信を115200baudで初期化
  pinMode(CAN_CS, OUTPUT);    // CAN_CS pinをoutput-modeに設定
  pinMode(CAN_INT, INPUT);    // CAN_INT pinをinput-modeに設定
  digitalWrite(CAN_CS, HIGH); // CAN_C pinをHIGHに設定

  // CAN通信の初期化
  CCP.begin();

  // LPS35HWの初期化
  while (!lps35hw.begin_I2C(0x5C)) {  // I2C address 0x5Cでsensorを初期化
    Serial.println("Couldn't find LPS35HW chip"); // sensorが見つからない場合、error messageを表示
    delay(10);               // 10msの遅延
  }
  delay(10);                 // さらに10msの遅延
  lps35hw.setDataRate(LPS35HW_RATE_75_HZ); // sensorのdatalateを75Hzに設定
}

void loop() {
  static uint32_t time = 0; // 前回のtimestampを保存するための静的変数
  if (millis() - time >= 10) { // 10msごとに処理を実行
    time = millis();           // 現在のtime stampを保存
    CCP.float_to_device(CCP_surface_pressure1_pressure_pa, lps35hw.readPressure()); // 圧力dataをCAN deviceに送信
  }
 }