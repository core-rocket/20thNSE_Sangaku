#include <SPI.h>
#include <string.h>
#include <CCP_MCP2515.h>

#define CAN0_CS D0
#define CAN0_INT D1

CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

//テレメトリー
int downlink = 0;

uint32_t data_A_GNSS_latitude_udeg = 0;
uint32_t data_A_GNSS_longitude_udeg = 0;

const unsigned long interval = 1000;  // 処理の間隔を10ミリ秒（100Hz）に設定
unsigned long previousMillis = 0;     // 前回の処理時間を保存する変数

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(CAN0_CS, OUTPUT);
  pinMode(CAN0_INT, INPUT);

  digitalWrite(CAN0_CS, HIGH);
  CCP.begin();

  delay(100);
}

void loop() {
  unsigned long currentMillis = millis();  // 現在の時間を取得
  // put your main code here, to run repeatedly:
  // 地上局へのテレメトリ
  if (!digitalRead(CAN0_INT))  // データ受信確認
  {
    CCP.read_device();
    switch (CCP.id) {
      case CCP_downlink:
        downlink = CCP.data_uint32();
        break;
      case CCP_A_GNSS_latitude_udeg:
        data_A_GNSS_latitude_udeg = CCP.data_uint32();
        break;
      case CCP_A_GNSS_longitude_udeg:
        data_A_GNSS_longitude_udeg = CCP.data_uint32();
        break;
      default:
        break;
    }
  }

  Serial.print("downlink:");
  Serial.println(downlink);
  Serial.println(downlink, BIN);

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // 前回の処理時間を現在の時間に更新

    // Serial1.print("downlink:");
    Serial1.println(downlink);
    // Serial1.println(downlink, BIN);
    Serial1.print(data_A_GNSS_latitude_udeg);
    Serial1.print(",");
    Serial1.println(data_A_GNSS_longitude_udeg);

    Serial.print(data_A_GNSS_latitude_udeg);
    Serial.print(",");
    Serial.println(data_A_GNSS_longitude_udeg);
  }
}
