#include <SPI.h>
#include <string.h>
#include <CCP_MCP2515.h>

#define CAN0_CS D0
#define CAN0_INT D1

CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

//テレメトリー
int downlink_binary = 0;
int downlink = 0;

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
  // put your main code here, to run repeatedly:
  // 地上局へのテレメトリ
  if (!digitalRead(CAN0_INT))  // データ受信確認
  {
    CCP.read_device();
    switch (CCP.id) {
      case CCP_downkink:
        downlink_binary = CCP.data_uint32();
        break;
      default:
        break;
    }
  }


  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // 前回の処理時間を現在の時間に更新

    downlink = bin_to_dec(downlink_binary);
    Serial1.print("downlink:");
    Serial1.print(downlink);
  }
}

void bin_to_dec{

}

