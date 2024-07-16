#include <SPI.h>
#include <string.h>
#include <TimerTCC0.h>
#include <TimerTC3.h>

#include <CCP_MCP2515.h>

#define CAN0_CS 0
#define CAN0_INT 1

CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

bool is100hz = false;
bool is1hz = false;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(CAN0_CS, OUTPUT);
  pinMode(CAN0_INT, INPUT);

  digitalWrite(CAN0_CS, HIGH);
  CCP.begin();

  // CAN用タイマー
  TimerTcc0.initialize(10000);  // 10,000us=100Hz
  TimerTcc0.attachInterrupt(TimerIsr);

  // ダウンリンク用タイマー
  TimerTc3.initialize(1000000);
  TimerTc3.attachInterrupt(TimerCnt);

  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:
  // 地上局へのテレメトリ
  if (!digitalRead(CAN0_INT))  // データ受信確認
  {
    CCP.read_device();
    switch (CCP.id) {
      case :
        ;
        break;
      case :
        ;
        break;
      case :
        ;
        break;
      default:
        break;
    }
  }
}
