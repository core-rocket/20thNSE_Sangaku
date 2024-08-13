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

uint32_t data_A_GNSS_latitude_udeg = 0;
uint32_t data_A_GNSS_longitude_udeg = 0;

void setup() {
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
  // 地上局へのテレメトリ
  if (!digitalRead(CAN0_INT))  // データ受信確認
  {
    CCP.read_device();
    switch (CCP.id) {
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

  if (is100hz) {
    is100hz = false;
  }

  if (is1hz) {
    is1hz = false;

    Serial1.print(data_A_GNSS_latitude_udeg);
    Serial1.print(",");
    Serial1.println(data_A_GNSS_longitude_udeg);
  }
}

void TimerIsr() {
  if (is100hz) {
    Serial.println("overrun");
  }
  is100hz = true;
}

void TimerCnt() {
  if (is1hz) {
    Serial.println("overrun10hz");
  }
  is1hz = true;
}
