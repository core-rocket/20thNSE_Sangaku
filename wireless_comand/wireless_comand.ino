#include <SPI.h>
#include <string.h>
#include <CCP_MCP2515.h>
#include <TimerTCC0.h>
#include <TimerTC3.h>

//CAN
#define CAN0_CS D0
#define CAN0_INT D1

CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

//デバッグ
#define debug

void setup() {
  Serial.begin(115200);
  // CAN設定
  pinMode(CAN0_CS, OUTPUT);  //0ピンをCAN通信の送信用に設定
  pinMode(CAN0_INT, INPUT);  //1ピンをCAN通信の入力用に設定
  digitalWrite(CAN0_CS, HIGH);
  CCP.begin();

  // CAN用タイマー
  // TimerTcc0.initialize(10000);  // 10,000us=100Hz
  // TimerTcc0.attachInterrupt(TimerIsr);

  // ダウンリンク用タイマー
  // TimerTc3.initialize(1000000);
  // TimerTc3.attachInterrupt(TimerCnt);
}

void loop() {
  //　コマンド
  if (Serial1.available() > 0) {
    String inputString = Serial1.readStringUntil('\n');  // 受信した文字列を読み取る
    inputString.trim();                                  //前後の空白文字を削除する
    if (inputString == "N") {
      CCP.string_to_device(CCP_parachute_fuse, "NOTOPEN");
    } else if (inputString == "GO") {
      CCP.string_to_device(CCP_parachute_fuse, "CLEAR");
    } else if (inputString == "CHECK") {
      CCP.string_to_device(CCP_opener_control, "CHECK");
    } else if (inputString == "READY") {
      CCP.string_to_device(CCP_opener_control, "READY");
    } else if (inputString == "CHECK") {
      CCP.string_to_device(CCP_A_flash_control, "CHECK");
      CCP.string_to_device(CCP_B_flash_control, "CHECK");
      CCP.string_to_device(CCP_C_flash_control, "CHECK");
    } else if (inputString == "START") {
      CCP.string_to_device(CCP_A_flash_control, "START");
      CCP.string_to_device(CCP_B_flash_control, "START");
      CCP.string_to_device(CCP_C_flash_control, "START");
    } else if (inputString == "STOP") {
      CCP.string_to_device(CCP_A_flash_control, "STOP");
      CCP.string_to_device(CCP_B_flash_control, "STOP");
      CCP.string_to_device(CCP_C_flash_control, "STOP");
    } else if (inputString == "CLEAR") {
      CCP.string_to_device(CCP_A_flash_control, "CLEAR");
      CCP.string_to_device(CCP_B_flash_control, "CLEAR");
      CCP.string_to_device(CCP_C_flash_control, "CLEAR");
    }
  }

#ifdef debug
  if (Serial1.available() > 0) {
    String inputString = Serial1.readStringUntil('\n');  // 受信した文字列を読み取る
    inputString.trim();                                  //前後の空白文字を削除する
    if (inputString == "OPEN") {
      CCP.string_to_device(CCP_parachute_control, "OPEN");
      Serial.println("OPEN");
    } else if (inputString == "CLOSE") {
      CCP.string_to_device(CCP_parachute_control, "CLOSE");
      Serial.print("CLOSE");
    }
  }
#endif


  // if (!digitalRead(CAN0_INT)) {
  //   switch (ccp_CAN.id) {
  //     case CCP_parachute_fuse:
  //       if (ccp_CAN.str_match("N", 1)) {
  //       }
  //   }
  // }
}
