#include <SPI.h>
#include <string.h>
#include <CCP_MCP2515.h>

//CAN
#define CAN0_CS D0
#define CAN0_INT D1

CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

//デバッグ
#define debug

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  // CAN設定
  pinMode(CAN0_CS, OUTPUT);  //0ピンをCAN通信の送信用に設定
  pinMode(CAN0_INT, INPUT);  //1ピンをCAN通信の入力用に設定
  digitalWrite(CAN0_CS, HIGH);
  CCP.begin();
}


void loop() {
  //　コマンド
  if (Serial1.available() > 0) {
    String inputString = Serial1.readStringUntil('\n');  // 受信した文字列を読み取る
    inputString.trim();
    Serial.print("uplink==");     //前後の空白文字を削除する
    Serial.println(inputString);  // Serial1.println(inputString);
    if (inputString == "N") {
      CCP.string_to_device(CCP_parachute_fuse, const_cast<char*>("EMST"));
      Serial.println("NOTOPEN");
      // Serial1.println("return NOTOPEN");
    } else if (inputString == "GO") {
      CCP.string_to_device(CCP_parachute_fuse, const_cast<char*>("CLEAR"));
      Serial.println("CLEAR");
      // Serial1.println("return CLEAR");
    } else if (inputString == "CHECK") {
      CCP.string_to_device(CCP_opener_control, const_cast<char*>("CHECK"));
      Serial.println("CHECK");
      // Serial1.println("return CHECK");
    } else if (inputString == "READY") {
      CCP.string_to_device(CCP_opener_control, const_cast<char*>("READY"));
      Serial.println("READY");
      // Serial1.println("return READY");
    } else if (inputString == "OK") {
    } else {
      Serial.println("NG");
      // Serial1.println("return NG");
    }
    //  else if (inputString == "CHECK") {
    //   CCP.string_to_device(CCP_A_flash_control, "CHECK");
    //   CCP.string_to_device(CCP_B_flash_control, "CHECK");
    //   CCP.string_to_device(CCP_C_flash_control, "CHECK");
    // } else if (inputString == "START") {
    //   CCP.string_to_device(CCP_A_flash_control, "START");
    //   CCP.string_to_device(CCP_B_flash_control, "START");
    //   CCP.string_to_device(CCP_C_flash_control, "START");
    // } else if (inputString == "STOP") {
    //   CCP.string_to_device(CCP_A_flash_control, "STOP");
    //   CCP.string_to_device(CCP_B_flash_control, "STOP");
    //   CCP.string_to_device(CCP_C_flash_control, "STOP");
    // } else if (inputString == "CLEAR") {
    //   CCP.string_to_device(CCP_A_flash_control, "CLEAR");
    //   CCP.string_to_device(CCP_B_flash_control, "CLEAR");
    //   CCP.string_to_device(CCP_C_flash_control, "CLEAR");
    // }
  }
  // if (Serial.available()>0) {
  //     String uplink = Serial.readStringUntil('\n');  // 書き込んだ文字列を送信します
  //     uplink.trim();
  //     Serial1.println(uplink);
  //   }

#ifdef debug
  if (Serial1.available() > 0) {
    String inputString = Serial1.readStringUntil('\n');  // 受信した文字列を読み取る
    inputString.trim();                                  //前後の空白文字を削除する
    if (inputString == "OPEN") {
      byte sndStat1 = CCP.string_to_device(CCP_parachute_control, const_cast<char*>("OPEN"));
      if (sndStat1 == CAN_OK) {
        Serial.println("Message Sent Successfully!");
        // Serial1.println("Message Sent Successfully!");
      } else {
        Serial.println("Error Sending Message...");
        // Serial1.println("Error Sending Message...");
      }
      Serial.println("OPEN");
    } else if (inputString == "CLOSE") {
      byte sndStat = CCP.string_to_device(CCP_parachute_control, const_cast<char*>("CLOSE"));
      if (sndStat == CAN_OK) {
        Serial.println("Message Sent Successfully!");
        // Serial1.println("Message Sent Successfully!");
      } else {
        Serial.println("Error Sending Message...");
        // Serial1.println("Error Sending Message...");
      }

      Serial.println("CLOSE");
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
