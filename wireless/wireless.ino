#include <SPI.h>
#include <string.h>
#include <CCP_MCP2515.h>

//CAN
#define CAN0_CS D0
#define CAN0_INT D1

CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

//デバッグ
// #define debug

//テレメトリー
int downlink = 0;

uint32_t data_A_GNSS_latitude_udeg = 0;
uint32_t data_A_GNSS_longitude_udeg = 0;

const unsigned long interval = 1000;  // 処理の間隔を10ミリ秒（100Hz）に設定
unsigned long previousMillis = 0;     // 前回の処理時間を保存する変数

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  // CAN設定
  pinMode(CAN0_CS, OUTPUT);  //0ピンをCAN通信の送信用に設定
  pinMode(CAN0_INT, INPUT);  //1ピンをCAN通信の入力用に設定
  digitalWrite(CAN0_CS, HIGH);
  CCP.begin();

  delay(100);
}


void loop() {
  unsigned long currentMillis = millis();  // 現在の時間を取得

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
    } else if (inputString == "OPEN") {
      CCP.string_to_device(CCP_parachute_control, const_cast<char*>("OPEN"));
      Serial.println("OPEN");
    } else if (inputString == "CLOSE") {
      CCP.string_to_device(CCP_parachute_control, const_cast<char*>("CLOSE"));
      Serial.println("CLOSE");
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
      CCP.string_to_device(CCP_parachute_control, const_cast<char*>("OPEN"));
      Serial.println("OPEN");
    } else if (inputString == "CLOSE") {
      CCP.string_to_device(CCP_parachute_control, const_cast<char*>("CLOSE"));
      Serial.println("CLOSE");
    }
  }

#endif

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

  // Serial.print("downlink:");
  // Serial.println(downlink);
  // Serial.println(downlink, BIN);

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // 前回の処理時間を現在の時間に更新

    // Serial1.print("downlink:");
    // Serial1.println(downlink);
    Serial1.print(downlink, BIN);
    Serial1.print(",");
    Serial1.print(data_A_GNSS_latitude_udeg);
    Serial1.print(",");
    Serial1.println(data_A_GNSS_longitude_udeg);
  }
}
