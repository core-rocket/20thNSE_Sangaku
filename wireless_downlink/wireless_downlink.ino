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

//テレメトリー
int downlink_key = 0;
int downlink_emst = 0;
int downlink_STM_1 = 0;  //state transition model
int downlink_STM_2 = 0;
int downlink_open_accel = 0;
int downlink_open_altitude = 0;
int downlink_outground_accel = 0;
int downlink_outground_altitude = 0;
int downlink_meco_time = 0;
int downlink_top_time = 0;


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
    switch (CCP.id) {]
      case CCP_parachute_fuse:
        if (CCP.str_match("NOTOPEN", 7)) {
        downlink_emst = 1;
      }
      else if (CCP.str_match("CLEAR", 5)) {
        downlink_emst = 0;
      }
      break;
      case CCP_key_state:
        if (CCP.str_match("ON", 2)) {
          downlink_key = 1;
        } else if (CCP.str_match("OFF", 3)) {
          downlink_key = 0;
        }
        break;
      case CCP_opener_state:
        if (CCP.str_match("CHECK", 5)) {
          downlink_STM_1 = 0;
          downlink_STM_2 = 0;
        } else if (CCP.str_match("READY", 5)) {
          downlink_STM_1 = 0;
          downlink_STM_2 = 1;
        } else if (CCP.str_match("FLIGHT", 6)) {
          downlink_STM_1 = 1;
          downlink_STM_2 = 0;
        } else if (CCP.str_match("OPENED", 6)) {
          downlink_STM_1 = 1;
          downlink_STM_2 = 1;
        }
        break;
      case CCP_lift_off_judge:
        if (CCP.str_match("ACCSEN", 6)) {
          downlink_outground_accel = 1;
        } else if (CCP.str_match("ALTSEN", 6)) {
          downlink_outground_altitude = 1;
        } else if (CCP.str_match("------", 6)) {
          downlink_outground_altitude = 0;
          downlink_outground_altitude = 0;
        }
        break;
      case CCP_lift_off_judge:
        if (CCP.str_match("ACCSEN", 6)) {
          downlink_meco_time = 1;
        }
        break;
      case CCP_lift_off_judge:
        if (CCP.str_match("ACCSEN", 6)) {
          downlink_top_time = 1;
        }
        break;
      case CCP_lift_off_judge:
        if (CCP.str_match("ACCSEN", 6)) {
          downlink_open_accel = 1;
        } 
        break;
      case CCP_lift_off_judge:
        if (CCP.str_match("ACCSEN", 6)) {
          downlink_open_altitude = 1;
        } 
        break;
      default:
        break;
    }
  }

  //Downlink
  //テレメトリ送信
  Serial1.print("downlink:");
  Serial1.print(downlink_emst);
  Serial1.print(downlink_key);
  Serial1.print(downlink_STM_1);
  Serial1.print(downlink_STM_2);
  Serial1.print(downlink_outground_accel);
  Serial1.print(downlink_outground_altitude);
  Serial1.print(downlink_meco_time);
  Serial1.print(downlink_top_time);
  Serial1.print(downlink_open_accel);
  Serial1.println(downlink_open_altitude);
}
