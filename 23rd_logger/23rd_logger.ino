//loggerをシンプルにしてるコード．(エクスポートはできないようになってる)
#include <CCP_MCP2515.h>
#include <CCP_W25Q512.h>
#include <SD.h>
// #include<TimerTCC0.h>
// #include <TinyGPSPlus.h>

#define DEBUG

//pins
#define CAN_CS 0
#define CAN_INT 1
#define FLASH_CS 3
#define SD_CS 5
// #define LED_YELLOW LED_BUILTIN
// #define LED_BLUE PIN_LED_RXL
// //timer
// #define TIMERPULSE 1000000//1Hz

typedef enum mode {
  SLEEP,
  CLEARING,
  ACTIVE,
} MODE;

MODE mode;

CCP_MCP2515 CCP_can(CAN_CS, CAN_INT);
CCP_W25Q512 CCP_flash(SPI, FLASH_CS, CAN_INT);

bool flashava = false;

// TinyGPSPlus gps;

// //timer
// bool timer_1Hz = false;



void setup()
{
  pinMode(CAN_CS, OUTPUT);
  pinMode(CAN_INT, INPUT);
  pinMode(FLASH_CS, OUTPUT);
  // pinMode(LED_YELLOW, OUTPUT);
  digitalWrite(CAN_CS    , HIGH);
  digitalWrite(FLASH_CS   , HIGH);
  digitalWrite(SD_CS      , HIGH);
  // digitalWrite(LED_YELLOW , LOW);
  // digitalWrite(LED_BLUE   , HIGH);
  flashava = CANFlashInit();
  // Serial1.begin(9600);//gps
  Serial.begin(112500);


#ifdef DEBUG
  delay(3000);

  Serial.print(F("serial:ok,flash:"));
  if (flashava) {
    Serial.print(F("ok"));
  } else {
    Serial.print(F("err"));
  }
  Serial.println();
#endif
  // //timer
  // TimerTcc0.initialize(TIMERPULSE);
  // TimerTcc0.attachInterrupt(TimerIsr);
}

void loop() {
//   while (Serial1.available() > 0) {
//     if (gps.encode(Serial1.read())) {
//       if (gps.location.isValid()) {
//         CCP_can.uint32_to_device(CCP_GNSS_latitude_udeg, gps.location.lat() * 1000000);
//         CCP_can.uint32_to_device(CCP_GNSS_longitude_udeg, gps.location.lng() * 1000000);
// #ifdef DEBUG
//         Serial.print(gps.location.lat(), 6);
//         Serial.print(F(","));
//         Serial.println(gps.location.lng(), 6);
// #endif
//       }
//     }
//   }

  if (CCP_flash.flash_addr >= 0x3FFFF00) {
    if (mode == ACTIVE) {
      // digitalWrite(LED_YELLOW, LOW);
      CCP_can.string_to_device(CCP_A_flash_state, "SLEEP");
      mode = SLEEP;
    }
  }

  if (digitalRead(CAN_INT)) {
    switch (mode) {
      case SLEEP:
        CCP_flash.flash_buf();
      case CLEARING:
        if (CCP_flash.IsBusy()) {
          // digitalWrite(LED_YELLOW, !digitalRead(LED_YELLOW));
          delay(100);
        } else {
          // digitalWrite(LED_YELLOW, LOW);
          mode = SLEEP;
          CCP_can.string_to_device(CCP_A_flash_state, "SLEEP");
        }
      case ACTIVE:
        CCP_flash.flash_buf();
      default:
        break;
    }
  }

  if (!digitalRead(CAN_INT)) { //when CAN message exists
    CCP_can.read_device();
    if (CCP_can.id == CCP_A_flash_control) {
      if (CCP_can.str_match("CHECK", 5)) {
        if (CCP_flash.is_cleared()) {
          CCP_can.string_to_device(CCP_A_flash_state, "OK-CLR");
        } else {
          CCP_can.string_to_device(CCP_A_flash_state, "NOTCLR");
        }
      } else if (CCP_can.str_match("START", 5)) {
        mode = ACTIVE;
        // digitalWrite(LED_YELLOW, HIGH);
        CCP_can.string_to_device(CCP_A_flash_state, "ACTIVE");
      } else if (CCP_can.str_match("STOP", 4)) {
        mode = SLEEP;
        // digitalWrite(LED_YELLOW, LOW);
        CCP_can.string_to_device(CCP_A_flash_state, "SLEEP");
      } else if (CCP_can.str_match("CLEAR", 5)) {
        CCP_flash.clear_flash(false);//no wait
        mode = CLEARING;
        CCP_can.string_to_device(CCP_A_flash_state, "CLRING");
      }
    }
    if (mode == ACTIVE) {
      CCP_flash.byte_to_device(CCP_can.id, CCP_can.msg.msg_byte);
    }
  }

  // if (timer_1Hz) {
  //   //heart beat追加するときここをコメントアウト
  //   // switch (mode) {
  //   //   case SLEEP:
  //   //     CCP_can.string_to_device(CCP_A_flash_state, "SLEEP");
  //   //     if (CCP_flash.is_cleared()) {
  //   //       CCP_can.string_to_device(CCP_A_flash_state, "OK-CLR");
  //   //     } else {
  //   //       CCP_can.string_to_device(CCP_A_flash_state, "NOTCLR");
  //   //     }
  //   //   case CLEARING:
  //   //     CCP_can.string_to_device(CCP_A_flash_state, "CLRING");
  //   //   case ACTIVE:
  //   //     CCP_can.string_to_device(CCP_A_flash_state, "ACTIVE");
  //   //   default:
  //   //     break;
  //   // }
  //   timer_1Hz = false;
  // }

}

// void TimerIsr() {
// #ifdef DEBUG
//   if (timer_1Hz) {
//     Serial.println(F("1Hzoverrun"));
//   }
// #endif
//   timer_1Hz = true;
// }



bool CANFlashInit() {
  // digitalWrite(LED_YELLOW, LOW);
  CCP_can.begin();
  if (CCP_flash.begin() != 4) {
    CCP_can.string_to_device(CCP_A_flash_state, "ERR");
    return false;
  }
  return true;
}