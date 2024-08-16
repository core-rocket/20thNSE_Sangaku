#include <CCP_MCP2515.h>
#include <CCP_W25Q512.h>
#include <SPI.h>

#define CAN0_CS D0
#define CAN0_INT D1
#define FLASH_CS D2
#define redled 16
#define greenled 17

CCP_MCP2515 CCP_can(CAN0_CS, CAN0_INT);
CCP_W25Q512 CCP_flash(SPI, FLASH_CS, CAN0_INT);

typedef enum {
  ACTIVE,
  SLEEP,
  CLRING,
} MODE;

MODE mode;

void setup() {
  delay(500);

  pinMode(CAN0_CS, OUTPUT);
  pinMode(CAN0_INT, INPUT);
  pinMode(FLASH_CS, OUTPUT);
  pinMode(redled, OUTPUT);
  pinMode(greenled, OUTPUT);

  digitalWrite(CAN0_CS, HIGH);
  digitalWrite(FLASH_CS, HIGH);
  digitalWrite(redled, LOW);
  digitalWrite(greenled, LOW);

  Serial.println(115200);
  delay(1000);

  CCP_can.begin();

  while (CCP_flash.begin() != 4) {
    CCP_can.string_to_device(CCP_A_flash_state, const_cast<char*>("ERR"));
    Serial.println("flasherr");
    delay(500);
  }

  mode = ACTIVE;
}

void loop() {
  if (Serial.available() > 0) {
    String comand = Serial.readStringUntil('\n');
    comand.trim();
    if (comand = "active") {
      mode = ACTIVE;
      Serial.println("ACIVE MODE");
    } else if (comand = "clear") {
      mode = CLRING;
      Serial.println("CLR");
    }
  }
  if (CCP_flash.flash_addr >= 0x3FFFF00) {
    if (mode == ACTIVE) {
      digitalWrite(greenled, LOW);
      CCP_can.string_to_device(CCP_A_flash_state, const_cast<char*>("SLEEP"));
      mode = SLEEP;
    }
  }

  if (mode == ACTIVE) {
    CCP_flash.byte_to_device(CCP_can.id, CCP_can.msg.msg_byte);
  }

  if (digitalRead(CAN0_INT)) {
    CCP_flash.flash_buf();
    Serial.println("Writing");
    if (mode == CLRING) {
      if (CCP_flash.IsBusy()) {
        digitalWrite(greenled, !digitalRead(greenled));
        Serial.println("");
        // delay(100);
      } else {
        digitalWrite(greenled, HIGH);
        CCP_can.string_to_device(CCP_A_flash_state, const_cast<char*>("SLEEP"));
        mode = SLEEP;
      }
    }
  }

  if (!digitalRead(CAN0_INT)) {
    CCP_can.read_device();

    if (CCP_can.id == CCP_A_flash_control) {
      if (CCP_can.str_match(const_cast<char*>("CHECK"), 5)) {
        if (CCP_flash.is_cleared()) {
          CCP_can.string_to_device(CCP_A_flash_state, const_cast<char*>("OK-CLR"));
        } else {
          CCP_can.string_to_device(CCP_A_flash_state, const_cast<char*>("NOTCLR"));
        }
      } else if (CCP_can.str_match(const_cast<char*>("START"), 5)) {
        digitalWrite(greenled, LOW);
        CCP_can.string_to_device(CCP_A_flash_state, const_cast<char*>("ACTIVE"));
        mode = ACTIVE;
      } else if (CCP_can.str_match(const_cast<char*>("STOP"), 4)) {
        digitalWrite(greenled, LOW);
        CCP_can.string_to_device(CCP_A_flash_state, const_cast<char*>("SLEEP"));
        mode = SLEEP;
      } else if (CCP_can.str_match(const_cast<char*>("CLEAR"), 5)) {
        CCP_flash.clear_flash(false);
        CCP_can.string_to_device(CCP_A_flash_state, const_cast<char*>("CLRING"));
        mode = CLRING;
      }
    }
  }
}