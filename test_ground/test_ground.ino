#include <string.h>

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  if (Serial1.available()) {
    String inputString = Serial1.readStringUntil('\n');  // 受信した文字列を読み取ります
    inputString.trim();
    Serial.println(inputString);
  }

  if (Serial.available()) {
    String uplink = Serial.readStringUntil('\n');  // 書き込んだ文字列を送信します
    uplink.trim();
    Serial1.println(uplink);
  }
}