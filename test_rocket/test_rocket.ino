#include <string.h>
// #include <TimerTCC0.h>

// bool is1hz = false;
// bool comand = false;

String downlink = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  //   TimerTcc0.initialize(3000000);  // 10,000us=100Hz
  //   TimerTcc0.attachInterrupt(TimerIsr);
}

void loop() {
  if (Serial1.available()) {
    String inputString = Serial1.readStringUntil('\n');  // 受信した文字列を読み取ります
    inputString.trim();
    Serial.println(inputString);
    if (inputString == "A") {
      downlink = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
    } else if (inputString == "B") {
      downlink = "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB";
    }
  }

  Serial1.println(downlink);
  Serial.println(downlink);
  delay(3000);
  // if (is1hz) {
  //   is1hz = false;
  //   Serial1.println(downlink);
  //   Serial.println(downlink);
  // }
}

// void TimerIsr() {
//   if (is1hz) {
//     Serial.println("overrun");
//   }
//   is1hz = true;
// }
