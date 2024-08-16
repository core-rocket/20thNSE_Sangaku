#include <string.h>
#include <SoftwareSerial.h>

#define rxPin D2
#define txPin D1

// Set up a new SoftwareSerial object
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

void setup() {
  // Define pin modes for TX and RX
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  // Set the baud rate for the SoftwareSerial object
  mySerial.begin(115200);
  Serial.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  if (mySerial.available() > 0) {
    String inputString = mySerial.readStringUntil('\n');  // 受信した文字列を読み取ります
    inputString.trim();
    Serial.println(inputString);
  }

  if (Serial1.available() > 0) {
    String inputString = Serial1.readStringUntil('\n');  // 受信した文字列を読み取ります
    inputString.trim();
    Serial.println(inputString);
  }

  // if (Serial.available()) {
  //   String uplink = Serial.readStringUntil('\n');  // 書き込んだ文字列を送信します
  //   uplink.trim();
  //   mySerial.println(uplink);
  // }
}
