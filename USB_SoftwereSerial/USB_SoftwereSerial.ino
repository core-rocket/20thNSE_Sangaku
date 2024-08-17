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
}

void loop() {
  if (mySerial.available() > 0) {                //downlink
    String downlink1 = mySerial.readStringUntil('\n');  // 受信した文字列を読み取ります
    downlink1.trim();
    Serial.println(downlink1);
  }

  if (Serial.available() > 0) {                //GPS
    String downlink2 = Serial.readStringUntil('\n');  // 受信した文字列を読み取ります
    downlink2.trim();
    mySerial.println(downlink2);
  }
}