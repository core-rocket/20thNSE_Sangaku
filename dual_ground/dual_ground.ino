#include <string.h>
#include <SoftwareSerial.h>

#define rxPin D2
#define txPin D1

// Set up a new SoftwareSerial object
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);


String downlink1 = "downlink1";
String downlink2 = "downlink2";

void setup() {
  // Define pin modes for TX and RX
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  // Set the baud rate for the SoftwareSerial object
  mySerial.begin(115200);
  Serial.begin(115200);
  Serial1.begin(115200);

  delay(5000);
}
}

void loop() {
  if (mySerial.available() > 0) {                //downlink
    downlink1 = mySerial.readStringUntil('\n');  // 受信した文字列を読み取ります
    downlink1.trim();
    Serial.println(downlink1);
  }

  if (Serial1.available() > 0) {                //GPS
    downlink2 = Serial1.readStringUntil('\n');  // 受信した文字列を読み取ります
    downlink2.trim();
    Serial.println(downlink2);
  }
}