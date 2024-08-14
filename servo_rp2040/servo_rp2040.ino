#include <Servo.h>

Servo myservo1;
Servo myservo2;
const int SV_PIN_1 = D4;
const int SV_PIN_2 = D5;

void setup() {
  Serial.begin(9600); // シリアル通信を9600bpsで開始
  myservo1.attach(SV_PIN_1, 500, 2400);
  myservo2.attach(SV_PIN_2, 500, 2400);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // シリアルモニタからの入力を読み取る
    input.trim(); // 入力の前後の空白を削除

    Serial.print("Received input: "); // デバッグメッセージ
    Serial.println(input);

    if (input == "open") {
      Serial.println("Opening..."); // デバッグメッセージ
      myservo1.write(90);  // サーボモーター1を90度の位置まで動かす
      myservo2.write(90);  // サーボモーター2を90度の位置まで動かす
    } else if (input == "close") {
      Serial.println("Closing..."); // デバッグメッセージ
      myservo1.write(180);  // サーボモーター1を180度の位置まで動かす
      myservo2.write(0);  // サーボモーター2を0度の位置まで動かす
    } else {
      Serial.println("Unknown command"); // 不明なコマンドの場合のデバッグメッセージ
    }
  }
}