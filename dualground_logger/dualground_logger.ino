#include <string.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#define OLED_ADRS 0x3D  //SA0=L(SA0=H の場合は 0x3D)

#define rxPin D2
#define txPin D1

const int _MISO = 4;
const int _MOSI = 7;
const int _CS = 5;
const int _SCK = 6;

// Set up a new SoftwareSerial object
SoftwareSerial mySerial = SoftwareSerial(rxPin, txPin);

int DisplayON = 0x0F,
    ClearDisplay = 0x01,
    ReturnHome = 0x02;

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

  Wire.begin();  //Wire ﾗｲﾌﾞﾗﾘを初期化し、I2C ﾏｽﾀとしてﾊﾞｽに接続
  init_oled();

  Serial.print("Initializing SD card...");

  // Ensure the SPI pinout the SD card is connected to is configured properly
  SPI.setRX(_MISO);
  SPI.setTX(_MOSI);
  SPI.setSCK(_SCK);

  delay(5000);

  // see if the card is present and can be initialized:
  if (!SD.begin(_CS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  char moji[] = "START";
  for (int i = 0; i < 20; i++) {
    Serial.print(moji[i]);
    writeData(moji[i]);
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

  char downlink1_array[] = downlink1;
  char downlink2_array[] = downlink2;

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(downlink1);
    dataFile.print(":");
    dataFile.println(downlink2);
    dataFile.close();
    // print to the serial port too:
    Serial.println(downlink1);
    Serial.println(":");
    Serial.println(downlink2);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }

  for (int i = 0; i < 20; i++) {
    Serial.print(downlink1_array[i]);
    writeData(downlink1_array[i]);
  }

  writeCommand(0x20 + 0x80);  //2 行目の先頭

  for (int i = 0; i < 20; i++) {
    Serial.print(downlink2_array[i]);
    writeData(downlink2_array[i]);
  }

  contrast_max();  //輝度を最大に設定
  while (1) {}
}

//----main end----
void writeData(byte t_data) {
  Wire.beginTransmission(OLED_ADRS);
  Wire.write(0x40);
  Wire.write(t_data);
  Wire.endTransmission();
  delay(1);
}
void writeCommand(byte t_command) {
  Wire.beginTransmission(OLED_ADRS);
  Wire.write(0x00);
  Wire.write(t_command);
  Wire.endTransmission();
  delay(10);
}
void contrast_max() {
  writeCommand(0x2a);  //RE=1
  writeCommand(0x79);  //SD=1
  writeCommand(0x81);  //コントラストセット
  writeCommand(0xFF);  //輝度ＭＡＸ
  writeCommand(0x78);  //SD を０にもどす
  writeCommand(0x28);  //2C=高文字　28=ノーマル
  delay(100);
}
void init_oled() {
  delay(100);
  writeCommand(ClearDisplay);  // Clear Display
  delay(20);
  writeCommand(ReturnHome);  // ReturnHome
  delay(2);
  writeCommand(DisplayON);  // Send Display on command
  delay(2);
  writeCommand(ClearDisplay);  // Clear Display
  delay(20);
}
