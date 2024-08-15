#include <Adafruit_LPS35HW.h>

// SPIピンの定義
#define LPS_CS   10
#define LPS_SCK  13
#define LPS_MISO 12
#define LPS_MOSI 11

Adafruit_LPS35HW lps35hw = Adafruit_LPS35HW();

void setup() {
  Serial.begin(115200);
  // シリアルポートの準備ができるまで待機
  while (!Serial) { delay(1); }

  Serial.println("Adafruit LPS35HW SPI Test");

  // SPIモードでの初期化
  if (!lps35hw.begin_SPI(LPS_CS, LPS_SCK, LPS_MISO, LPS_MOSI)) {
    Serial.println("Couldn't find LPS35HW chip");
    while (1);
  }
  Serial.println("Found LPS35HW chip");
}

void loop() {
  // 圧力データと温度データを取得
  float temperature = lps35hw.readTemperature();
  float pressure = lps35hw.readPressure();

  // データをシリアルモニタに出力
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" hPa");

  // 1秒（1000ミリ秒）待機
  delay(1000);
}


