#define CPP_A_pressure_altitude_m 0x080
//CAN
#include <CCP_MCP2515.h>
#define CAN0_INT 2
#define CAN0_CS 3
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

double altitude_median = 0;

// setup()ではdelay()使用可
void setup() {
  Serial.begin(115200);
  //CAN
  Serial.println("start");
  CCP.begin();
  Serial.println("next");
}

// loop()と，ここから呼び出される関数ではdelay()使用禁止
void loop() {
  altitude_median++;
  Serial.print(altitude_median);
  CCP.float_to_device(CPP_A_pressure_altitude_m, altitude_median);
  Serial.println("tranced_to_device");
  delay(1000);
}
