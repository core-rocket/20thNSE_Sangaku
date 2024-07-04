#define CPP_A_pressure_altitude_m 0x080
//CAN
#include <CCP_MCP2515.h>
#define CAN0_INT 2
#define CAN0_CS 3
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

String uplink = "";
float altitudedata_m; // Declare the variable to store altitude data

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
  CCP.read_device();
  switch (CCP.id) {
    case CPP_A_pressure_altitude_m:
       altitudedata_m = CCP.data_float(); //高度の値を代入
       Serial.println(altitudedata_m);
      break;
  }
  if (Serial.available()) {
    uplink = Serial.readStringUntil('\n');
    if (uplink == "CHECK") {
      Serial.println("CHECK_to_transition");
    } else if (uplink == "READY") {
      Serial.println("READY_to_transition");
    }
  }
}
