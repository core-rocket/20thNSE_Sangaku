float altitude_m = 0;
float temperature_c = 0;
float humidity_ %= 0;
float pressure_hPa = 0;
float gas_KOhms = 0;
//CAN
#include <CCP_MCP2515.h>
#define CAN0_INT D1
#define CAN0_CS D0
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);
void setup() {
  Serial.begin(115200);
  //CAN
  CCP.begin();
}
void loop() {
  CCP.read_device();
  switch (CCP.id) {
    case CCP_nose_pressure_hPa:
      pressure_hPa = CCP.data_float();
      break;
    case CCP_nose_temperature_C:
      temperature_c = CCP.data_float();  //加速度のZ軸方向の値を代入
      break;
    case CCP_nose_humidity_percent:
      humidity_ % = CCP.data_float();  //高度の値を代入
      break;
    case CCP_nose_altitude_m:
      altitude_m = CCP.data_float();
      break;
    case CCP_nose_gas_KOhms:
      gas_KOhms = CCP.data_float();
      break;
  }
  Serial.print("気圧,");
  Serial.println(pressure_hPa);
  Serial.print("気温,");
  Serial.println(temperature_c);
  Serial.print("湿度,");
  Serial.println(humidity_ %)
    Serial.print("高度,");
  Serial.println(altitude_m);
  Serial.print("ガス,");
  Serial.pritnln(gas_KOhms);
}