#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define bme_I2Cadr 0x77
//key
const int voltagePin = A0;           // アナログ入力ピンを指定
const float thresholdVoltage = 2.5;  // しきい値電圧 (例: 2.5V)
//LED
#define PWM_LED_RED 20   //GPIO20を使用する
#define PWM_LED_BLUE 21  //GPIO21を使用する
// digitalWrite(PIN_LED, HIGH);
// digitalWrite(PIN_LED, LOW);

//CAN
#include <CCP_MCP2515.h>
#define CAN0_INT D1
#define CAN0_CS D0
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

//BME280
Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
#define SEALEVELPRESSURE_HPA (1013.25)
float get_bme_pressure[10];
float get_bme_temperature[10];
float get_bme_humidity[10];
float get_bme_altitude[10];
float pressure_median;
float temperature_median;
float humidity_median;
float altitude_median;

//BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float accel_data[10][3];  //x,y,z軸方向のデータを10回格納するために[10][3]
float orientation_data[10][3];
float velocity_data[10][3];
float linearaccel_data[10][3];
float magnetometer_data[10][3];
float gravity_data[10][3];

float bno_accel_median[3];  //x,y,z軸方向の中央値データを格納
float bno_orient_median[3];
float bno_velocity_median[3];
float bno_linearaccel_median[3];
float bno_magnet_median[3];
float bno_gravity_median[3];

//
unsigned long time_100Hz = 0;
int count_10Hz = 0;

// setup()ではdelay()使用可
void setup(void) {
  Serial.begin(115200);
  Wire.begin();
  CCP.begin();

  pinMode(PWM_LED_RED, OUTPUT);
  pinMode(PWM_LED_BLUE, OUTPUT);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test");
  Serial.println("Hello");
  unsigned status;

  // BME280の初期化
  if (!bme.begin(bme_I2Cadr)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    while (1) delay(10);
  }
  Serial.println("-- Default Test --");
  Serial.println(F("BME280 Sensor event test"));

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  Serial.print("Hello");
}

// loop()と，ここから呼び出される関数ではdelay()使用禁止
void loop() {
  if (millis() - time_100Hz >= 10) {
    time_100Hz += 10;
    count_10Hz++;  //100Hz処理されたときに+1
    //100Hzで実行する処理
    sensors_event_t temp_event, pressure_event, humidity_event;
    // bme_temp->getEvent(&temp_event);
    // bme_pressure->getEvent(&pressure_event);
    // bme_humidity->getEvent(&humidity_event);
    get_bme_pressure[count_10Hz] = bme.readPressure() / 100.0F;  //bmeの生データを配列に格納
    get_bme_temperature[count_10Hz] = bme.readTemperature();
    get_bme_humidity[count_10Hz] = bme.readHumidity();
    get_bme_altitude[count_10Hz] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    //BNO055
    sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    printEvent(&orientationData, orientation_data, count_10Hz);
    printEvent(&angVelocityData, velocity_data, count_10Hz);
    printEvent(&linearAccelData, linearaccel_data, count_10Hz);
    printEvent(&magnetometerData, magnetometer_data, count_10Hz);
    printEvent(&accelerometerData, accel_data, count_10Hz);
    printEvent(&gravityData, gravity_data, count_10Hz);

    //キャリブレーション(センサーや計測機器が正確に測定できるように調整するプロセス)の実行
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    if (count_10Hz >= 9) {
      count_10Hz = 0;
      // 10Hzで実行する処理
      //LED_debug
      digitalWrite(PWM_LED_RED, HIGH);
      //BNOの中央値計算10Hz処理
      medianfilter_xyz(accel_data, bno_accel_median);
      medianfilter_xyz(orientation_data, bno_orient_median);
      medianfilter_xyz(velocity_data, bno_velocity_median);
      medianfilter_xyz(linearaccel_data, bno_linearaccel_median);
      medianfilter_xyz(magnetometer_data, bno_magnet_median);
      medianfilter_xyz(gravity_data, bno_gravity_median);
      //BMEの中央値計算10Hz処理
      pressure_median = medianfilter_10Hz(get_bme_pressure);
      temperature_median = medianfilter_10Hz(get_bme_temperature);
      humidity_median = medianfilter_10Hz(get_bme_humidity);
      altitude_median = medianfilter_10Hz(get_bme_altitude);
      //debug
      Serial.print("加速度");
      Serial.println(bno_accel_median[2]);
      Serial.print("高度");
      Serial.println(altitude_median);
      Serial.print("温度");
      Serial.println(temperature_median);
      //CAN(sensor_to_flush/opener)
      CCP.fp16_to_device(CCP_A_accel_mss, bno_accel_median[0], bno_accel_median[1], bno_accel_median[2]);
      CCP.fp16_to_device(CCP_A_gyro_rads, bno_orient_median[0], bno_orient_median[1], bno_orient_median[2]);
      CCP.fp16_to_device(CCP_A_mag_uT, bno_velocity_median[0], bno_velocity_median[1], bno_velocity_median[2]);
      CCP.fp16_to_device(CCP_A_euler_rad, bno_linearaccel_median[0], bno_linearaccel_median[1], bno_linearaccel_median[2]);
      CCP.fp16_to_device(CCP_A_magnetic_Am, bno_magnet_median[0], bno_magnet_median[1], bno_magnet_median[2]);
      CCP.fp16_to_device(CCP_A_gravity_mss, bno_gravity_median[0], bno_gravity_median[1], bno_gravity_median[2]);
      CCP.float_to_device(CCP_A_pressure_hPa, pressure_median);
      CCP.float_to_device(CCP_A_temperature_C, temperature_median);
      CCP.float_to_device(CCP_A_humidity_percent, humidity_median);
      CCP.float_to_device(CCP_A_altitude_m, altitude_median);
      digitalWrite(PWM_LED_RED, LOW);
    }
  }
  // 常に実行する処理
  int sensorValue = analogRead(voltagePin);      // アナログ入力から値を読み取る
  float voltage = sensorValue * (5.0 / 1023.0);  // センサ値を電圧に変換（5V基準）
  if (voltage >= thresholdVoltage) {             //参考に
    // CCP.char_to_device(CP_opener_control,"READY");
  } else {
  }
}

void medianfilter_xyz(float dataholder[10][3], float datamedian[3]) {
  float temp[10];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 10; j++) {
      temp[j] = dataholder[j][i];
    }
    datamedian[i] = medianfilter_10Hz(temp);
  }
}

//10Hz_output_中央値計算
float medianfilter_10Hz(float arr[]) {
  // 一時的な配列をソート
  int n = 10;
  sortArray(arr, n);
  // 中央値を返す
  if (n % 2 == 0) {
    return (arr[n / 2 - 1] + arr[n / 2]) / 2.0;
  } else {
    return arr[n / 2];
  }
}

void sortArray(float arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        // Swap arr[j] and arr[j + 1]
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

//get_bno055data
void printEvent(sensors_event_t *event, float dataholder[][3], int count) {
  double x = -1000000, y = -1000000, z = -1000000;

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {  //加速度
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {  //方位
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {  //磁場
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {  //回転儀
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {  //角速度
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {  //直線加速度
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_GRAVITY) {  //重力
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    Serial.print("Unk:");
  }
  dataholder[count][0] = x;
  dataholder[count][1] = y;
  dataholder[count][2] = z;
}