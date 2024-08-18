#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BME280.h>

#define Wire_SDA 6
#define Wire_SCL 7
#define PWM_LED_BLUE 0   //値取得・送信確認用&エラー確認用
#define PWM_LED_WHITE 1  //エラー確認用

#define bme_I2Cadr 0x77

//CAN
#include <CCP_MCP2515.h>
#define CAN0_INT D1
#define CAN0_CS D0
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

//デバッグ
#define debug

// id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float accel_data[10][3];
float orientation_data[10][3];
float velocity_data[10][3];
float linearaccel_data[10][3];
float magnetometer_data[10][3];
float gravity_data[10][3];

float bno_accel_median[3];
float bno_orient_median[3];
float bno_velocity_median[3];
float bno_linearaccel_median[3];
float bno_magnet_median[3];
float bno_gravity_median[3];

//BME280
Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
#define SEALEVELPRESSURE_HPA (1013.25)
float sealevelpressure = 1013.25;
float tmp = 0;
float difference;
float difference_2;
float get_bme_pressure[10];
float get_bme_temperature[10];
float get_bme_altitude[10];
float get_bme_humidity[10];
float pressure_median;
float temperature_median;
float altitude_median;
float humidity_median;

//global
unsigned long time_100Hz = 0;
int count_10Hz = 0;
int count_1Hz = 0;

void setup(void) {
  Serial.begin(115200);
  Wire.begin();
  CCP.begin();
  pinMode(PWM_LED_BLUE, OUTPUT);
  pinMode(PWM_LED_WHITE, OUTPUT);

  unsigned status;
  // default settings
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    digitalWrite(PWM_LED_BLUE, HIGH);  //BMEI2C_error
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x");
    Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  Serial.println("-- Default Test --");
  // BME280の初期化
  Serial.println(F("BME280 Sensor event test"));
  if (!bme.begin(bme_I2Cadr)) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    digitalWrite(PWM_LED_WHITE, HIGH);  //BMEI2C_error
    while (1) delay(10);
  }
  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    digitalWrite(PWM_LED_WHITE, HIGH);  //BNOI2C_error
    while (1)
      ;
  }
  delay(2000);
  digitalWrite(PWM_LED_WHITE, LOW);
  digitalWrite(PWM_LED_BLUE, LOW);
}

void loop(void) {
  if (millis() - time_100Hz >= 10) {
    //100Hz処理
    time_100Hz += 10;
    count_10Hz++;
    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
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

    get_bme_pressure[count_10Hz] = bme.readPressure() / 100.0F;
    get_bme_temperature[count_10Hz] = bme.readTemperature();
    get_bme_altitude[count_10Hz] = bme.readAltitude(SEALEVELPRESSURE_HPA);
    get_bme_humidity[count_10Hz] = bme.readHumidity();

    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    if (count_10Hz >= 9) {
      //10Hz処理
      switch (count_1Hz) {
        case 4:
          digitalWrite(PWM_LED_BLUE, LOW);  //青LED消灯
          break;
        case 9:
          digitalWrite(PWM_LED_BLUE, HIGH);  //青LED点灯
          count_1Hz = 0;
          break;
      }
      digitalWrite(PWM_LED_WHITE, HIGH);  //白LED点灯
      medianfilter_10Hz_output(accel_data, bno_accel_median);
      medianfilter_10Hz_output(orientation_data, bno_orient_median);
      medianfilter_10Hz_output(velocity_data, bno_velocity_median);
      medianfilter_10Hz_output(linearaccel_data, bno_linearaccel_median);
      medianfilter_10Hz_output(magnetometer_data, bno_magnet_median);
      medianfilter_10Hz_output(gravity_data, bno_gravity_median);
      digitalWrite(PWM_LED_WHITE, LOW);  //白LED点灯

      //CAN_to_device
      CCP.fp16_to_device(CCP_B_accel_mss, bno_accel_median[0], bno_accel_median[1], bno_accel_median[2]);
      CCP.fp16_to_device(CCP_B_gyro_rads, bno_orient_median[0], bno_orient_median[1], bno_orient_median[2]);
      CCP.fp16_to_device(CCP_B_mag_uT, bno_velocity_median[0], bno_velocity_median[1], bno_velocity_median[2]);
      CCP.fp16_to_device(CCP_B_euler_rad, bno_linearaccel_median[0], bno_linearaccel_median[1], bno_linearaccel_median[2]);
      CCP.fp16_to_device(CCP_B_magnetic_Am, bno_magnet_median[0], bno_magnet_median[1], bno_magnet_median[2]);
      CCP.fp16_to_device(CCP_B_gravity_mss, bno_gravity_median[0], bno_gravity_median[1], bno_gravity_median[2]);

      pressure_median = findMedian(get_bme_pressure, 10);
      temperature_median = findMedian(get_bme_temperature, 10);
      humidity_median = findMedian(get_bme_humidity, 10);

      //CAN to device
      CCP.float_to_device(CCP_B_pressure_hPa, pressure_median);
      CCP.float_to_device(CCP_B_temperature_C, temperature_median);
      CCP.float_to_device(CCP_B_humidity_percent, humidity_median);
      CCP.float_to_device(CCP_B_altitude_m, difference);

#ifdef debug
      tmp = altitude_median;
#endif
      altitude_median = findMedian(get_bme_altitude, 10);
#ifdef debug
      difference = altitude_median - tmp;
      Serial.print("差, ");
      Serial.println(difference);

      Serial.print("accel");
      print_data(bno_accel_median);
      Serial.print("orient");
      print_data(bno_orient_median);
      Serial.print("velocity");
      print_data(bno_velocity_median);
      Serial.print("linearaccel");
      print_data(bno_linearaccel_median);
      Serial.print("magnet");
      print_data(bno_magnet_median);
      Serial.print("gravity");
      print_data(bno_gravity_median);

      Serial.print("気圧：");
      Serial.println(pressure_median);
      Serial.print("気温：");
      Serial.println(temperature_median);
      Serial.println("---------------------------------");
      Serial.print("高度　, ");
      Serial.println(altitude_median);
      Serial.print("差 ,");
      Serial.println(difference);
      Serial.print("湿度：");
      Serial.println(humidity_median);
#endif
      count_10Hz = 0;
    }
  }
}


void print_data(float datamedian[3]) {
  for (int i = 0; i < 3; i++) {
    if (i == 0) {
      Serial.print("x:");
    } else if (i == 1) {
      Serial.print("y:");
    } else {
      Serial.print("z:");
    }
    Serial.print(datamedian[i]);
    Serial.print(", ");
  }
  Serial.println();
}


void medianfilter_10Hz_output(float dataholder[10][3], float datamedian[3]) {
  float temp[10];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 10; j++) {
      temp[j] = dataholder[j][i];
    }
    datamedian[i] = findMedian(temp, 10);
  }
}

float findMedian(float arr[], int n) {
  // 一時的な配列をソート
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

  // 中央値を返す
  if (n % 2 == 0) {
    return (arr[n / 2 - 1] + arr[n / 2]) / 2.0;
  } else {
    return arr[n / 2];
  }
}



//get_bno055data
void printEvent(sensors_event_t *event, float dataholder[10][3], int count) {
  double x = -1000000, y = -1000000, z = -1000000;

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_GRAVITY) {
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
