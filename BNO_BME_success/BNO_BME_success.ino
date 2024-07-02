#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BME280.h>

#define Wire_SDA 6
#define Wire_SCL 7
#define PWM_LED_RED 20
#define PWM_LED_BLUE 21

#define bme_I2Cadr 0x77

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

float accel_data[5][3];
float orientation_data[5][3];
float velocity_data[5][3];
float linearaccel_data[5][3];
float magnetometer_data[5][3];
float gravity_data[5][3];

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
// Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
#define SEALEVELPRESSURE_HPA (1013.25)
// #define BME_SCK 13
// #define BME_MISO 12
// #define BME_MOSI 11
// #define BME_CS 10


float get_bme_pressure[10];
float get_bme_temperature[10];
float get_bme_altitude[10];
// float get_bme_humidity[10];
float pressure_median;
float temperature_median;
float altitude_median;
// float humidity_median;

//global
unsigned long time_100Hz = 0;
int count_10Hz = 0;
int count_median_exection = 0;
int dataIndex = 0;

void setup(void) {
  Serial.begin(115200);
  Wire.setPins(Wire_SDA, Wire_SCL);
  Wire.begin();

  // Wire1.begin(Wire1_SDA, Wire1_SCL);
  pinMode(PWM_LED_RED, OUTPUT);
  pinMode(PWM_LED_BLUE, OUTPUT);

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test");
  Serial.println("Hello");
    unsigned status;
  // default settings
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
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
    while (1) delay(10);
  }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();

  /* Initialise the sensor */
  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }


  // bme_humidity->printSensorDetails();
  Serial.print("Hello");
  delay(1000);
}

void loop(void) {
  if (millis() - time_100Hz >= 10) {
    time_100Hz += 10;
    count_10Hz++;

    if (count_10Hz >= 10) {
      dataIndex = count_median_exection % 5;
      count_median_exection++;

      //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
      sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
      bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
      bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

      printEvent(&orientationData, orientation_data, dataIndex);
      printEvent(&angVelocityData, velocity_data, dataIndex);
      printEvent(&linearAccelData, linearaccel_data, dataIndex);
      printEvent(&magnetometerData, magnetometer_data, dataIndex);
      printEvent(&accelerometerData, accel_data, dataIndex);
      printEvent(&gravityData, gravity_data, dataIndex);

      get_bme_pressure[count_10Hz] = bme.readPressure() / 100.0F;
      get_bme_temperature[count_10Hz] = bme.readTemperature();
      get_bme_altitude[count_10Hz] = bme.readAltitude(SEALEVELPRESSURE_HPA);
      // get_bme_humidity[count_10Hz] = humidity_event.relative_humidity;

      uint8_t system, gyro, accel, mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);

      delay(BNO055_SAMPLERATE_DELAY_MS);

      if (count_median_exection % 5 == 0) {
        digitalWrite(PWM_LED_RED, HIGH);
        medianfilter_50Hz_output(accel_data, bno_accel_median);
        medianfilter_50Hz_output(orientation_data, bno_orient_median);
        medianfilter_50Hz_output(velocity_data, bno_velocity_median);
        medianfilter_50Hz_output(linearaccel_data, bno_linearaccel_median);
        medianfilter_50Hz_output(magnetometer_data, bno_magnet_median);
        medianfilter_50Hz_output(gravity_data, bno_gravity_median);
        //CAN_to_device

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
        Serial.println("---------------------------------");
        digitalWrite(PWM_LED_RED, LOW);
      }
      if (count_median_exection % 10 == 0) {

        pressure_median = findMedian(get_bme_pressure, 10);
        temperature_median = findMedian(get_bme_temperature, 10);
        altitude_median = findMedian(get_bme_altitude, 10);
        // humidity_median = findMedian(get_bme_humidity, 10);
        Serial.print("気圧：");
        Serial.println(pressure_median);
        Serial.print("気温：");
        Serial.println(temperature_median);
        Serial.print("高度：");
        Serial.println(altitude_median);
        printValues();
        // Serial.print("湿度：");
        // Serial.println(humidity_median);
      }
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


void medianfilter_50Hz_output(float dataholder[5][3], float datamedian[3]) {
  float temp[5];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 5; j++) {
      temp[j] = dataholder[j][i];
    }
    datamedian[i] = findMedian(temp, 5);
  }
}

float findMedian(float arr[], int n) {
  // 一時的な配列をソート
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
void printEvent(sensors_event_t *event, float dataholder[5][3], int count) {
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
void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}
