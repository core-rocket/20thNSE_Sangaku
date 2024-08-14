/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#include <CCP_MCP2515.h>
#define CAN0_INT D1
#define CAN0_CS D0

// CAN
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

float altitude_m = 0;
float temperature_c = 0;
float humidity_per= 0;
float pressure_hPa = 0;
float gas_KOhms=0;

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme;  // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);

void setup() {
  Serial.begin(115200);
  // CAN
  CCP.begin();
  Wire.begin();
  while (!Serial)
    ;
  Serial.println(F("BME680 test"));
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1)
      ;
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms
}

void loop() {
  if (!bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  temperature_c = bme.temperature;
  Serial.print(temperature_c);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  pressure_hPa=bme.pressure/100.0;
  Serial.print(pressure_hPa);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
 humidity_per= bme.humidity;
  Serial.print(humidity_per);
  Serial.println(" %");

  Serial.print("Gas = ");
  gas_KOhms=bme.gas_resistance / 1000.0;
  Serial.print(gas_KOhms);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  altitude_m = bme.readAltitude(SEALEVELPRESSURE_HPA);
  Serial.print(altitude_m);
  Serial.println(" m");

  CCP.float_to_device(CCP_nose_pressure_hPa, pressure_hPa);
  CCP.float_to_device(CCP_nose_temperature_C, temperature_c);
  CCP.float_to_device(CCP_nose_humidity_percent, humidity_per);
  CCP.float_to_device(CCP_nose_altitude_m, altitude_m);
  CCP.float_to_device(CCP_nose_gas_KOhms, gas_KOhms);

  Serial.println();
}