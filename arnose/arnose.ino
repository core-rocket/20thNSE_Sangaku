#include <Arduino.h>

const int pressurePin = 1; // Analog pin for pressure signal
// const int oscillationPin = A1; // Analog pin for oscillation signal

const float maxPressure_kPa = 400.0; // Maximum pressure in kPa
const float sensitivity_mv_per_kPa = 45.0; // Sensitivity in mV/kPa

const unsigned int maxSystolicPressure = 240; // Maximum systolic pressure in mmHg
const unsigned int maxDiastolicPressure = 170; // Maximum diastolic pressure in mmHg

volatile unsigned int pressureRaw; // Raw pressure signal value
volatile unsigned int oscillationRaw; // Raw oscillation signal value
volatile float pressure_kPa; // Pressure in kPa
volatile unsigned int BPsys; // Systolic pressure
volatile unsigned int BPdia; // Diastolic pressure

// Conversion functions
float adc2pressure(unsigned int adc_val) {
  return ((adc_val / 4095.0) * maxPressure_kPa); // Convert ADC value to pressure in kPa
}

unsigned int setSys(float pressure_kPa) {
  // Convert pressure to systolic pressure value using your formula
  return (unsigned int)((pressure_kPa / maxPressure_kPa) * maxSystolicPressure);
}

unsigned int setDia(float pressure_kPa) {
  // Convert pressure to diastolic pressure value using your formula
  return (unsigned int)((pressure_kPa / maxPressure_kPa) * maxDiastolicPressure);
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Simulating raw pressure and oscillation values, replace with actual sensor readings
  pressureRaw = analogRead(pressurePin);
  // oscillationRaw = analogRead(oscillationPin);

  // Convert raw pressure ADC value to pressure in kPa
  pressure_kPa = adc2pressure(pressureRaw);

  // Calculate systolic and diastolic pressures
  BPsys = setSys(pressure_kPa);
  BPdia = setDia(pressure_kPa);

  // Print results
  Serial.print("Raw Pressure: ");
  Serial.println(pressureRaw);
  Serial.print("Pressure (kPa): ");
  Serial.println(pressure_kPa, 2); // Print with 2 decimal places
  Serial.print("Systolic Pressure: ");
  Serial.println(BPsys);
  Serial.print("Diastolic Pressure: ");
  Serial.println(BPdia);

  // Print oscillation signal value
  // Serial.print("Oscillation Signal: ");
  // Serial.println(oscillationRaw);

  delay(1000); // Add a delay between measurements 
}