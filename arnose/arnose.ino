float IN_min = 0.2;
float IN_max = 4.7;
float OUT_min = 0;
float voltage = 0;
float outputkPa = 0;
float outputmmH2O = 0;

int sensorValue = 0;

void setup() {
  // initialize serial communication at 9600 bps
  Serial.begin(9600);
}

void loop() {
  // read the sensor input on analog pin 1
  sensorValue = analogRead(A7); // XIAO_SAMD
  // Convert the analog reading of A7 from 0-1023 to a voltage 0-3.3V
  voltage = sensorValue * (3.3 / 1023.0);
  outputkPa = fmap(voltage, IN_min, IN_max, OUT_min, 10);
  outputmmH2O = fmap(voltage, IN_min, IN_max, OUT_min, 1019.78);
  // print out the values on serial monitor
  Serial.print("Volt: ");
  Serial.println(voltage);
  Serial.print("kPa: ");
  Serial.println(outputkPa);
  Serial.print("mmH2O: ");
  Serial.println(outputmmH2O);
  Serial.println();
  delay(200);  // put your main code here, to run repeatedly:
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
