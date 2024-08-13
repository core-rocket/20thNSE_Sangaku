void setup() {
  // initialize serial communication at 9600 bps
  Serial.begin(9600);

}

void loop() {
  // read the sensor input on analog pin 1
  int sensorValue = analogRead(A1);
  // Convert the analog reading of A1 from 0-1023 to a voltage 0-5V
  float voltage = sensorValue * (5.0 / 1023.0);
  float outputkPa = fmap(voltage, 0.2, 4.7, 0, 10);
  float outputmmH2O = fmap(voltage, 0.2, 4.7, 0, 1019.78);
  // print out the values on serial monitor
  Serial.print("Volt: ");
  Serial.println(voltage);
  Serial.print("kPa: ");
  Serial.println(outputkPa);
  Serial.print("mmH2O: ");
  Serial.println(outputmmH2O);
  Serial.println();
  delay(200);// put your main code here, to run repeatedly:
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
