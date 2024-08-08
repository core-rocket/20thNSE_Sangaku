#include <Wire.h>  //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include <MicroNMEA.h>  //http://librarymanager/All#MicroNMEA
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

//CANに必要なもの
#include <SPI.h>
#include <string.h>
#include <CCP_MCP2515.h>

#define CAN0_INT 1
#define CAN0_CS 0

CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

void setup() {
  Serial.begin(115200);
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false) {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);  //Set the I2C port to output both NMEA and UBX messages
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  //Save (only) the communications port settings to flash and BBR

  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_ALL);  // Make sure the library is passing all NMEA messages to processNMEA

  myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA);  // Or, we can be kind to MicroNMEA and _only_ pass the GGA messages to it

  // CAN
  pinMode(CAN0_CS, OUTPUT);
  pinMode(CAN0_INT, INPUT);
  digitalWrite(CAN0_CS, HIGH);
  CCP.begin();
}

void loop() {
  myGNSS.checkUblox();  //See if new data is available. Process bytes as they come in.

  if (nmea.isValid() == true) {
    long latitude_mdeg = nmea.getLatitude();
    long longitude_mdeg = nmea.getLongitude();

    Serial.print("Latitude (deg): ");
    Serial.println(latitude_mdeg / 1000000., 6);
    Serial.print("Longitude (deg): ");
    Serial.println(longitude_mdeg / 1000000., 6);

    CCP.uint32_to_device(CCP_A_GNSS_latitude_udeg, nmea.getLatitude() / 1000000);
    CCP.uint32_to_device(CCP_A_GNSS_longitude_udeg, nmea.getLongitude() / 1000000);

    nmea.clear();  // Clear the MicroNMEA storage to make sure we are getting fresh data
  } else {
    Serial.println("Waiting for fresh data");
  }

  delay(1000);  //Don't pound too hard on the I2C bus
}

//This function gets called from the SparkFun u-blox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GNSS::processNMEA(char incoming) {
  //Take the incoming char from the u-blox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}
