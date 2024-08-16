const int _MISO = D9;
const int _MOSI = D10;
const int _CS = D2;
const int _SCK = D8;

#include <SPI.h>
#include <SD.h>
#include <CCP_MCP2515.h>
#include <CCP_W25Q512.h>

#define CAN0_CS D0
#define CAN0_INT D1

// CAN
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);

// Other
char msgString[128];
char str_buf[7];  // 6+\0

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);


  delay(10000);
  Serial.print("Initializing SD card...");

  // Ensure the SPI pinout the SD card is connected to is configured properly
  SPI.setRX(_MISO);
  SPI.setTX(_MOSI);
  SPI.setSCK(_SCK);

  // see if the card is present and can be initialized:
  if (!SD.begin(_CS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void loop() {
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  if (!digitalRead(CAN0_INT))  // データ受信確認
  {
    CCP.read_device();
    if (CCP.id < 0x40) {
      CCP.string(str_buf, 7);
      sprintf(msgString, "%d,ID,%03x,time,%d000,string,%s,,,,", millis(), CCP.id, CCP.time16(), str_buf);
    } else if (CCP.id < 0x80) {
      sprintf(msgString, "%d,ID,%03x,time,%lu,uint32,%lu,,,,", millis(), CCP.id, CCP.time32(), CCP.data_uint32());
    } else if (CCP.id < 0xC0) {
      sprintf(msgString, "%d,ID,%03x,time,%lu,float,%8.2f,,,,", millis(), CCP.id, CCP.time32(), CCP.data_float());
    } else {
      sprintf(msgString, "%d,ID,%03x,time,%d000,fp16_0,%8.2f,fp16_1,%8.2f,fp16_2,%8.2f", millis(), CCP.id, CCP.time16(), CCP.data_fp16_0(), CCP.data_fp16_1(), CCP.data_fp16_2());
    }
  }
  File dataFile = SD.open("data.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(msgString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(msgString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}
