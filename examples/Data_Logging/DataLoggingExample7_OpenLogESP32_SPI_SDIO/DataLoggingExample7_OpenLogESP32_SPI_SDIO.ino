/*
  Configuring the GNSS to automatically send RXM SFRBX and RAWX reports over SPI and log them to file on SD card using full 4-bit SDIO
  By: Paul Clark
  SparkFun Electronics
  Date: October 20th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the u-blox GNSS to send RXM SFRBX and RAWX reports automatically
  and log the data to SD card in UBX format.
  
  This code is written for the OpenLog ESP32 (DEV-20594) - coming soon!

  Hardware set-up:
  Close the DSEL jumper on the ZED-F9P breakout - to select SPI mode
  Connect:
  OpenLog ESP32 : ZED-F9P
  GND             GND
  3V3_SW          3V3
  SCK  (18)       SCK
  PICO (23)       PICO (MOSI)
  POCI (19)       POCI (MISO)
  33              CS

  ** Please note: this example will only work on u-blox ADR or High Precision GNSS or Time Sync products **

  Data is logged in u-blox UBX format.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  OpenLog ESP32: https://www.sparkfun.com/products/20594
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
*/

#define GNSS_CS 33   // Connect the ZED-F9P CS pin to OpenLog ESP32 pin 33
#define EN_3V3_SW 32 // The 3.3V_SW regulator Enable pin is connected to D32
#define STAT_LED 25  // The OpenLog ESP32 STAT LED is connected to pin 25
#define IMU_CS 5     // The ISM330 IMU CS is connected to pin 5
#define MAG_CS 27    // The MMC5983 Mag CS is connected to pin 27

#include "FS.h"
#include "SD_MMC.h"
File myFile;

#include <SPI.h>
#define spiPort SPI

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#define sdWriteSize 512 // Write data to the SD card in blocks of 512 bytes
#define fileBufferSize 16384 // Allocate 16KBytes of RAM for UBX message storage
uint8_t *myBuffer; // Use myBuffer to hold the data while we write it to SD card

unsigned long lastPrint; // Record when the last Serial print took place

// Note: we'll keep a count of how many SFRBX and RAWX messages arrive - but the count will not be completely accurate.
// If two or more SFRBX messages arrive together as a group and are processed by one call to checkUblox, the count will
// only increase by one.

int numSFRBX = 0; // Keep count of how many SFRBX message groups have been received (see note above)
int numRAWX = 0; // Keep count of how many RAWX message groups have been received (see note above)

// Callback: newSFRBX will be called when new RXM SFRBX data arrives
// See u-blox_structs.h for the full definition of UBX_RXMSFRBX_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoRXMSFRBXcallback
//        /                  _____  This _must_ be UBX_RXM_SFRBX_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void newSFRBX(UBX_RXM_SFRBX_data_t *ubxDataStruct)
{
  numSFRBX++; // Increment the count
}

// Callback: newRAWX will be called when new RXM RAWX data arrives
// See u-blox_structs.h for the full definition of UBX_RXMRAWX_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoRXMRAWXcallback
//        /             _____  This _must_ be UBX_RXM_RAWX_data_t
//        |            /                _____ You can use any name you like for the struct
//        |            |               /
//        |            |               |
void newRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct)
{
  numRAWX++; // Increment the count
}

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  Serial.println(F("SparkFun OpenLog ESP32 GNSS Logging : SPI and SDIO"));

  pinMode(STAT_LED, OUTPUT); // Flash the STAT LED each time we write to the SD card
  digitalWrite(STAT_LED, LOW);

  pinMode(GNSS_CS, OUTPUT);
  digitalWrite(GNSS_CS, HIGH);
  pinMode(IMU_CS, OUTPUT);
  digitalWrite(IMU_CS, HIGH);
  pinMode(MAG_CS, OUTPUT);
  digitalWrite(MAG_CS, HIGH);

  pinMode(EN_3V3_SW, OUTPUT); // Enable power for the microSD card and GNSS
  digitalWrite(EN_3V3_SW, HIGH);
  
  delay(1000); // Allow time for the SD card to start up

  spiPort.begin();

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Initialize the GNSS. Wait for a 3D fix. Get the date and time for the log file

  Serial.println(F("Initializing the GNSS..."));

  //myGNSS.enableDebugging(); // Uncomment this line to see helpful debug messages on Serial

  myGNSS.setFileBufferSize(fileBufferSize); // setFileBufferSize must be called _before_ .begin

  // Connect to the u-blox module using SPI port, csPin and speed setting
  // ublox devices generally work up to 5MHz. We'll use 4MHz for this example:
  bool begun = false;
  do
  {
    begun = myGNSS.begin(spiPort, GNSS_CS, 4000000);
    if (!begun)
    {
      Serial.println(F("u-blox GNSS not detected on SPI bus. Please check wiring."));
      delay(1000);
    }
  }
  while (!begun);
  
  //myGNSS.factoryDefault(); delay(5000); // Uncomment this line to reset the module back to its factory defaults

  myGNSS.setSPIOutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the SPI port to output both UBX and NMEA messages

  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Optional: save (only) the communications port settings to flash and BBR

  Serial.print(F("Waiting for a 3D fix"));

  uint8_t fix = 0;

  do
  {
    fix = myGNSS.getFixType();
    delay(1000);
    Serial.print(F("."));
  }
  while ( fix != 3 ); // Wait for a 3D fix

  Serial.println();

  uint16_t y = myGNSS.getYear();
  uint8_t M = myGNSS.getMonth();
  uint8_t d = myGNSS.getDay();
  uint8_t h = myGNSS.getHour();
  uint8_t m = myGNSS.getMinute();
  uint8_t s = myGNSS.getSecond();

  char szBuffer[40] = {'\0'};
  snprintf(szBuffer, sizeof(szBuffer), "/%04d%02d%02d%02d%02d%02d.ubx", y, M, d, h, m, s);

  Serial.println(F("GNSS initialized."));

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Initialize the SD card. Open the log file

  Serial.println(F("Initializing SD card..."));

  // Begin the SD card
  if(!SD_MMC.begin())
  {
    Serial.println(F("Card mount failed. Freezing..."));
    while(1);
  }

  // Open the log file for writing
  Serial.printf("Log file is: %s\r\n", szBuffer);
  myFile = SD_MMC.open((const char *)szBuffer, FILE_WRITE);
  if(!myFile)
  {
    Serial.println(F("Failed to open log file for writing. Freezing..."));
    while(1);
  }

  Serial.println(F("SD card initialized."));

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Wait for a key press

  while (Serial.available()) // Make sure the Serial buffer is empty
  {
    Serial.read();
  }

  Serial.println(F("Press any key to start logging."));

  while (!Serial.available()) // Wait for the user to press a key
  {
    ; // Do nothing
  }

  delay(100); // Wait, just in case multiple characters were sent

  while (Serial.available()) // Empty the Serial buffer
  {
    Serial.read();
  }

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Enable RAWX and SFRBX

  myGNSS.setNavigationFrequency(4); // Set navigation rate to 4Hz

  myGNSS.setAutoRXMSFRBXcallbackPtr(&newSFRBX); // Enable automatic RXM SFRBX messages with callback to newSFRBX

  myGNSS.logRXMSFRBX(); // Enable RXM SFRBX data logging

  myGNSS.setAutoRXMRAWXcallbackPtr(&newRAWX); // Enable automatic RXM RAWX messages with callback to newRAWX

  myGNSS.logRXMRAWX(); // Enable RXM RAWX data logging

  myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_SPI, 1); // Ensure the GxGGA (Global positioning system fix data) message is enabled. Send every measurement.
  myGNSS.enableNMEAMessage(UBX_NMEA_GSA, COM_PORT_SPI, 1); // Ensure the GxGSA (GNSS DOP and Active satellites) message is enabled. Send every measurement.
  myGNSS.enableNMEAMessage(UBX_NMEA_GSV, COM_PORT_SPI, 1); // Ensure the GxGSV (GNSS satellites in view) message is enabled. Send every measurement.

  myGNSS.setNMEALoggingMask(SFE_UBLOX_FILTER_NMEA_ALL); // Enable logging of all enabled NMEA messages

  myBuffer = new uint8_t[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card  

  Serial.println(F("Press any key to stop logging."));

  lastPrint = millis(); // Initialize lastPrint
}

void loop()
{
  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  while (myGNSS.fileBufferAvailable() >= sdWriteSize) // Check to see if we have at least sdWriteSize waiting in the buffer
  {
    digitalWrite(STAT_LED, HIGH); // Flash the STAT LED each time we write to the SD card

    myGNSS.extractFileBufferData(myBuffer, sdWriteSize); // Extract exactly sdWriteSize bytes from the UBX file buffer and put them into myBuffer

    myFile.write(myBuffer, sdWriteSize); // Write exactly sdWriteSize bytes from myBuffer to the ubxDataFile on the SD card

    // In case the SD writing is slow or there is a lot of data to write, keep checking for the arrival of new data
    myGNSS.checkUblox(); // Check for the arrival of new data and process it.
    myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

    digitalWrite(STAT_LED, LOW); // Turn the STAT LED off again
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  if (millis() > (lastPrint + 1000)) // Print the message count once per second
  {
    Serial.print(F("Number of message groups received: SFRBX: ")); // Print how many message groups have been received (see note above)
    Serial.print(numSFRBX);
    Serial.print(F(" RAWX: "));
    Serial.println(numRAWX);

    uint16_t maxBufferBytes = myGNSS.getMaxFileBufferAvail(); // Get how full the file buffer has been (not how full it is now)

    //Serial.print(F("The maximum number of bytes which the file buffer has contained is: ")); // It is a fun thing to watch how full the buffer gets
    //Serial.println(maxBufferBytes);

    if (maxBufferBytes > ((fileBufferSize / 5) * 4)) // Warn the user if fileBufferSize was more than 80% full
    {
      Serial.println(F("Warning: the file buffer has been over 80% full. Some data may have been lost."));
    }

    lastPrint = millis(); // Update lastPrint
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  if (Serial.available()) // Check if the user wants to stop logging
  {
    uint16_t remainingBytes = myGNSS.fileBufferAvailable(); // Check if there are any bytes remaining in the file buffer

    while (remainingBytes > 0) // While there is still data in the file buffer
    {
      digitalWrite(STAT_LED, HIGH); // Flash the STAT LED while we write to the SD card

      uint16_t bytesToWrite = remainingBytes; // Write the remaining bytes to SD card sdWriteSize bytes at a time
      if (bytesToWrite > sdWriteSize)
      {
        bytesToWrite = sdWriteSize;
      }

      myGNSS.extractFileBufferData(myBuffer, bytesToWrite); // Extract bytesToWrite bytes from the UBX file buffer and put them into myBuffer

      myFile.write(myBuffer, bytesToWrite); // Write bytesToWrite bytes from myBuffer to the ubxDataFile on the SD card

      remainingBytes -= bytesToWrite; // Decrement remainingBytes
    }

    digitalWrite(STAT_LED, LOW); // Turn the STAT LED off

    myFile.close(); // Close the data file

    myGNSS.setNavigationFrequency(1); // Set navigation rate to 1Hz

    myGNSS.disableMessage(UBX_CLASS_RXM, UBX_RXM_RAWX, COM_PORT_SPI);
    myGNSS.disableMessage(UBX_CLASS_RXM, UBX_RXM_SFRBX, COM_PORT_SPI);

    Serial.println(F("Logging stopped. Freezing..."));
    while(1); // Do nothing more
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
}
