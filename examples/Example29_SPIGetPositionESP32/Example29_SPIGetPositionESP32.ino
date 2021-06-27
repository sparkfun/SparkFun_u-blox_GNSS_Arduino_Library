/*
  Reading lat and long via UBX binary commands over SPI (ESP32 specific example)
  By: Andrew Berridge
  Date: 27th June 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to query a u-blox module for its lat/long/altitude over SPI. We also
  turn off the NMEA output on the SPI port. This decreases the amount of SPI traffic 
  dramatically.

  Note: Long/lat are large numbers because they are * 10^7. To convert lat/long
  to something google maps understands simply divide the numbers by 10,000,000. We 
  do this so that we don't have to use floating point numbers.

  Leave NMEA parsing behind. Now you can simply ask the module for the datums you want!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  You need to connect the SPI pins from your microcontroller to the specific pins on your SparkFun product. 
  Connections will vary based on your microcontroller, but for reference please refer to this tutorial on SPI:
  https://learn.sparkfun.com/tutorials/serial-peripheral-interface-spi/all
  
  Most new boards now use the terms:
  CS - Chip Select
  COP - Controller Out, Peripheral In
  CIPO - Controller in, Peripheral Out
  SCK - Serial Clock

  You can choose any pin for Chip Select, but the others are likely either defined by the library you are using
  (see here for the standard Arduino one: https://www.arduino.cc/en/reference/SPI) or the microcontroller. The
  ESP32 has two standard, selectable SPI ports, for example.

  IMPORTANT: there have been reports that some ublox devices do not respond to the UBX protocol over SPI
  with the factory settings. You may find you need to connect to the device via USB and u-center and set
  the incoming protocol to UBX only. Make sure you disable all other protocols as inputs if you can't
  get things to work! Hopefully this is just a bug in the ublox firmware that will be fixed soon ;-)
  
*/

#include <SPI.h> //Needed for SPI to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

// Instantiate an instance of the SPI class. 
// This is the default ESP32 SPI interface. Your configuration may be different, depending on the microcontroller you are using!
SPIClass spiPort (HSPI); 
const uint8_t csPin = 15; // Change this to suit your own connection for the chip select pin

long lastTime = 0; //Simple local timer. Limits amount of SPI traffic to u-blox module.

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  spiPort.begin(); 

  myGNSS.setFileBufferSize(100); // set the file buffer size
  
  // Connect to the u-blox module using SPI port, csPin and speed setting
  // ublox devices generally work up to 5Mhz. We'll use 4Mhz for this example:
  if (myGNSS.begin(spiPort, csPin, 4000000) == false) 
  {
    Serial.println(F("u-blox GNSS not detected on SPI bus. Please check wiring. Freezing."));
    while (1);
  }
  myGNSS.disableMessage(UBX_CLASS_HNR, UBX_HNR_PVT, 0x04);
  myGNSS.disableMessage(UBX_CLASS_HNR, UBX_HNR_ATT, 0x04);
  myGNSS.disableMessage(UBX_CLASS_HNR, UBX_HNR_INS, 0x04);
  myGNSS.setAutoPVT(false); //Tell the GNSS to "send" each solution
  myGNSS.setPortInput(COM_PORT_SPI, COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}

void loop()
{
  //Query module only every second. Doing it more often will just cause SPI traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = myGNSS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
  }
}