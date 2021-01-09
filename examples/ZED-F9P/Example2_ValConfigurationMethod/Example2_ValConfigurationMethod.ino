/*
  Configuring u-blox Module using new VALGET / VALSET / VALDEL methods

  Please see u-blox_config_keys.h for the definitions of _all_ of the configuration keys
  
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  u-blox deprecated many -CFG messages and replaced them with new
  VALGET, VALSET, VALDEL methods. This shows the basics of how to use
  these methods.

  Leave NMEA parsing behind. Now you can simply ask the module for the datums you want!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  byte response;
  response = myGNSS.getVal8(UBLOX_CFG_I2C_ADDRESS, VAL_LAYER_RAM); // Get the I2C address (see u-blox_config_keys.h for details)
  Serial.print(F("I2C Address: 0x"));
  Serial.println(response >> 1, HEX); //We have to shift by 1 to get the common '7-bit' I2C address format

  response = myGNSS.getVal8(UBLOX_CFG_I2COUTPROT_NMEA, VAL_LAYER_RAM); // Get the flag indicating is NMEA should be output on I2C
  Serial.print(F("Output NMEA over I2C port: 0x"));
  Serial.print(response, HEX);
}

void loop()
{
}
