/*
  Download the entire configuration of a module
  By: SparkFun Electronics / Nathan Seidle
  Date: May 19th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to download the entire ZED configuration and display
  it over serial. This same data can be piped to an SD card or other text file
  that can later be opened by u-center or by the uploadDeviceConfig() function.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  Serial.begin(115200); // You may need to increase this for high navigation rates!
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  myGNSS.setPacketCfgPayloadSize(1024); //Increase max payload to allow for 580 byte key responses

  //Basic call - Output config file to Serial
  myGNSS.downloadDeviceConfig();
  
  //Stream - Download config data to Serial. This can also be an SD file.
  //layer - Read the RAM layer. Currently only RAM layer is tested.
  //maxWait - Device can take up to 1000ms to respond with keys
  //downloadDeviceConfig(Serial, VAL_LAYER_RAM, 1000); 

  Serial.println("Config read complete!");
}

void loop()
{
}