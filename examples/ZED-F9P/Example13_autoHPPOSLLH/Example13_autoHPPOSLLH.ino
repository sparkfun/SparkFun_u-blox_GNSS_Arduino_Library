/*
  Configuring the GNSS to automatically send HPPOSLLH position reports over I2C
  By: Paul Clark
  Date: October 27th 2020

  Based on an earlier example:
  By: Nathan Seidle and Thorsten von Eicken
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the U-Blox GNSS the send navigation reports automatically
  and retrieving the latest one via getHPPOSLLH. This eliminates the blocking in getHPPOSLLH while the GNSS
  produces a fresh navigation solution at the expense of returning a slighly old solution.

  This can be used over serial or over I2C, this example shows the I2C use. With serial the GNSS
  simply outputs the UBX_NAV_HPPOSLLH packet. With I2C it queues it into its internal I2C buffer (4KB in
  size?) where it can be retrieved in the next I2C poll.

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
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable lots of helpful debug messages
  //myGNSS.enableDebugging(Serial, true); // Uncomment this line to enable the minimum of helpful debug messages

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  //myGNSS.factoryDefault(); delay(5000);

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR

  myGNSS.setNavigationFrequency(1); //Produce one solution per second


  // The acid test: all four of these combinations should work seamlessly :-)

  //myGNSS.setAutoPVT(false); // Library will poll each reading
  //myGNSS.setAutoHPPOSLLH(false); // Library will poll each reading

  //myGNSS.setAutoPVT(true); // Tell the GPS to "send" each solution automatically
  //myGNSS.setAutoHPPOSLLH(false); // Library will poll each reading

  //myGNSS.setAutoPVT(false); // Library will poll each reading
  //myGNSS.setAutoHPPOSLLH(true); // Tell the GPS to "send" each hi res solution automatically

  myGNSS.setAutoPVT(true); // Tell the GPS to "send" each solution automatically
  myGNSS.setAutoHPPOSLLH(true); // Tell the GPS to "send" each hi res solution automatically
}

void loop()
{
  // Calling getHPPOSLLH returns true if there actually is a fresh navigation solution available.
  // Calling getPVT returns true if there actually is a fresh navigation solution available.
  if ((myGNSS.getHPPOSLLH()) || (myGNSS.getPVT()))
  {
    Serial.println();

    long highResLatitude = myGNSS.getHighResLatitude();
    Serial.print(F("Hi Res Lat: "));
    Serial.print(highResLatitude);

    int highResLatitudeHp = myGNSS.getHighResLatitudeHp();
    Serial.print(F(" "));
    Serial.print(highResLatitudeHp);

    long highResLongitude = myGNSS.getHighResLongitude();
    Serial.print(F(" Hi Res Long: "));
    Serial.print(highResLongitude);

    int highResLongitudeHp = myGNSS.getHighResLongitudeHp();
    Serial.print(F(" "));
    Serial.print(highResLongitudeHp);

    unsigned long horizAccuracy = myGNSS.getHorizontalAccuracy();
    Serial.print(F(" Horiz accuracy: "));
    Serial.print(horizAccuracy);

    long latitude = myGNSS.getLatitude();
    Serial.print(F(" Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.println(longitude);
  }
  else
  {
    Serial.print(".");
    delay(50);
  }
}
