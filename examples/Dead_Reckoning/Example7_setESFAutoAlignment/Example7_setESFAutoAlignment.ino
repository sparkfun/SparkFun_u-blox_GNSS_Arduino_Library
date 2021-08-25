/*
  By: Nathan Seidle
  SparkFun Electronics
  Date: August, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example configures the AutoAlignment option for the IMU.
  The ZED-F9R Integration guide recommends enabling Auto Alignment once
  the device has been attached to the vehicle's frame.
  Enabling auto-alignment will cause the the sensor fusion status
  to begin initialization. After driving around a few turns, the sensors
  should enter 'Calibrated' state. See example 1 for fusion state or
  monitor UBX-ESF-STATUS.

  As of writing the ZED-F9R is using HPS v1.2 firmware. Please update using u-center if necessary.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9R: https://www.sparkfun.com/products/16344
  ZED-F9R pHat: https://www.sparkfun.com/products/16475
  NEO-M8U: https://www.sparkfun.com/products/16329

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a Redboard Qwiic
  If you don't have a platform with a Qwiic connection use the
  SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/17912)
  Open the serial monitor at 115200 baud to see the output

*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("Warning! u-blox GPS did not begin correctly."));
    Serial.println(F("(This may be because the I2C port is busy with HNR messages.)"));
  }

  bool esfAutoAlignment = myGNSS.getESFAutoAlignment();
  Serial.print(F("esfAutoAlignment: "));
  if (esfAutoAlignment == true)
    Serial.println(F("True"));
  else
    Serial.println(F("False"));

  myGNSS.setESFAutoAlignment(true); //Enable UBX-CFG-ESFALG Automatic IMU-mount Alignment

  myGNSS.setAutoHNRATT(false); //Make sure auto HNR attitude messages are disabled
  myGNSS.setAutoHNRINS(false); //Make sure auto HNR vehicle dynamics messages are disabled
  myGNSS.setAutoHNRPVT(false); //Make sure auto HNR PVT messages are disabled

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}

void loop()
{
  // ESF data is produced at the navigation rate, so by default we'll get fresh data once per second
  if (myGNSS.getEsfInfo()) // Poll new ESF STATUS data
  {
    Serial.print(F("Fusion Mode: "));
    Serial.print(myGNSS.packetUBXESFSTATUS->data.fusionMode);
    if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 0)
      Serial.println(F("  Sensor is initializing..."));
    else if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 1)
      Serial.println(F("  Sensor is calibrated!"));
    else if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 2)
      Serial.println(F("  Sensor fusion is suspended!"));
    else if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 3)
      Serial.println(F("  Sensor fusion is disabled!"));
  }

  // Poll and print selected HNR data
  if (myGNSS.getHNRAtt(125) == true) // Request HNR Att data using a 125ms timeout
  {
    Serial.print(F("Roll: "));
    Serial.print(myGNSS.getHNRroll(), 2); // Use the helper function to get the roll in degrees
    Serial.print(F(" Pitch: "));
    Serial.print(myGNSS.getHNRpitch(), 2); // Use the helper function to get the pitch in degrees
    Serial.print(F(" Heading: "));
    Serial.println(myGNSS.getHNRheading(), 2); // Use the helper function to get the heading in degrees
  }
  if (myGNSS.getHNRDyn(125) == true) // Request HNR Dyn data using a 125ms timeout
  {
    Serial.print(F("xAccel: "));
    Serial.print(myGNSS.packetUBXHNRINS->data.xAccel);
    Serial.print(F(" yAccel: "));
    Serial.print(myGNSS.packetUBXHNRINS->data.yAccel);
    Serial.print(F(" zAccel: "));
    Serial.println(myGNSS.packetUBXHNRINS->data.zAccel);
  }
  if (myGNSS.getHNRPVT(125) == true) // Request HNR PVT data using a 125ms timeout
  {
    Serial.print(F("ns: "));
    Serial.print(myGNSS.packetUBXHNRPVT->data.nano);
    Serial.print(F(" Lat: "));
    Serial.print(myGNSS.packetUBXHNRPVT->data.lat);
    Serial.print(F(" Lon: "));
    Serial.println(myGNSS.packetUBXHNRPVT->data.lon);
  }

}