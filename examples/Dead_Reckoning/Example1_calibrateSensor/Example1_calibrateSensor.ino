/*
  By: Elias Santistevan
  SparkFun Electronics
  Date: May, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  NEO-M8U: https://www.sparkfun.com/products/16329
  ZED-F9R: https://www.sparkfun.com/products/16344  

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a Redboard Qwiic
  If you don't have a platform with a Qwiic connection use the 
  SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output

  To take advantage of the internal IMU of either the Dead Reckoning GNSS
  boards (ZED-F9R, NEO-M8U), you must first calibrate it. This includes securing the GNSS module
  to your vehicle so that it is stable within 2 degrees and that the frame of
  reference of the board is consistent with the picture outlined in the
  Receiver-Description-Prot-Spec Datasheet under Automotive/Untethered Dead
  Reckoning. You may also check either the ZED-F9R or NEO-M8U Hookup Guide for
  more information. After the board is secure, you'll need to put the module
  through certain conditions for proper calibration: acceleration, turning,
  stopping for a few minutes, getting to a speed over 30km/h all under a clear sky 
  with good GNSS signal. This example simply looks at the
  "fusionMode" status which indicates whether the SparkFun Dead Reckoning is
  initializing - 0, calibrated - 1, or if an error has occurred - 2,3.  
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  //myGNSS.resetIMUalignment(); // Uncomment this line to reset the IMU alignment
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

  delay(250);
}
