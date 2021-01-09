/*
  By: Paul Clark
  SparkFun Electronics
  Date: December, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example configures the High Navigation Rate on the NEO-M8U and then
  polls and displays the attitude solution, vehicle dynamics information
  and high rate position, velocity and time.
  
  This example polls the high rate data.
  (The next example uses "autoHNR" to receive the HNR data automatically.)
  
  Please make sure your NEO-M8U is running UDR firmware >= 1.31. Please update using u-center if necessary:
  https://www.u-blox.com/en/product/neo-m8u-module#tab-documentation-resources

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  NEO-M8U: https://www.sparkfun.com/products/16329

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a Redboard Qwiic
  If you don't have a platform with a Qwiic connection use the 
	SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output

*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_Ublox_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GPS myGPS;

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  //myGPS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  if (myGPS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("Warning! u-blox GPS did not begin correctly."));
    Serial.println(F("(This may be because the I2C port is busy with HNR messages.)"));
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  if (myGPS.setHNRNavigationRate(10) == true) //Set the High Navigation Rate to 10Hz
    Serial.println(F("setHNRNavigationRate was successful"));
  else
    Serial.println(F("setHNRNavigationRate was NOT successful"));

  myGPS.setAutoHNRATT(false); //Make sure auto HNR attitude messages are disabled
  myGPS.setAutoHNRINS(false); //Make sure auto HNR vehicle dynamics messages are disabled
  myGPS.setAutoHNRPVT(false); //Make sure auto HNR PVT messages are disabled
}

void loop()
{
  // Poll and print selected HNR data
  if (myGPS.getHNRAtt(125) == true) // Request HNR Att data using a 125ms timeout
  {
    Serial.print(F("Roll: "));
    Serial.print(myGPS.getHNRroll(), 2); // Use the helper function to get the roll in degrees
    Serial.print(F(" Pitch: "));
    Serial.print(myGPS.getHNRpitch(), 2); // Use the helper function to get the pitch in degrees
    Serial.print(F(" Heading: "));
    Serial.println(myGPS.getHNRheading(), 2); // Use the helper function to get the heading in degrees
  }
  if (myGPS.getHNRDyn(125) == true) // Request HNR Dyn data using a 125ms timeout
  {
    Serial.print(F("xAccel: "));
    Serial.print(myGPS.packetUBXHNRINS->data.xAccel);
    Serial.print(F(" yAccel: "));
    Serial.print(myGPS.packetUBXHNRINS->data.yAccel);
    Serial.print(F(" zAccel: "));
    Serial.println(myGPS.packetUBXHNRINS->data.zAccel);
  }
  if (myGPS.getHNRPVT(125) == true) // Request HNR PVT data using a 125ms timeout
  {
    Serial.print(F("ns: "));
    Serial.print(myGPS.packetUBXHNRPVT->data.nano);
    Serial.print(F(" Lat: "));
    Serial.print(myGPS.packetUBXHNRPVT->data.lat);
    Serial.print(F(" Lon: "));
    Serial.println(myGPS.packetUBXHNRPVT->data.lon);
  }
}
