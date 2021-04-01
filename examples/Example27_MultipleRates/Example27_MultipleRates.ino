/*
  Configuring the GNSS to produce multiple messages at different rates
  By: Paul Clark
  SparkFun Electronics
  Date: April 1st, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the U-Blox GNSS to output multiple messages at different rates:
  PVT is output every second;
  POSECEF is output every five seconds;
  VELNED is output every ten seconds.

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

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setMeasurementRate(1000); //Produce a measurement every 1000ms
  myGNSS.setNavigationRate(1); //Produce a navigation solution every measurement
  
  myGNSS.setAutoPVTrate(1); //Tell the GNSS to send the PVT solution every measurement
  myGNSS.setAutoNAVPOSECEFrate(5); //Tell the GNSS to send each POSECEF solution every 5th measurement
  myGNSS.setAutoNAVVELNEDrate(10); //Tell the GNSS to send each VELNED solution every 10th measurement
  //myGNSS.saveConfiguration(); //Optional: Save the current settings to flash and BBR
}

void loop()
{
  // Calling getPVT returns true if there actually is a fresh navigation solution available.
  if (myGNSS.getPVT())
  {
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
    Serial.println(F(" (mm)"));
  }

  // Calling getNAVPOSECEF returns true if there actually is a fresh position solution available.
  if (myGNSS.getNAVPOSECEF())
  {
    Serial.print(F("ecefX: "));
    Serial.print((float)myGNSS.packetUBXNAVPOSECEF->data.ecefX / 100.0, 2); // convert ecefX to m

    Serial.print(F(" ecefY: "));
    Serial.print((float)myGNSS.packetUBXNAVPOSECEF->data.ecefY / 100.0, 2); // convert ecefY to m

    Serial.print(F(" ecefZ: "));
    Serial.print((float)myGNSS.packetUBXNAVPOSECEF->data.ecefZ / 100.0, 2); // convert ecefY to m
    Serial.println(F(" (m)"));

    myGNSS.flushNAVPOSECEF(); //Mark all the data as read/stale so we get fresh data next time
  }

  // Calling getNAVVELNED returns true if there actually is fresh velocity data available.
  if (myGNSS.getNAVVELNED())
  {
    Serial.print(F("velN: "));
    Serial.print((float)myGNSS.packetUBXNAVVELNED->data.velN / 100.0, 2); // convert velN to m/s

    Serial.print(F(" velE: "));
    Serial.print((float)myGNSS.packetUBXNAVVELNED->data.velE / 100.0, 2); // convert velE to m/s

    Serial.print(F(" velD: "));
    Serial.print((float)myGNSS.packetUBXNAVVELNED->data.velD / 100.0, 2); // convert velD to m/s
    Serial.println(F(" (m/s)"));

    myGNSS.flushNAVVELNED(); //Mark all the data as read/stale so we get fresh data next time
  }
}
