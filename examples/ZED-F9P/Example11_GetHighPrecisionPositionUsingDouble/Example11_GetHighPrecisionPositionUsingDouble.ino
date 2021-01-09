/*
  Get the high precision geodetic solution for latitude and longitude using double
  By: Nathan Seidle
  Modified by: Paul Clark (PaulZC)
  SparkFun Electronics
  Date: April 17th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to inspect the accuracy of the high-precision
  positional solution. Please see below for information about the units.

  ** This example will only work correctly on platforms which support 64-bit double **

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and (e.g.) a Redboard Artemis https://www.sparkfun.com/products/15444
  or an Artemis Thing Plus https://www.sparkfun.com/products/15574
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> // Needed for I2C to GNSS

#define myWire Wire // This will work on the Redboard Artemis and the Artemis Thing Plus using Qwiic
//#define myWire Wire1 // Uncomment this line if you are using the extra SCL1/SDA1 pins (D17 and D16) on the Thing Plus

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal

  myWire.begin();

  //myGNSS.enableDebugging(Serial); // Uncomment this line to enable debug messages

  if (myGNSS.begin(myWire) == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // Check that this platform supports 64-bit (8 byte) double
  if (sizeof(double) < 8)
  {
    Serial.println(F("Warning! Your platform does not support 64-bit double."));
    Serial.println(F("The latitude and longitude will be inaccurate."));
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  //myGNSS.setNavigationFrequency(20); //Set output to 20 times a second

  byte rate = myGNSS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate: ");
  Serial.println(rate);

  //myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
}

void loop()
{
  //Query module only every second.
  //The module only responds when a new position is available.
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer

    // getHighResLatitude: returns the latitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLatitudeHp: returns the high resolution component of latitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getHighResLongitude: returns the longitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLongitudeHp: returns the high resolution component of longitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getElipsoid: returns the height above ellipsoid as an int32_t in mm
    // getElipsoidHp: returns the high resolution component of the height above ellipsoid as an int8_t in mm * 10^-1
    // getMeanSeaLevel: returns the height above mean sea level as an int32_t in mm
    // getMeanSeaLevelHp: returns the high resolution component of the height above mean sea level as an int8_t in mm * 10^-1
    // getHorizontalAccuracy: returns the horizontal accuracy estimate from HPPOSLLH as an uint32_t in mm * 10^-1

    // First, let's collect the position data
    int32_t latitude = myGNSS.getHighResLatitude();
    int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
    int32_t longitude = myGNSS.getHighResLongitude();
    int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
    int32_t ellipsoid = myGNSS.getElipsoid();
    int8_t ellipsoidHp = myGNSS.getElipsoidHp();
    int32_t msl = myGNSS.getMeanSeaLevel();
    int8_t mslHp = myGNSS.getMeanSeaLevelHp();
    uint32_t accuracy = myGNSS.getHorizontalAccuracy();

    // Defines storage for the lat and lon as double
    double d_lat; // latitude
    double d_lon; // longitude

    // Assemble the high precision latitude and longitude
    d_lat = ((double)latitude) / 10000000.0; // Convert latitude from degrees * 10^-7 to degrees
    d_lat += ((double)latitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )
    d_lon = ((double)longitude) / 10000000.0; // Convert longitude from degrees * 10^-7 to degrees
    d_lon += ((double)longitudeHp) / 1000000000.0; // Now add the high resolution component (degrees * 10^-9 )

   // Print the lat and lon
    Serial.print("Lat (deg): ");
    Serial.print(d_lat, 9);
    Serial.print(", Lon (deg): ");
    Serial.print(d_lon, 9);

    // Now define float storage for the heights and accuracy
    float f_ellipsoid;
    float f_msl;
    float f_accuracy;

    // Calculate the height above ellipsoid in mm * 10^-1
    f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;
    // Now convert to m
    f_ellipsoid = f_ellipsoid / 10000.0; // Convert from mm * 10^-1 to m

    // Calculate the height above mean sea level in mm * 10^-1
    f_msl = (msl * 10) + mslHp;
    // Now convert to m
    f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m

    // Convert the horizontal accuracy (mm * 10^-1) to a float
    f_accuracy = accuracy;
    // Now convert to m
    f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m

    // Finally, do the printing
    Serial.print(", Ellipsoid (m): ");
    Serial.print(f_ellipsoid, 4); // Print the ellipsoid with 4 decimal places

    Serial.print(", Mean Sea Level (m): ");
    Serial.print(f_msl, 4); // Print the mean sea level with 4 decimal places

    Serial.print(", Accuracy (m): ");
    Serial.println(f_accuracy, 4); // Print the accuracy with 4 decimal places
  }
}
