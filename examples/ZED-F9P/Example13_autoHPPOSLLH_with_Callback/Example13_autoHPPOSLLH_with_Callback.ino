/*
  Configuring the GNSS to automatically send HPPOSLLH position reports over I2C
  and uses callbacks to process and display the data automatically
  By: Paul Clark
  Date: October 27th 2020

  Based on an earlier example:
  By: Nathan Seidle and Thorsten von Eicken
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the U-Blox GNSS the send navigation reports automatically
  and and uses callbacks to process and display the data automatically. No more polling!

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

// Callback: printHPdata will be called when new NAV HPPOSLLH data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_HPPOSLLH_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoHPPOSLLHcallback
//        /                  _____  This _must_ be UBX_NAV_HPPOSLLH_data_t
//        |                 /                   _____ You can use any name you like for the struct
//        |                 |                  /
//        |                 |                  |
void printHPdata(UBX_NAV_HPPOSLLH_data_t *ubxDataStruct)
{
  Serial.println();

  long highResLatitude = ubxDataStruct->lat;
  Serial.print(F("Hi Res Lat: "));
  Serial.print(highResLatitude);

  int highResLatitudeHp = ubxDataStruct->latHp;
  Serial.print(F(" "));
  Serial.print(highResLatitudeHp);

  long highResLongitude = ubxDataStruct->lon;
  Serial.print(F(" Hi Res Long: "));
  Serial.print(highResLongitude);

  int highResLongitudeHp = ubxDataStruct->lonHp;
  Serial.print(F(" "));
  Serial.print(highResLongitudeHp);

  float horizAccuracy = ((float)ubxDataStruct->hAcc) / 10000.0; // Convert hAcc from mm*0.1 to m
  Serial.print(F(" Horiz accuracy: "));
  Serial.println(horizAccuracy);
}

// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /                _____ You can use any name you like for the struct
//        |                 |               /
//        |                 |               |
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
    Serial.println();

    Serial.print(F("Time: ")); // Print the time
    uint8_t hms = ubxDataStruct->hour; // Print the hours
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = ubxDataStruct->min; // Print the minutes
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F(":"));
    hms = ubxDataStruct->sec; // Print the seconds
    if (hms < 10) Serial.print(F("0")); // Print a leading zero if required
    Serial.print(hms);
    Serial.print(F("."));
    unsigned long millisecs = ubxDataStruct->iTOW % 1000; // Print the milliseconds
    if (millisecs < 100) Serial.print(F("0")); // Print the trailing zeros correctly
    if (millisecs < 10) Serial.print(F("0"));
    Serial.print(millisecs);

    long latitude = ubxDataStruct->lat; // Print the latitude
    Serial.print(F(" Lat: "));
    Serial.print(latitude);

    long longitude = ubxDataStruct->lon; // Print the longitude
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
    Serial.print(F(" Height above MSL: "));
    Serial.print(altitude);
    Serial.println(F(" (mm)"));
}

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

  myGNSS.setNavigationFrequency(2); //Produce two solutions per second

  myGNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata

  myGNSS.setAutoHPPOSLLHcallbackPtr(&printHPdata); // Enable automatic NAV HPPOSLLH messages with callback to printHPdata
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it. You could set up a timer interrupt to do this for you.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  Serial.print(".");
  delay(50);
}
