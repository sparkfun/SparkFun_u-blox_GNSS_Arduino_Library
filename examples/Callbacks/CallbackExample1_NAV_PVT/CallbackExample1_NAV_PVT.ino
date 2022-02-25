/*
  Configuring the GNSS to automatically send position reports over I2C and display them using a callback
  By: Paul Clark
  SparkFun Electronics
  Date: December 30th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the u-blox GNSS to send navigation reports automatically
  and access the data via a callback. No more polling!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
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

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  myGNSS.setNavigationFrequency(2); //Produce two solutions per second

  myGNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  Serial.print(".");
  delay(50);
}
