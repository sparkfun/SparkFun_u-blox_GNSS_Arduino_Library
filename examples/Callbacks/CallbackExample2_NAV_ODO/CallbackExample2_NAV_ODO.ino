/*
  Configuring the GNSS to automatically send odometer reports over I2C and display the data using a callback
  By: Paul Clark
  SparkFun Electronics
  Date: December 30th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the u-blox GNSS to send odometer reports automatically
  and display the data via a callback. No more polling!

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

// Callback: printODOdata will be called when new NAV ODO data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_ODO_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoNAVODOcallback
//        /                  _____  This _must_ be UBX_NAV_ODO_data_t
//        |                 /                   _____ You can use any name you like for the struct
//        |                 |                  /
//        |                 |                  |
void printODOdata(UBX_NAV_ODO_data_t *ubxDataStruct)
{
    Serial.println();

    Serial.print(F("TOW: ")); // Print the Time Of Week
    unsigned long iTOW = ubxDataStruct->iTOW; // iTOW is in milliseconds
    Serial.print(iTOW);
    Serial.print(F(" (ms)"));

    Serial.print(F(" Distance: "));
    unsigned long distance = ubxDataStruct->distance; // Print the distance
    Serial.print(distance);
    Serial.print(F(" (m)"));

    Serial.print(F(" Total Distance: "));
    unsigned long totalDistance = ubxDataStruct->totalDistance; // Print the total distance
    Serial.print(totalDistance);
    Serial.println(F(" (m)"));
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

  myGNSS.setNavigationFrequency(1); //Produce one solution per second

  //myGNSS.resetOdometer(); //Uncomment this line to reset the odometer

  myGNSS.setAutoNAVODOcallbackPtr(&printODOdata); // Enable automatic NAV ODO messages with callback to printODOdata
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  Serial.print(".");
  delay(50);
}
