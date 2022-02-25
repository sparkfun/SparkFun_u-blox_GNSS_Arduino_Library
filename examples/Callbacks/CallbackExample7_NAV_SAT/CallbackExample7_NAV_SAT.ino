/*
  Configuring the GNSS to automatically send NAV SAT reports over I2C and display them using a callback
  By: Paul Clark
  SparkFun Electronics
  Date: December 1st, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the u-blox GNSS to send NAV SAT reports automatically
  and access the data via a callback. No more polling!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

// Callback: newNAVSAT will be called when new NAV SAT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_SAT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoNAVSATcallback
//        /             _____  This _must_ be UBX_NAV_SAT_data_t
//        |            /                _____ You can use any name you like for the struct
//        |            |               /
//        |            |               |
void newNAVSAT(UBX_NAV_SAT_data_t *ubxDataStruct)
{
  Serial.println();

  Serial.print(F("New NAV SAT data received. It contains data for "));
  Serial.print(ubxDataStruct->header.numSvs);
  if (ubxDataStruct->header.numSvs == 1)
    Serial.println(F(" SV."));
  else
    Serial.println(F(" SVs."));

  // Just for giggles, print the signal strength for each SV as a barchart
  for (uint16_t block = 0; block < ubxDataStruct->header.numSvs; block++) // For each SV
  {
    switch (ubxDataStruct->blocks[block].gnssId) // Print the GNSS ID
    {
      case 0:
        Serial.print(F("GPS     "));
      break;
      case 1:
        Serial.print(F("SBAS    "));
      break;
      case 2:
        Serial.print(F("Galileo "));
      break;
      case 3:
        Serial.print(F("BeiDou  "));
      break;
      case 4:
        Serial.print(F("IMES    "));
      break;
      case 5:
        Serial.print(F("QZSS    "));
      break;
      case 6:
        Serial.print(F("GLONASS "));
      break;
      default:
        Serial.print(F("UNKNOWN "));
      break;      
    }
    
    Serial.print(ubxDataStruct->blocks[block].svId); // Print the SV ID
    
    if (ubxDataStruct->blocks[block].svId < 10) Serial.print(F("   "));
    else if (ubxDataStruct->blocks[block].svId < 100) Serial.print(F("  "));
    else Serial.print(F(" "));

    // Print the signal strength as a bar chart
    for (uint8_t cno = 0; cno < ubxDataStruct->blocks[block].cno; cno++)
      Serial.print(F("="));

    Serial.println();
  }
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

  myGNSS.setAutoNAVSATcallbackPtr(&newNAVSAT); // Enable automatic NAV SAT messages with callback to newNAVSAT
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  Serial.print(".");
  delay(50);
}
