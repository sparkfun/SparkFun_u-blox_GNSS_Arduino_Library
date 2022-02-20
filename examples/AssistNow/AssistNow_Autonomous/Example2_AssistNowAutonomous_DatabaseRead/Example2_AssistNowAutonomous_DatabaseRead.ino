/*
  Read the AssistNow Autonomous database from the module
  By: SparkFun Electronics / Paul Clark
  Date: November 29th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to enable, check the status of, and read the AssistNow Autonomous data from the module.

  Note: this example will only work on boards which have plenty of RAM available.
        The database can be several kBytes in length.

  Note: this example will not work on the ZED-F9P. "The ZED-F9P supports AssistNow Online only."

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Thing Plus - ESP32 WROOM:        https://www.sparkfun.com/products/15663
  SparkFun GPS Breakout - ZOE-M8Q (Qwiic):  https://www.sparkfun.com/products/15193

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ESP32 Thing Plus
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printSATdata will be called when new NAV SAT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_SAT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoNAVSATcallback
//        /                  _____  This _must_ be UBX_NAV_SAT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printSATdata(UBX_NAV_SAT_data_t *ubxDataStruct)
{
  Serial.println();
  
  Serial.print(F("UBX-NAV-SAT contains data for "));
  Serial.print(ubxDataStruct->header.numSvs);
  if (ubxDataStruct->header.numSvs == 1)
    Serial.println(F(" SV"));
  else
    Serial.println(F(" SVs"));

  uint16_t numAopAvail = 0; // Count how many SVs have AssistNow Autonomous data available
    
  for (uint16_t block = 0; block < ubxDataStruct->header.numSvs; block++) // For each SV
  {
    if (ubxDataStruct->blocks[block].flags.bits.aopAvail == 1) // If the aopAvail bit is set
      numAopAvail++; // Increment the number of SVs
  }

  Serial.print(F("AssistNow Autonomous data is available for "));
  Serial.print(numAopAvail);
  if (numAopAvail == 1)
    Serial.println(F(" SV"));
  else
    Serial.println(F(" SVs"));
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printAOPstatus will be called when new NAV AOPSTATUS data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_AOPSTATUS_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoNAVAOPSTATUScallback
//        /                  _____  This _must_ be UBX_NAV_AOPSTATUS_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printAOPstatus(UBX_NAV_AOPSTATUS_data_t *ubxDataStruct)
{
  //Serial.println();
  
  Serial.print(F("AOPSTATUS status is "));
  Serial.println(ubxDataStruct->status);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  delay(1000);

  Serial.begin(115200);
  Serial.println(F("AssistNow Example"));

  while (Serial.available()) Serial.read(); // Empty the serial buffer
  Serial.println(F("Press any key to begin..."));
  while (!Serial.available()); // Wait for a keypress

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Start I2C. Connect to the GNSS.

  Wire.begin(); //Start I2C

  if (myGNSS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("u-blox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  Serial.println(F("u-blox module connected"));

  myGNSS.setI2COutput(COM_TYPE_UBX); //Turn off NMEA noise

  //myGNSS.enableDebugging(Serial, true); // Uncomment this line to see helpful debug messages on Serial

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Enable AssistNow Autonomous data collection.

  if (myGNSS.setAopCfg(1) == true)
  {
    Serial.println(F("aopCfg enabled"));
  }
  else
  {
    Serial.println(F("Could not enable aopCfg. Please check wiring. Freezing."));
    while (1);
  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Enable automatic UBX-NAV-SAT and UBX-NAV-AOPSTATUS messages and set up the callbacks

  myGNSS.setNavigationFrequency(1); //Produce one solution per second

  myGNSS.setAutoNAVSATcallbackPtr(&printSATdata); // Enable automatic NAV SAT messages with callback to printSATdata
  myGNSS.setAutoAOPSTATUScallbackPtr(&printAOPstatus); // Enable automatic NAV AOPSTATUS messages with callback to printAOPstatus

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Keep displaying NAV SAT and AOPSTATUS until the user presses a key

  Serial.println(F("AssistNow Autonomous data collection is in progress. Press any key to quit and read the database."));

  while (Serial.available()) Serial.read(); // Empty the serial buffer

  while (!Serial.available()) // Wait for the arrival of a keypress
  {
    myGNSS.checkUblox(); // Check for the arrival of new data and process it.
    myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
  
    Serial.print(".");
    delay(50);
  }

  Serial.println();

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Disable the automatic UBX-NAV-SAT and UBX-NAV-AOPSTATUS messages

  myGNSS.setAutoNAVSAT(false);
  myGNSS.setAutoAOPSTATUS(false);
  delay(1100);

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Read the AssistNow Autonomous database from the module and pretty-print it (so it can be copied and pasted into the next example)

  #define MAX_DATABASE_LENGTH 32768 // Allocate 32kBytes to store the navigation database
  size_t maxDatabaseLen = MAX_DATABASE_LENGTH;
  uint8_t *database = new uint8_t[MAX_DATABASE_LENGTH]; // The database will be stored here

  Serial.println(F("Storage has been allocated for the database.")); Serial.flush();

  size_t actualDatabaseLen = myGNSS.readNavigationDatabase(database, maxDatabaseLen); // Read the database

  Serial.print(F("The Navigation Database length was "));
  Serial.println(actualDatabaseLen);

  if (actualDatabaseLen == maxDatabaseLen)
    Serial.println(F("There was not enough memory to store the entire database. Some data will have been lost!"));

  // Pretty-print the database so it can be copied into the next example
  Serial.println(F("Copy and paste the following into the next example, so you can write it back to the module:"));
  Serial.println();
  Serial.print(F("size_t databaseLen = "));
  Serial.print(actualDatabaseLen);
  Serial.println(F(";"));
  Serial.print(F("const uint8_t database["));
  Serial.print(actualDatabaseLen);
  Serial.println(F("] = {"));
  size_t i;
  for(i = 0; i < actualDatabaseLen; i++)
  {
    if ((i % 32) == 0)
      Serial.print(F("  0x"));
    if (*(database + i) < 0x10) // Print leading zero
      Serial.print(F("0"));
    Serial.print(*(database + i), HEX);
    if (i == (actualDatabaseLen - 1))
      Serial.println();
    else if ((i % 32) == 31)
      Serial.println(F(","));
    else
      Serial.print(F(", 0x"));
  }
  Serial.println(F("};"));

}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  // Nothing to do here
}
