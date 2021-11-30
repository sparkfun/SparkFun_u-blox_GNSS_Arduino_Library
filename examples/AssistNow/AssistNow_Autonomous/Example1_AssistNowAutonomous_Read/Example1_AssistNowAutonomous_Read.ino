/*
  Read the AssistNow Autonomous data from the module
  By: SparkFun Electronics / Paul Clark
  Date: November 29th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to enable, check the status of, and read the AssistNow Autonomous data from the module.

  Note: this example will only work on boards which have plenty of RAM available.
        The database can be several kBytes in length and needs to be stored twice:
        once inside the library (by readNavigationDatabase); and again in this example code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Thing Plus - ESP32 WROOM:        https://www.sparkfun.com/products/15663
  ZED-F9P RTK2:                             https://www.sparkfun.com/products/16481
  SparkFun GPS Breakout - ZOE-M8Q (Qwiic):  https://www.sparkfun.com/products/15193

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ESP32 Thing Plus
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

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
  // Keep calling getAOPSTATUSstatus until it returns zero (indicating AssistNow Autonomous data collection is complete)
  // or the user presses a key

  Serial.println(F("AssistNow Autonomous data collection is in progress. Press any key to quit."));
  Serial.println(F("NAV AOPSTATUS status indicates when the AssistNow Autonomous subsystem is idle (0) or running (not 0)."));

  bool keepGoing = true;
  int zerosSeen = 0; // Keep track of how many times aopStatus has been zero
  while (Serial.available()) Serial.read(); // Empty the serial buffer

  while (keepGoing && !Serial.available()) // Wait for keepGoing to go false, or for the arrival of a keypress
  {
    delay(1000);
    uint8_t aopStatus = myGNSS.getAOPSTATUSstatus();
    Serial.print(F("NAV AOPSTATUS status is: "));
    Serial.println(aopStatus);
    if (aopStatus == 0) // aopStatus will be zero when the AssistNow Autonomous subsystem is idle - i.e. data collection is complete
    {
      zerosSeen++; // Keep track of how long aopStatus has been zero
      if (zerosSeen >= 30)
        keepGoing = false; // Stop after seeing 30 consecutive zeros
    }
    else
    {
      zerosSeen = 0; // Reset the number of zeros seen
    }
  }

  if (!keepGoing)
    Serial.println(F("AssistNow Autonomous data collection is complete!"));

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Read the AssistNow Autonomous data from the module and pretty-print it (so it can be copied and pasted into Example2)

  #define MAX_DATABASE_LENGTH 32768 // Allocate 32kBytes to store the navigation database
  size_t maxDatabaseLen = MAX_DATABASE_LENGTH;
  uint8_t *database = new uint8_t[MAX_DATABASE_LENGTH]; // The database will be stored here

  Serial.println(F("Storage has been allocated for the database.")); Serial.flush();

  size_t actualDatabaseLen = myGNSS.readNavigationDatabase(database, maxDatabaseLen); // Read the database

  Serial.print(F("The Navigation Database length was "));
  Serial.println(actualDatabaseLen);

  if (actualDatabaseLen == maxDatabaseLen)
    Serial.println(F("There was not enough memory to store the entire database. Some data will have been lost!"));

  // Pretty-print the database so it can be copied into Example2
  Serial.println(F("Copy and paste the following into Example2, so you can write it back to the module:"));
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