/*
  Read the AssistNow Autonomous data from the module
  By: SparkFun Electronics / Paul Clark
  Date: November 29th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to enable, check the status of, and read the AssistNow Autonomous data from the module.

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
  // Keep calling getAOPSTATUSstatus until it returns zero (indicating AssistNow Autonomous data collection si complete)
  // or the user presses a key

  Serial.println(F("AssistNow Autonomous data collection is in progress. Press any key to quit."));

  bool keepGoing = true;
  while (Serial.available()) Serial.read(); // Empty the serial buffer

  while (keepGoing && !Serial.available()); // Wait keepGoing to go false, or for the arrival of a keypress
  {
    delay(1000);
    Serial.print(F("NAV AOPSTATUS status is: "));
    uint8_t aopStatus = myGNSS.getAOPSTATUSstatus();
    Serial.print(aopStatus);
    Serial.println(F(". (Don't worry! This could take a _long_ time...)"));
    if (aopStatus == 0) // aopStatus will be zero when the AssistNow Autonomous subsystem is idle - i.e. data collection is complete
      keepGoing = false;
  }

  if (!keepGoing)
        Serial.print(F("AssistNow Autonomous data collection is complete!"));

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Read the AssistNow Autonomous data from the module and pretty-print it (so it can be copied and pasted into Example2)        

}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
}
