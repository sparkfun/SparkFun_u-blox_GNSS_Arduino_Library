/*
  Write the AssistNow Autonomous database data to the module
  By: SparkFun Electronics / Paul Clark
  Date: December 1st, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to write the AssistNow Autonomous database date back to the module.

  This example is written for the ESP32. A WiFi connection is used to get network time to pass to the module.
  (You could use an RTC instead.)

  Copy and paste the database data from the previous example into database.h.

  Update secrets.h with your:
  - WiFi credentials

  Note: this example will not work on the ZED-F9P. "The ZED-F9P supports AssistNow Online only."

  Note: this example works best if you have the GNSS RAM battery-backup disabled.
  All SparkFun boards have battery-backup for the RAM which will means the database is retained if you disconnect the power.
  The module will use the database data from the battery-backed RAM when you turn the power back on.
  You will only see the improvement in the time-to-first-fix if you disable the battery first - or you are using a non-SparkFun
  board that does not have the backup battery.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Thing Plus - ESP32 WROOM:        https://www.sparkfun.com/products/15663
  SparkFun GPS Breakout - ZOE-M8Q (Qwiic):  https://www.sparkfun.com/products/15193

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ESP32 Thing Plus
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include "database.h" // <- Copy and paste the database data from the previous example into database.h

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#include "time.h"

const char* ntpServer = "pool.ntp.org"; // The Network Time Protocol Server

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
  // Connect to WiFi.

  Serial.print(F("Connecting to local WiFi"));

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.println(F("WiFi connected!"));

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Set the RTC using network time. (Code taken from the SimpleTime example.)

  // Request the time from the NTP server and use it to set the ESP32's RTC.
  configTime(0, 0, ntpServer); // Set the GMT and daylight offsets to zero. We need UTC, not local time.

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
  }
  else
  {
    Serial.println(&timeinfo, "Time is: %A, %B %d %Y %H:%M:%S");
  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Push the RTC time to the module

  // Uncomment the next line to enable the 'major' debug messages on Serial so you can see what AssistNow data is being sent
  //myGNSS.enableDebugging(Serial, true);

  if(getLocalTime(&timeinfo))
  {
    // setUTCTimeAssistance uses a default time accuracy of 2 seconds which should be OK here.
    // Have a look at the library source code for more details.
    myGNSS.setUTCTimeAssistance(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }
  else
  {
    Serial.println("Failed to obtain time. This will not work well. The GNSS needs accurate time to start up quickly.");
  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Push the AssistNow Autonomous data to the module

  size_t bytesPushed = myGNSS.pushAssistNowData(database, databaseLen);

  Serial.print(F("Pushed "));
  Serial.print(bytesPushed);
  Serial.println(F(" bytes of AssistNow Autonomous data to the module"));

  if (bytesPushed != databaseLen)
    Serial.println(F("Warning: bytesPushed does not match databaseLen! Maybe the database contains bad data? Or there was a communication error?"));

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Disconnect the WiFi as it's no longer needed

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println(F("WiFi disconnected"));
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  // Print the UBX-NAV-PVT data so we can see how quickly the fixType goes to 3D
  
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
  Serial.print(F(" (mm)"));

  byte SIV = myGNSS.getSIV();
  Serial.print(F(" SIV: "));
  Serial.print(SIV);

  byte fixType = myGNSS.getFixType();
  Serial.print(F(" Fix: "));
  if(fixType == 0) Serial.print(F("No fix"));
  else if(fixType == 1) Serial.print(F("Dead reckoning"));
  else if(fixType == 2) Serial.print(F("2D"));
  else if(fixType == 3) Serial.print(F("3D"));
  else if(fixType == 4) Serial.print(F("GNSS + Dead reckoning"));
  else if(fixType == 5) Serial.print(F("Time only"));

  Serial.println();
}
