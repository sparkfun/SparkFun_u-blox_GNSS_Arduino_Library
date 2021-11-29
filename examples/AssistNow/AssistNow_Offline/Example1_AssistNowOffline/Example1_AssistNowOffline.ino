/*
  Use ESP32 WiFi to get AssistNow Offline data from u-blox Thingstream
  By: SparkFun Electronics / Paul Clark
  Date: November 26th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to obtain AssistNow Offline data from u-blox Thingstream over WiFi
  and push it over I2C to a u-blox module.

  The module still needs to be given time assistance to achieve a fast fix. This example
  uses network time to do that. If you don't have a WiFi connection, you may have to use
  a separate RTC to provide the time.

  Note: AssistNow Offline is not supported by the ZED-F9P! "The ZED-F9P supports AssistNow Online only."

  You will need to have a token to be able to access Thingstream. See the AssistNow README for more details.

  Update secrets.h with your:
  - WiFi credentials
  - AssistNow token string

  Uncomment the "#define USE_MGA_ACKs" below to test the more robust method of using the
  UBX_MGA_ACK_DATA0 acknowledgements to confirm that each MGA message has been accepted.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun Thing Plus - ESP32 WROOM:        https://www.sparkfun.com/products/15663
  SparkFun GPS Breakout - ZOE-M8Q (Qwiic):  https://www.sparkfun.com/products/15193

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ESP32 Thing Plus
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

//#define USE_MGA_ACKs // Uncomment this line to use the UBX_MGA_ACK_DATA0 acknowledgements

#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"

const char assistNowServer[] = "https://offline-live1.services.u-blox.com";
//const char assistNowServer[] = "https://offline-live2.services.u-blox.com"; // Alternate server

const char getQuery[] = "GetOfflineData.ashx?";
const char tokenPrefix[] = "token=";
const char tokenSuffix[] = ";";
const char getGNSS[] = "gnss=gps,glo;"; // GNSS can be: gps,qzss,glo,bds,gal
const char getFormat[] = "format=mga;"; // Data format. Leave set to mga for M8 onwards. Can be aid.
const char getPeriod[] = "period=1;"; // Optional. The number of weeks into the future that the data will be valid. Can be 1-5. Default = 4.
const char getMgaResolution[] = "resolution=1;"; // Optional. Data resolution: 1 = every day; 2 = every other day; 3 = every 3rd day.
//Note: always use resolution=1. findMGAANOForDate does not yet support finding the 'closest' date. It needs an exact match.

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

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Connect to WiFi.

  Serial.print(F("Connecting to local WiFi"));

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
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
  // Use HTTP GET to receive the AssistNow_Online data

  const int URL_BUFFER_SIZE  = 256;
  char theURL[URL_BUFFER_SIZE]; // This will contain the HTTP URL
  int payloadSize = 0; // This will be updated with the length of the data we get from the server
  String payload; // This will store the data we get from the server

  // Assemble the URL
  // Note the slash after the first %s (assistNowServer)
  snprintf(theURL, URL_BUFFER_SIZE, "%s/%s%s%s%s%s%s%s%s",
    assistNowServer,
    getQuery,
    tokenPrefix,
    myAssistNowToken,
    tokenSuffix,
    getGNSS,
    getFormat,
    getPeriod,
    getMgaResolution
    );

  Serial.print(F("HTTP URL is: "));
  Serial.println(theURL);

  HTTPClient http;

  http.begin(theURL);

  int httpCode = http.GET(); // HTTP GET

  // httpCode will be negative on error
  if(httpCode > 0)
  {
    // HTTP header has been sent and Server response header has been handled
    Serial.printf("[HTTP] GET... code: %d\r\n", httpCode);
  
    // If the GET was successful, read the data
    if(httpCode == HTTP_CODE_OK) // Check for code 200
    {
      payloadSize = http.getSize();
      Serial.printf("Server returned %d bytes\r\n", payloadSize);
      
      payload = http.getString(); // Get the payload

      // Pretty-print the payload as HEX
      /*
      int i;
      for(i = 0; i < payloadSize; i++)
      {
        if (payload[i] < 0x10) // Print leading zero
          Serial.print("0");
        Serial.print(payload[i], HEX);
        Serial.print(" ");
        if ((i % 16) == 15)
          Serial.println();
      }
      if ((i % 16) != 15)
        Serial.println();
      */
    }
  }
  else
  {
    Serial.printf("[HTTP] GET... failed, error: %s\r\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();  
  
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Find where the AssistNow data for today starts and ends

  size_t todayStart = 0; // Default to sending all the data
  size_t tomorrowStart = (size_t)payloadSize;
  
  // Uncomment the next line to enable the 'major' debug messages on Serial so you can see what AssistNow data is being sent
  //myGNSS.enableDebugging(Serial, true);

  if (payloadSize > 0)
  {
    if(getLocalTime(&timeinfo))
    {
      // Find the start of today's data
      todayStart = myGNSS.findMGAANOForDate(payload, (size_t)payloadSize, timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);
      if (todayStart < (size_t)payloadSize)
      {
        Serial.print(F("Found the data for today starting at location "));
        Serial.println(todayStart);
      }
      else
      {
        Serial.println("Could not find the data for today. This will not work well. The GNSS needs help to start up quickly.");
      }
      
      // Find the start of tomorrow's data
      tomorrowStart = myGNSS.findMGAANOForDate(payload, (size_t)payloadSize, timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, 1);
      if (tomorrowStart < (size_t)payloadSize)
      {
        Serial.print(F("Found the data for tomorrow starting at location "));
        Serial.println(tomorrowStart);
      }
      else
      {
        Serial.println("Could not find the data for tomorrow. (Today's data may be the last?)");
      }
    }
    else
    {
      Serial.println("Failed to obtain time. This will not work well. The GNSS needs accurate time to start up quickly.");
    }
  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Push the RTC time to the module

  if(getLocalTime(&timeinfo)) // Get the local time again, just to make sure we are using the most accurate time
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
  // Push the AssistNow data for today to the module - without the time

  if (payloadSize > 0)
  {  

#ifndef USE_MGA_ACKs

    // ***** Don't use the UBX_MGA_ACK_DATA0 messages *****

    // Push the AssistNow data for today. Don't use UBX_MGA_ACK_DATA0's. Use the default delay of 7ms between messages.
    myGNSS.pushAssistNowData(todayStart, true, payload, tomorrowStart - todayStart);

#else

    // ***** Use the UBX_MGA_ACK_DATA0 messages *****

    // Tell the module to return UBX_MGA_ACK_DATA0 messages when we push the AssistNow data
    myGNSS.setAckAiding(1);

    // Speed things up by setting setI2CpollingWait to 1ms
    myGNSS.setI2CpollingWait(1);

    // Push the AssistNow data for today.
    myGNSS.pushAssistNowData(todayStart, true, payload, tomorrowStart - todayStart, SFE_UBLOX_MGA_ASSIST_ACK_YES, 100);

    // Set setI2CpollingWait to 125ms to avoid pounding the I2C bus
    myGNSS.setI2CpollingWait(125);

#endif

  }

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
