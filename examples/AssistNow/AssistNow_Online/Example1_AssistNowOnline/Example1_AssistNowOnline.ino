/*
  Use ESP32 WiFi to get AssistNow Online data from u-blox Thingstream
  By: SparkFun Electronics / Paul Clark
  Date: November 24th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to obtain AssistNow Online data from u-blox Thingstream over WiFi
  and push it over I2C to a u-blox module.

  You will need to have a token to be able to access Thingstream. See the AssistNow README for more details.

  Update secrets.h with your:
  - WiFi credentials
  - AssistNow token string

  Uncomment the "#define USE_MGA_ACKs" below to test the more robust method of using the
  UBX_MGA_ACK_DATA0 acknowledgements to confirm that each MGA message has been accepted.

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

//#define USE_MGA_ACKs // Uncomment this line to use the UBX_MGA_ACK_DATA0 acknowledgements

#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"

const char assistNowServer[] = "https://online-live1.services.u-blox.com";
//const char assistNowServer[] = "https://online-live2.services.u-blox.com"; // Alternate server

const char getQuery[] = "GetOnlineData.ashx?";
const char tokenPrefix[] = "token=";
const char tokenSuffix[] = ";";
const char getGNSS[] = "gnss=gps,glo;"; // GNSS can be: gps,qzss,glo,bds,gal
const char getDataType[] = "datatype=eph,alm,aux;"; // Data type can be: eph,alm,aux,pos

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

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
  // Use HTTP GET to receive the AssistNow_Online data

  const int URL_BUFFER_SIZE  = 256;
  char theURL[URL_BUFFER_SIZE]; // This will contain the HTTP URL
  int payloadSize = 0; // This will be updated with the length of the data we get from the server
  String payload; // This will store the data we get from the server

  // Assemble the URL
  // Note the slash after the first %s (assistNowServer)
  snprintf(theURL, URL_BUFFER_SIZE, "%s/%s%s%s%s%s%s",
    assistNowServer,
    getQuery,
    tokenPrefix,
    myAssistNowToken,
    tokenSuffix,
    getGNSS,
    getDataType);

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
  // Push the AssistNow data to the module

  if (payloadSize > 0)
  {  
    // Uncomment the next line to enable the 'major' debug messages on Serial so you can see what AssistNow data is being sent
    //myGNSS.enableDebugging(Serial, true);

#ifndef USE_MGA_ACKs

    // ***** Don't use the UBX_MGA_ACK_DATA0 messages *****

    // Push all the AssistNow data. Don't use UBX_MGA_ACK_DATA0's. Use the default delay of 7ms between messages.
    myGNSS.pushAssistNowData(payload, (size_t)payloadSize);

#else

    // ***** Use the UBX_MGA_ACK_DATA0 messages *****

    // Tell the module to return UBX_MGA_ACK_DATA0 messages when we push the AssistNow data
    myGNSS.setAckAiding(1);

    // Speed things up by setting setI2CpollingWait to 1ms
    myGNSS.setI2CpollingWait(1);

    // Push all the AssistNow data.
    // We have called setAckAiding(1) to instruct the module to return MGA-ACK messages.
    // So, we could set the pushAssistNowData mgaAck parameter to SFE_UBLOX_MGA_ASSIST_ACK_YES.
    // But, just for giggles, let's use SFE_UBLOX_MGA_ASSIST_ACK_ENQUIRE just to confirm that the
    // MGA-ACK messages are actually enabled.
    // Wait for up to 100ms for each ACK to arrive! 100ms is a bit excessive... 7ms is nearer the mark.
    myGNSS.pushAssistNowData(payload, (size_t)payloadSize, SFE_UBLOX_MGA_ASSIST_ACK_ENQUIRE, 100);

    // Set setI2CpollingWait to 125ms to avoid pounding the I2C bus
    myGNSS.setI2CpollingWait(125);

#endif

  }

  Serial.println(F("Here we go!"));
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
