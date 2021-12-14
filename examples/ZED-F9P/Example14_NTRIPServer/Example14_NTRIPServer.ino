/*
  Use ESP32 WiFi to push RTCM data to RTK2Go (Caster) as a Server
  By: SparkFun Electronics / Nathan Seidle
  Date: December 14th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to gather RTCM data over I2C and push it to a casting service over WiFi.
  It's confusing, but the Arduino is acting as a 'server' to a 'caster'. In this case we will
  use RTK2Go.com as our caster because it is free. A rover (car, surveyor stick, etc) can
  then connect to RTK2Go as a 'client' and get the RTCM data it needs.

  You will need to register your mountpoint here: http://www.rtk2go.com/new-reservation/
  (They'll probably block the credentials we include in this example)

  To see if your mountpoint is active go here: http://rtk2go.com:2101/

  This is a proof of concept. Serving RTCM to a caster over WiFi is useful when you need to
  set up a high-precision base station.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/16481
  RTK Surveyor: https://www.sparkfun.com/products/17369

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ESP32 Thing Plus
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <WiFi.h>
#include "secrets.h"
WiFiClient ntripCaster;

#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

//Global Variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSentRTCM_ms = 0;           //Time of last data pushed to socket
int maxTimeBeforeHangup_ms = 10000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster

uint32_t serverBytesSent = 0; //Just a running total
long lastReport_ms = 0;       //Time of last report of bytes sent
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  Serial.begin(115200); // You may need to increase this for high navigation rates!
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  Serial.print("Connecting to local WiFi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.print("\nWiFi connected with IP: ");
  Serial.println(WiFi.localIP());

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //UBX+RTCM3 is not a valid option so we enable all three.

  myGNSS.setNavigationFrequency(1); //Set output in Hz. RTCM rarely benefits from >1Hz.

  //Disable all NMEA sentences
  bool response = true;
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_I2C);

  if (response == false)
  {
    Serial.println(F("Failed to disable NMEA. Freezing..."));
    while (1)
      ;
  }
  else
    Serial.println(F("NMEA disabled"));

  //Enable necessary RTCM sentences
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1); //Enable message 1005 to output through UART2, message every second
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10); //Enable message every 10 seconds

  if (response == false)
  {
    Serial.println(F("Failed to enable RTCM. Freezing..."));
    while (1)
      ;
  }
  else
    Serial.println(F("RTCM sentences enabled"));

  //-1280208.308,-4716803.847,4086665.811 is SparkFun HQ so...
  //Units are cm with a high precision extension so -1234.5678 should be called: (-123456, -78)
  //For more infomation see Example12_setStaticPosition
  //Note: If you leave these coordinates in place and setup your antenna *not* at SparkFun, your receiver
  //will be very confused and fail to generate correction data because, well, you aren't at SparkFun...
  //See this tutorial on getting PPP coordinates: https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station/all
  response &= myGNSS.setStaticPosition(-128020830, -80, -471680384, -70, 408666581, 10); //With high precision 0.1mm parts
  if (response == false)
  {
    Serial.println(F("Failed to enter static position. Freezing..."));
    while (1)
      ;
  }
  else
    Serial.println(F("Static position set"));

  //Alternatively to setting a static position, you could do a survey-in
  //but it takes much longer to start generating RTCM data. See Example4_BaseWithLCD
  //myGNSS.enableSurveyMode(60, 5.000); //Enable Survey in, 60 seconds, 5.0m

  //If you were setting up a full GNSS station, you would want to save these settings.
  //Because setting an incorrect static position will disable the ability to get a lock, we will skip saving during this example
  //if (myGNSS.saveConfiguration() == false) //Save the current settings to flash and BBR
  //  Serial.println(F("Module failed to save"));

  Serial.println(F("Module configuration complete"));
}

void loop()
{
  if (Serial.available())
    beginServing();

  Serial.println(F("Press any key to start serving"));

  delay(1000);
}

void beginServing()
{
  Serial.println("Begin transmitting to caster. Press any key to stop");
  delay(10); //Wait for any serial to arrive
  while (Serial.available())
    Serial.read(); //Flush

  while (Serial.available() == 0)
  {
    //Connect if we are not already
    if (ntripCaster.connected() == false)
    {
      Serial.printf("Opening socket to %s\n", casterHost);

      if (ntripCaster.connect(casterHost, casterPort) == true) //Attempt connection
      {
        Serial.printf("Connected to %s:%d\n", casterHost, casterPort);

        const int SERVER_BUFFER_SIZE = 512;
        char serverRequest[SERVER_BUFFER_SIZE];

        snprintf(serverRequest,
                 SERVER_BUFFER_SIZE,
                 "SOURCE %s /%s\r\nSource-Agent: NTRIP SparkFun u-blox Server v1.0\r\n\r\n",
                 mountPointPW, mountPoint);

        Serial.println(F("Sending server request:"));
        Serial.println(serverRequest);
        ntripCaster.write(serverRequest, strlen(serverRequest));

        //Wait for response
        unsigned long timeout = millis();
        while (ntripCaster.available() == 0)
        {
          if (millis() - timeout > 5000)
          {
            Serial.println("Caster timed out!");
            ntripCaster.stop();
            return;
          }
          delay(10);
        }

        //Check reply
        bool connectionSuccess = false;
        char response[512];
        int responseSpot = 0;
        while (ntripCaster.available())
        {
          response[responseSpot++] = ntripCaster.read();
          if (strstr(response, "200") > 0) //Look for 'ICY 200 OK'
            connectionSuccess = true;
          if (responseSpot == 512 - 1)
            break;
        }
        response[responseSpot] = '\0';

        if (connectionSuccess == false)
        {
          Serial.printf("Failed to connect to Caster: %s", response);
          return;
        }
      } //End attempt to connect
      else
      {
        Serial.println("Connection to host failed");
        return;
      }
    } //End connected == false

    if (ntripCaster.connected() == true)
    {
      delay(10);
      while (Serial.available())
        Serial.read(); //Flush any endlines or carriage returns

      lastReport_ms = millis();
      lastSentRTCM_ms = millis();

      //This is the main sending loop. We scan for new ublox data but processRTCM() is where the data actually gets sent out.
      while (1)
      {
        if (Serial.available())
          break;

        myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

        //Close socket if we don't have new data for 10s
        //RTK2Go will ban your IP address if you abuse it. See http://www.rtk2go.com/how-to-get-your-ip-banned/
        //So let's not leave the socket open/hanging without data
        if (millis() - lastSentRTCM_ms > maxTimeBeforeHangup_ms)
        {
          Serial.println("RTCM timeout. Disconnecting...");
          ntripCaster.stop();
          return;
        }

        delay(10);

        //Report some statistics every 250
        if (millis() - lastReport_ms > 250)
        {
          lastReport_ms += 250;
          Serial.printf("Total sent: %d\n", serverBytesSent);
        }
      }
    }

    delay(10);
  }

  Serial.println("User pressed a key");
  Serial.println("Disconnecting...");
  ntripCaster.stop();

  delay(10);
  while (Serial.available())
    Serial.read(); //Flush any endlines or carriage returns
}

//This function gets called from the SparkFun u-blox Arduino Library.
//As each RTCM byte comes in you can specify what to do with it
//Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void SFE_UBLOX_GNSS::processRTCM(uint8_t incoming)
{
  if (ntripCaster.connected() == true)
  {
    ntripCaster.write(incoming); //Send this byte to socket
    serverBytesSent++;
    lastSentRTCM_ms = millis();
  }
}