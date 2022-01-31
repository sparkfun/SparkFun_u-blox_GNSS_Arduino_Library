/*
  Use ESP32 WiFi to get SPARTN data from PointPerfect (broker) as a Client
  By: u-blox AG / Michael Ammann
  Date: January 27th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to obtain SPARTN data from a PointPerfect Broker over WiFi
  and push it over I2C to a ZED-F9x.
  It's confusing, but the Arduino is acting as a 'client' to the PointPerfect SSR correction service.

  You will need to have a valid u-blox Thingstream account and have a PointPerfect Thing and payed plan. 
  Thingstream offers SSR corrections to SPARTN cabalble RTK receivers such as the u-blox ZED-F9 series 
  in continental Europ and US. There Network is planned to be expanded to ther regions over next years. 
  To see sign up go to https://portal.thingstream.io/app/location-services/things

  This is a proof of concept to show how to connect via MQTT to get SPARTN SSR correction. 
  Using WiFi for a rover is generally a bad idea because of limited WiFi range in the field. 
  You may use this exmaple in combination with a cell phone with hotspot mode enabled. 

  For more information about MQTT, SPARTN and PointPerfect Correction Services 
  please see: https://www.u-blox.com/en/product/pointperfect
  
  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/16481
  RTK Surveyor: https://www.sparkfun.com/products/18443
  RTK Express: https://www.sparkfun.com/products/18442
  
  Recommended Hardware:
  MicroMod GNSS Carrier Board: https://www.sparkfun.com/products/17722 
  ESP32 Micromod https://www.sparkfun.com/products/16781

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a ESP32 Thing Plus
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoMqttClient.h>  
#include "secrets.h"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;
    
//Global variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceived_ms = 0; //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("PointPerfect testing"));
  
  Wire.begin(); //Start I2C

  if (myGNSS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("u-blox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
 
  Serial.println(F("u-blox module connected"));
  myGNSS.setI2COutput(COM_TYPE_UBX); //Turn off NMEA noise
  myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_SPARTN); // Be sure SPARTN input is enabled.
   
  myGNSS.setNavigationFrequency(1); //Set output in Hz.
  Serial.print(F("Connecting to local WiFi"));
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.print(F("WiFi connected with IP: "));
  Serial.println(WiFi.localIP());
  
  while (Serial.available()) Serial.read();
}

void loop()
{
  if (Serial.available())
  {
    beginClient();
    while (Serial.available()) Serial.read(); //Empty buffer of any newline chars
  }

  Serial.println(F("Press any key to start MQTT/SPARTN Client."));

  delay(1000);
}

WiFiClientSecure wifiClient = WiFiClientSecure();
MqttClient mqttClient(wifiClient);

void mqttMessageHandler(int messageSize) {
  uint8_t spartnData[512 * 4]; //Most incoming data is around 500 bytes but may be larger
  int spartnCount = 0;
  Serial.print(F("Pushed data from "));
  Serial.print(mqttClient.messageTopic());
  Serial.println(F(" topic to ZED"));
  while (mqttClient.available())
  {
    char ch = mqttClient.read();
    //Serial.write(ch); //Pipe to serial port is fine but beware, it's a lot of binary data
    spartnData[spartnCount++] = ch;
    if (spartnCount == sizeof(spartnData)) 
      break;
  }

  if (spartnCount > 0)
  {
    //Push KEYS or SPARTN data to GNSS module over I2C
    myGNSS.pushRawData(spartnData, spartnCount, false);
    lastReceived_ms = millis();
  }
}

//Connect to STARTN MQTT broker, receive RTCM, and push to ZED module over I2C
void beginClient()
{
  Serial.println(F("Subscribing to Broker. Press key to stop"));
  delay(10); //Wait for any serial to arrive
  while (Serial.available()) Serial.read(); //Flush

  while (Serial.available() == 0)
  {
    //Connect if we are not already
    if (wifiClient.connected() == false)
    {
      // Connect to AWS IoT
      wifiClient.setCACert(AWS_CERT_CA);
      wifiClient.setCertificate(AWS_CERT_CRT);
      wifiClient.setPrivateKey(AWS_CERT_PRIVATE);
      if (!mqttClient.connect(AWS_IOT_ENDPOINT, AWS_IOT_PORT)) {
        Serial.print(F("MQTT connection failed! Error code = "));
        Serial.println(mqttClient.connectError());
        return;
      } else {
        Serial.println(F("You're connected to the PointPerfect MQTT broker: "));
        Serial.println(AWS_IOT_ENDPOINT);
        // Subscribe to MQTT and register a callback
        Serial.println(F("Subscribe to Topics")); 
        mqttClient.onMessage(mqttMessageHandler);
        mqttClient.subscribe(MQTT_TOPIC_KEY);
        mqttClient.subscribe(MQTT_TOPIC_SPARTN);
        lastReceived_ms = millis();
      } //End attempt to connect
    } //End connected == false
    else {
      mqttClient.poll();
    }
    //Close socket if we don't have new data for 10s
    if (millis() - lastReceived_ms > maxTimeBeforeHangup_ms)
    {
      Serial.println(F("SPARTN timeout. Disconnecting..."));
      if (mqttClient.connected() == true)
        mqttClient.stop();
      return;
    }

    delay(10);
  }

  Serial.println(F("User pressed a key"));
  Serial.println(F("Disconnecting..."));
  wifiClient.stop();
}
