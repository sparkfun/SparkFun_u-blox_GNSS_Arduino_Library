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
  Thingstream offers SSR corrections to SPARTN capable RTK receivers such as the u-blox ZED-F9 series 
  in continental Europe and US. Their Network is planned to be expanded to other regions over the next years. 
  To sign up, go to: https://portal.thingstream.io/app/location-services/things

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
#include <ArduinoMqttClient.h> // Click here to get the library: http://librarymanager/All#ArduinoMqttClient
#include "secrets.h"

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;
    
#define OK(ok) (ok ? F("  ->  OK") : F("  ->  ERROR!")) // Convert uint8_t into OK/ERROR

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global variables

long lastReceived_ms = 0; //5 RTCM messages take approximately ~300ms to arrive at 115200bps
int maxTimeBeforeHangup_ms = 10000; //If we fail to get a complete RTCM frame after 10s, then disconnect from caster

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallbackPtr
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  double latitude = ubxDataStruct->lat; // Print the latitude
  Serial.print(F("Lat: "));
  Serial.print(latitude / 10000000.0, 7);

  double longitude = ubxDataStruct->lon; // Print the longitude
  Serial.print(F("  Long: "));
  Serial.print(longitude / 10000000.0, 7);

  double altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
  Serial.print(F("  Height: "));
  Serial.print(altitude / 1000.0, 3);

  uint8_t fixType = ubxDataStruct->fixType; // Print the fix type
  Serial.print(F("  Fix: "));
  Serial.print(fixType);
  if (fixType == 0)
    Serial.print(F(" (None)"));
  else if (fixType == 1)
    Serial.print(F(" (Dead Reckoning)"));
  else if (fixType == 2)
    Serial.print(F(" (2D)"));
  else if (fixType == 3)
    Serial.print(F(" (3D)"));
  else if (fixType == 3)
    Serial.print(F(" (GNSS + Dead Reckoning)"));
  else if (fixType == 5)
    Serial.print(F(" (Time Only)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  uint8_t carrSoln = ubxDataStruct->flags.bits.carrSoln; // Print the carrier solution
  Serial.print(F("  Carrier Solution: "));
  Serial.print(carrSoln);
  if (carrSoln == 0)
    Serial.print(F(" (None)"));
  else if (carrSoln == 1)
    Serial.print(F(" (Floating)"));
  else if (carrSoln == 2)
    Serial.print(F(" (Fixed)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  uint32_t hAcc = ubxDataStruct->hAcc; // Print the horizontal accuracy estimate
  Serial.print(F("  Horizontal Accuracy Estimate: "));
  Serial.print(hAcc);
  Serial.print(F(" (mm)"));

  Serial.println();    
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printRXMCOR will be called when new RXM COR data arrives
// See u-blox_structs.h for the full definition of UBX_RXM_COR_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setRXMCORcallbackPtr
//        /                  _____  This _must_ be UBX_RXM_COR_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printRXMCOR(UBX_RXM_COR_data_t *ubxDataStruct)
{
  Serial.print(F("UBX-RXM-COR:  ebno: "));
  Serial.print(ubxDataStruct->ebno);

  Serial.print(F("  protocol: "));
  if (ubxDataStruct->statusInfo.bits.protocol == 1)
    Serial.print(F("RTCM3"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 2)
    Serial.print(F("SPARTN"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 29)
    Serial.print(F("PMP (SPARTN)"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 30)
    Serial.print(F("QZSSL6"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  errStatus: "));
  if (ubxDataStruct->statusInfo.bits.errStatus == 1)
    Serial.print(F("Error-free"));
  else if (ubxDataStruct->statusInfo.bits.errStatus == 2)
    Serial.print(F("Erroneous"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  msgUsed: "));
  if (ubxDataStruct->statusInfo.bits.msgUsed == 1)
    Serial.print(F("Not used"));
  else if (ubxDataStruct->statusInfo.bits.msgUsed == 2)
    Serial.print(F("Used"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  msgEncrypted: "));
  if (ubxDataStruct->statusInfo.bits.msgEncrypted == 1)
    Serial.print(F("Not encrypted"));
  else if (ubxDataStruct->statusInfo.bits.msgEncrypted == 2)
    Serial.print(F("Encrypted"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  msgDecrypted: "));
  if (ubxDataStruct->statusInfo.bits.msgDecrypted == 1)
    Serial.print(F("Not decrypted"));
  else if (ubxDataStruct->statusInfo.bits.msgDecrypted == 2)
    Serial.print(F("Successfully decrypted"));
  else
    Serial.print(F("Unknown"));

  Serial.println();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  Serial.println(F("PointPerfect testing"));

  Wire.begin(); //Start I2C

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring."));
    delay(2000);
  }
  Serial.println(F("u-blox GNSS module connected"));

  uint8_t ok = myGNSS.setI2COutput(COM_TYPE_UBX); //Turn off NMEA noise
  if (ok) ok = myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_SPARTN); // Be sure SPARTN input is enabled.

  if (ok) ok = myGNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED); // Set the differential mode - ambiguities are fixed whenever possible
  if (ok) ok = myGNSS.setNavigationFrequency(1); //Set output in Hz.
  if (ok) ok = myGNSS.setVal8(UBLOX_CFG_SPARTN_USE_SOURCE, 0); // Use IP source (default). Change this to 1 for L-Band (PMP)
  
  if (ok) ok = myGNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata so we can watch the carrier solution go to fixed

  if (ok) ok = myGNSS.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_COR_I2C, 1); // Enable UBX-RXM-COR messages on I2C
  if (ok) ok = myGNSS.setRXMCORcallbackPtr(&printRXMCOR); // Print the contents of UBX-RXM-COR messages so we can check if the SPARTN data is being decrypted successfully

  //if (ok) ok = myGNSS.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save the ioPort and message settings to NVM
  
  Serial.print(F("GNSS: configuration "));
  Serial.println(OK(ok));

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

  Serial.println(F("Press any key to start MQTT/SPARTN Client."));
  
}

void loop()
{
  if (Serial.available())
  {
    beginClient();

    while (Serial.available()) Serial.read(); //Empty buffer of any newline chars
    
    Serial.println(F("Press any key to start MQTT/SPARTN Client."));
  }

  myGNSS.checkUblox(); // Check for the arrival of new GNSS data and process it.
  myGNSS.checkCallbacks(); // Check if any GNSS callbacks are waiting to be processed.
}

WiFiClientSecure wifiClient = WiFiClientSecure();
MqttClient mqttClient(wifiClient);

void mqttMessageHandler(int messageSize)
{
  const uint16_t mqttLimit = 512;
  uint8_t *mqttData = new uint8_t[mqttLimit]; // Allocate memory to hold the MQTT data
  if (mqttData == NULL)
  {
    Serial.println(F("Memory allocation for mqttData failed!"));
    return;
  }

  Serial.print(F("Pushing data from "));
  Serial.print(mqttClient.messageTopic());
  Serial.println(F(" topic to ZED"));

  while (mqttClient.available())
  {
    uint16_t mqttCount = 0;

    while (mqttClient.available())
    {
      char ch = mqttClient.read();
      //Serial.write(ch); //Pipe to serial port is fine but beware, it's a lot of binary data
      mqttData[mqttCount++] = ch;
    
      if (mqttCount == mqttLimit)
        break;
    }

    if (mqttCount > 0)
    {
      //Push KEYS or SPARTN data to GNSS module over I2C
      myGNSS.pushRawData(mqttData, mqttCount, false);
      lastReceived_ms = millis();
    }
  }

  delete[] mqttData;
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
      mqttClient.setId(MQTT_CLIENT_ID);
      mqttClient.setKeepAliveInterval(60*1000);
      mqttClient.setConnectionTimeout( 5*1000);
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
        mqttClient.subscribe(MQTT_TOPIC_ASSISTNOW);
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

    myGNSS.checkUblox(); // Check for the arrival of new GNSS data and process it.
    myGNSS.checkCallbacks(); // Check if any GNSS callbacks are waiting to be processed.

    delay(10);
  }

  Serial.println(F("User pressed a key"));
  Serial.println(F("Disconnecting..."));
  wifiClient.stop();
}
