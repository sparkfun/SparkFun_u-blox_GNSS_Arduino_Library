/*
  Get the latest GPGGA / GNGGA NMEA sentence using callbacks
  By: Paul Clark
  SparkFun Electronics
  Date: January 12th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  ** Please note: this example will not run on Arduino Uno. See https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/blob/main/README.md#memory-usage

  This example shows how to turn on/off the NMEA sentences being output over I2C.
  It then demonstrates how to get the latest GPGGA or GNGGA message autonomously using callbacks.

  If the module is using multiple GNSS constellations, the GGA message will be prefixed with Talker ID "GN" instead of "GP".
  This example shows how to change the Talker ID so the GNGGA messages become GPGGA.
  It also shows how to enable "high precision mode" to include extra decimal places in the GGA messages.

  This example turns off all sentences except for GGA.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a RedBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

// Callback: printGPGGA will be called when new GPGGA NMEA data arrives
// See u-blox_structs.h for the full definition of NMEA_GGA_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setNMEAGPGGAcallback
//        /               _____  This _must_ be NMEA_GGA_data_t
//        |              /           _____ You can use any name you like for the struct
//        |              |          /
//        |              |          |
void printGPGGA(NMEA_GGA_data_t *nmeaData)
{
    Serial.print(F("\r\nGPGGA: Length: "));
    Serial.print(nmeaData->length);
    Serial.print(F("\tData: "));
    Serial.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
}

// Callback: printGNGGA will be called if new GNGGA NMEA data arrives
// See u-blox_structs.h for the full definition of NMEA_GGA_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setNMEAGNGGAcallback
//        /               _____  This _must_ be NMEA_GGA_data_t
//        |              /           _____ You can use any name you like for the struct
//        |              |          /
//        |              |          |
void printGNGGA(NMEA_GGA_data_t *nmeaData)
{
    Serial.print(F("\r\nGNGGA: Length: "));
    Serial.print(nmeaData->length);
    Serial.print(F("\tData: "));
    Serial.print((const char *)nmeaData->nmea); // .nmea is printable (NULL-terminated) and already has \r\n on the end
}

void setup()
{

  Serial.begin(115200);
  Serial.println(F("SparkFun u-blox GNSS Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // Disable or enable various NMEA sentences over the I2C interface
  myGNSS.setI2COutput(COM_TYPE_NMEA | COM_TYPE_UBX); // Turn on both UBX and NMEA sentences on I2C. (Turn off RTCM and SPARTN)
  myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_I2C); // Several of these are on by default on ublox board so let's disable them
  myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_I2C);
  myGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_I2C);
  myGNSS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_I2C);
  myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_I2C);
  myGNSS.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C); // Leave only GGA enabled at current navigation rate

  // Set the Main Talker ID to "GP". The NMEA GGA messages will be GPGGA instead of GNGGA
  myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);
  //myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_DEFAULT); // Uncomment this line to restore the default main talker ID

  myGNSS.setHighPrecisionMode(true); // Enable High Precision Mode - include extra decimal places in the GGA messages

  //myGNSS.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save only the ioPort and message settings to NVM

  Serial.println(F("Messages configured"));

  //myGNSS.setNMEAOutputPort(Serial); // Uncomment this line to echo all NMEA data to Serial for debugging

  // Set up the callback for GPGGA
  myGNSS.setNMEAGPGGAcallbackPtr(&printGPGGA);

  // Set up the callback for GNGGA
  myGNSS.setNMEAGNGGAcallbackPtr(&printGNGGA);
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  Serial.print(".");
  delay(50);
}
