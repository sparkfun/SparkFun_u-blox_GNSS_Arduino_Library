/*
  Get the GPGGA NMEA sentence using getLatestNMEAGPGGA
  By: Paul Clark
  SparkFun Electronics
  Date: January 12th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  ** Please note: this example will not run on Arduino Uno. See https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/blob/main/README.md#memory-usage

  This example shows how to turn on/off the NMEA sentences being output over I2C.
  It then demonstrates how to use the new getLatestNMEAGPGGA function to retrieve the latest GPGGA message.
  getLatestNMEAGPGGA returns immediately - it is not blocking.
  It returns:
    0 if no data is available
    1 if the data is valid but is stale (you have read it before)
    2 if the data is valid and fresh

  If the module is using multiple GNSS constellations, the GGA message will be prefixed with Talker ID "GN" instead of "GP".
  The library includes a getLatestNMEAGNGGA function too.
  This example shows how to use both functions - and how to change the Talker ID so the GNGGA messages become GPGGA.

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

  //Disable or enable various NMEA sentences over the I2C interface
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
}

void loop()
{
  // getLatestNMEAGPGGA calls checkUblox for us. We don't need to do it here

  NMEA_GGA_data_t data; // Storage for the GPGGA data
  uint8_t result = myGNSS.getLatestNMEAGPGGA(&data); // Get the latest GPGGA data (if any)
  
  if (result == 0)
  {
    Serial.println(F("No GPGGA data available"));
  }
  else if (result == 1)
  {
    Serial.println(F("GPGGA data is available but is stale"));
  }
  else // if (result == 2)
  {
    // Data contains .length and .nmea
    Serial.print(F("Latest GPGGA: Length: "));
    Serial.print(data.length);
    Serial.print(F("\tData: "));
    Serial.println((const char *)data.nmea); // .nmea is printable (NULL-terminated)
  }

  result = myGNSS.getLatestNMEAGNGGA(&data); // Get the latest GNGGA data (if any)
  
  if (result == 0)
  {
    Serial.println(F("No GNGGA data available"));
  }
  else if (result == 1)
  {
    Serial.println(F("GNGGA data is available but is stale"));
  }
  else // if (result == 2)
  {
    // Data contains .length and .nmea
    Serial.print(F("Latest GNGGA: Length: "));
    Serial.print(data.length);
    Serial.print(F("\tData: "));
    Serial.println((const char *)data.nmea); // .nmea is printable (NULL-terminated)
  }

  delay(250);
}
