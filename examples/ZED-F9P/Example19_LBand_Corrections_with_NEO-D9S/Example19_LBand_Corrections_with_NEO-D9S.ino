/*
  Use the NEO-D9S L-Band receiver to provide corrections to a ZED-F9x via UBX-RXM-PMP messages
  By: SparkFun Electronics / Paul Clark
  Based on original code by: u-blox AG / Michael Ammann
  Date: February 7th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to obtain SPARTN correction data from a NEO-D9S L-Band receiver and push it over I2C to a ZED-F9x.

  This is a proof of concept to show how the UBX-RXM-PMP corrections control the accuracy.

  You will need a Thingstream PointPerfect account to be able to access the SPARTN Credentials (IP Dynamic Keys).
  Copy and paste the Current Key and Next Key into secrets.h.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/16481
  NEO-D9S: Coming soon!

  Hardware Connections:
  Use Qwiic cables to connect the NEO-D9S and ZED-F9x GNSS to your board
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include "secrets.h" // <- Copy and paste the Current Key and Next Key into secrets.h

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS; // ZED-F9x
SFE_UBLOX_GNSS myLBand; // NEO-D9S

const uint32_t myLBandFreq = 1556290000; // Uncomment this line to use the US SPARTN 1.8 service
//const uint32_t myLBandFreq = 1545260000; // Uncomment this line to use the EU SPARTN 1.8 service

#define OK(ok) (ok ? F("  ->  OK") : F("  ->  ERROR!")) // Convert uint8_t into OK/ERROR

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: pushRXMPMP will be called when new PMP data arrives
// See u-blox_structs.h for the full definition of UBX_RXM_PMP_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoRXMPMPcallbackPtr
//        /               _____  This _must_ be UBX_RXM_PMP_data_t
//        |              /              _____ You can use any name you like for the struct
//        |              |             /
//        |              |             |
void pushRXMPMP(UBX_RXM_PMP_data_t *pmpData)
{
  //Push the PMP data to the GNSS
  Serial.print(F("New RXM-PMP data received. Message version is 0x0"));
  Serial.print(pmpData->version, HEX);
  Serial.print(F(". numBytesUserData is "));
  size_t numDataBytes = 504; // Version 0x00 messages always contain 504 bytes of userData
  if (pmpData->version == 0x01)
    numDataBytes = pmpData->numBytesUserData;
  Serial.print(numDataBytes);
  Serial.println(F(". Pushing them to the GNSS..."));

  myGNSS.pushRawData(pmpData->userData, numDataBytes);
}

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
  long latitude = ubxDataStruct->lat; // Print the latitude
  Serial.print(F("Lat: "));
  Serial.print(latitude / 10000000L);
  Serial.print(F("."));
  Serial.print(abs(latitude % 10000000L));

  long longitude = ubxDataStruct->lon; // Print the longitude
  Serial.print(F("  Long: "));
  Serial.print(longitude / 10000000L);
  Serial.print(F("."));
  Serial.print(abs(longitude % 10000000L));

  long altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
  Serial.print(F("  Height: "));
  Serial.print(altitude);
  Serial.print(F(" (mm)"));

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

void setup()
{
  Serial.begin(115200);
  Serial.println(F("NEO-D9S SPARTN Corrections"));

  Wire.begin(); //Start I2C

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin and configure the ZED-F9x

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring."));
    delay(2000);
  }
  Serial.println(F("u-blox GNSS module connected"));

  uint8_t ok = myGNSS.setI2COutput(COM_TYPE_UBX); //Turn off NMEA noise
  if (ok) ok = myGNSS.setPortInput(COM_PORT_I2C, COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_SPARTN); //Be sure SPARTN input is enabled

  if (ok) ok = myGNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED); // Set the differential mode - ambiguities are fixed whenever possible

  if (ok) ok = myGNSS.setNavigationFrequency(1); //Set output in Hz.

  //Configure the SPARTN IP Dynamic Keys
  //"When the receiver boots, the host should send 'current' and 'next' keys in one message." - Use setDynamicSPARTNKeys for this.
  //"Every time the 'current' key is expired, 'next' takes its place."
  //"Therefore the host should then retrieve the new 'next' key and send only that." - Use setDynamicSPARTNKey for this.
  // The key can be provided in binary format or in ASCII Hex format, but in both cases keyLengthBytes _must_ represent the binary key length in bytes.
  if (ok) ok = myGNSS.setDynamicSPARTNKeys(currentKeyLengthBytes, currentKeyGPSWeek, currentKeyGPSToW, currentDynamicKey,
                                           nextKeyLengthBytes, nextKeyGPSWeek, nextKeyGPSToW, nextDynamicKey);

  //if (ok) ok = myGNSS.saveConfiguration(VAL_CFG_SUBSEC_IOPORT | VAL_CFG_SUBSEC_MSGCONF); //Optional: Save the ioPort and message settings to NVM
  
  Serial.print(F("GNSS: configuration "));
  Serial.println(OK(ok));

  myGNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata so we can watch the carrier solution go to fixed

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin and configure the NEO-D9S L-Band receiver

  //myLBand.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myLBand.begin(Wire, 0x43) == false) //Connect to the u-blox NEO-D9S using Wire port. The D9S default I2C address is 0x43 (not 0x42)
  {
    Serial.println(F("u-blox NEO-D9S not detected at default I2C address. Please check wiring."));
    delay(2000);
  }
  Serial.println(F("u-blox NEO-D9S connected"));

          ok = myLBand.setVal32(UBLOX_CFG_PMP_CENTER_FREQUENCY,   myLBandFreq); // Default 1539812500 Hz
  if (ok) ok = myLBand.setVal16(UBLOX_CFG_PMP_SEARCH_WINDOW,      2200);        // Default 2200 Hz
  if (ok) ok = myLBand.setVal8(UBLOX_CFG_PMP_USE_SERVICE_ID,      0);           // Default 1 
  if (ok) ok = myLBand.setVal16(UBLOX_CFG_PMP_SERVICE_ID,         21845);       // Default 50821
  if (ok) ok = myLBand.setVal16(UBLOX_CFG_PMP_DATA_RATE,          2400);        // Default 2400 bps
  if (ok) ok = myLBand.setVal8(UBLOX_CFG_PMP_USE_DESCRAMBLER,     1);           // Default 1
  if (ok) ok = myLBand.setVal16(UBLOX_CFG_PMP_DESCRAMBLER_INIT,   26969);       // Default 23560
  if (ok) ok = myLBand.setVal8(UBLOX_CFG_PMP_USE_PRESCRAMBLING,   0);           // Default 0
  if (ok) ok = myLBand.setVal64(UBLOX_CFG_PMP_UNIQUE_WORD,        16238547128276412563ull); 
  if (ok) ok = myLBand.setVal(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C,   1); // Ensure UBX-RXM-PMP is enabled on the I2C port 
  if (ok) ok = myLBand.setVal(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART1, 1); // Output UBX-RXM-PMP on UART1
  if (ok) ok = myLBand.setVal(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART2, 1); // Output UBX-RXM-PMP on UART2
  if (ok) ok = myLBand.setVal32(UBLOX_CFG_UART1_BAUDRATE,         38400); // match baudrate with ZED default
  if (ok) ok = myLBand.setVal32(UBLOX_CFG_UART2_BAUDRATE,         38400); // match baudrate with ZED default
  
  Serial.print(F("L-Band: configuration "));
  Serial.println(OK(ok));

  myLBand.softwareResetGNSSOnly(); // Do a restart

  myLBand.setAutoRXMPMPcallbackPtr(&pushRXMPMP); // Call pushRXMPMP when new PMP data arrives. Push it to the GNSS

}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new GNSS data and process it.
  myGNSS.checkCallbacks(); // Check if any GNSS callbacks are waiting to be processed.

  myLBand.checkUblox(); // Check for the arrival of new PMP data and process it.
  myLBand.checkCallbacks(); // Check if any LBand callbacks are waiting to be processed.
}
