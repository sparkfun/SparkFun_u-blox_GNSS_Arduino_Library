/*
  NEO-D9S L-Band receiver example
  By: SparkFun Electronics / Paul Clark
  Date: March 7th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to display the NEO-D9S's received signal imbalance and magnitude, plus a summary of any received PMP data.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/16481
  NEO-D9S Correction Data Receiver: https://www.sparkfun.com/products/19390

  Hardware Connections:
  Use a Qwiic cable to connect the NEO-D9S L-Band corection data receiver to your board
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myLBand; // NEO-D9S

const uint32_t myLBandFreq = 1556290000; // Uncomment this line to use the US SPARTN 1.8 service
//const uint32_t myLBandFreq = 1545260000; // Uncomment this line to use the EU SPARTN 1.8 service

#define OK(ok) (ok ? F("  ->  OK") : F("  ->  ERROR!")) // Convert uint8_t into OK/ERROR

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printRXMPMP will be called when new PMP data arrives
// See u-blox_structs.h for the full definition of UBX_RXM_PMP_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setRXMPMPcallbackPtr
//        /               _____  This _must_ be UBX_RXM_PMP_data_t
//        |              /              _____ You can use any name you like for the struct
//        |              |             /
//        |              |             |
void printRXMPMP(UBX_RXM_PMP_data_t *pmpData)
{
  Serial.println(F("New PMP data received:"));

  Serial.print(F("PMP message version: "));
  Serial.println(pmpData->version);
  
  Serial.print(F("numBytesUserData :   "));
  Serial.println(pmpData->numBytesUserData);
  
  Serial.print(F("serviceIdentifier:   "));
  Serial.println(pmpData->serviceIdentifier);
  
  Serial.print(F("uniqueWordBitErrors: "));
  Serial.println(pmpData->uniqueWordBitErrors);
  
  Serial.print(F("fecBits:             "));
  Serial.println(pmpData->fecBits);
  
  Serial.print(F("ebno:                "));
  Serial.println(pmpData->ebno);

  Serial.println();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  Serial.begin(115200);
  Serial.println(F("NEO-D9S Example"));

  Wire.begin(); //Start I2C

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin and configure the NEO-D9S L-Band receiver

  //myLBand.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myLBand.begin(Wire, 0x43) == false) //Connect to the u-blox NEO-D9S using Wire port. The D9S default I2C address is 0x43 (not 0x42)
  {
    Serial.println(F("u-blox NEO-D9S not detected at default I2C address. Please check wiring."));
    delay(2000);
  }
  Serial.println(F("u-blox NEO-D9S connected"));

  uint8_t ok = myLBand.setVal32(UBLOX_CFG_PMP_CENTER_FREQUENCY,   myLBandFreq); // Default 1539812500 Hz
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
  if (ok) ok = myLBand.setVal(UBLOX_CFG_UART2OUTPROT_UBX, 1);         // Enable UBX output on UART2
  if (ok) ok = myLBand.setVal(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART2, 1); // Output UBX-RXM-PMP on UART2
  if (ok) ok = myLBand.setVal32(UBLOX_CFG_UART1_BAUDRATE,         38400); // match baudrate with ZED default
  if (ok) ok = myLBand.setVal32(UBLOX_CFG_UART2_BAUDRATE,         38400); // match baudrate with ZED default

  Serial.print(F("L-Band: configuration "));
  Serial.println(OK(ok));

  myLBand.softwareResetGNSSOnly(); // Do a restart

  myLBand.setRXMPMPcallbackPtr(&printRXMPMP); // Call printRXMPMP when new PMP data arrives
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  myLBand.checkUblox(); // Check for the arrival of new PMP data and process it.
  myLBand.checkCallbacks(); // Check if any LBand callbacks are waiting to be processed.

  UBX_MON_HW2_data_t hwStatus; // Create storage for the HW2 extended hardware status
  if (myLBand.getHW2status(&hwStatus)) // Request the extended hardware status
  {
    // Print the signal imbalance and magnitude
    Serial.print(F("Signal imbalance and magnitude:  ofsI: "));
    Serial.print(hwStatus.ofsI);
    Serial.print(F("  magI: "));
    Serial.print(hwStatus.magI);
    Serial.print(F("  ofsQ: "));
    Serial.print(hwStatus.ofsQ);
    Serial.print(F("  magQ: "));
    Serial.println(hwStatus.magQ);
  }
}
