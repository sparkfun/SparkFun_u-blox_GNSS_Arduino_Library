/*
  NEO-D9C QZSS-L6 receiver example
  By: SparkFun Electronics / Paul Clark
  Date: September 23rd, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to display a summary of the NEO-D9C's UBX-RXM-QZSSL6 data.
  It also enables UBX-RXM-QZSSL6 message output on both UART1 and UART2 at 38400 baud
  so you can feed the corrections directly to (e.g.) a ZED-F9P.

  We believe the NEO-D9C's I2C address should be 0x43 (like the NEO-D9S). But, reported by users in Japan,
  the initial NEO-D9C's use address 0x42 - which is the same as the ZED-F9P.

  If you have one of the initial NEO-D9C's, the address 0x42 should work for you.
  If you have a newer or upgraded NEO-D9C, then you may need to change to 0x43. See line 100.

  Also, again reported by users in Japan, the initial NEO-D9C's do not support UBX-CFG-PRT.
  The library uses UBX-CFG-PRT inside .begin (.isConnected) to check if the module is connected.
  This then fails with the initial NEO-D9C's.
  The work-around is to set the .begin assumeSuccess parameter to true.
  With newer NEO-D9C's this work-around may not be necessary. Again see line 100.
  
  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/16481
  NEO-D9S L-Band Correction Data Receiver: https://www.sparkfun.com/products/19390

  Hardware Connections:
  Use a Qwiic cable to connect the NEO-D9C QZSS-L6 corection data receiver to your board
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myQZSS; // NEO-D9C

#define OK(ok) (ok ? F("  ->  OK") : F("  ->  ERROR!")) // Convert uint8_t into OK/ERROR

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printRXMQZSSL6 will be called when new QZSS-L6 data arrives
// See u-blox_structs.h for the full definition of UBX_RXM_QZSSL6_message_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setRXMQZSSL6messageCallbackPtr
//        /                  _____  This _must_ be UBX_RXM_QZSSL6_message_data_t
//        |                 /                        _____ You can use any name you like for the struct
//        |                 |                       /
//        |                 |                       |
void printRXMQZSSL6(UBX_RXM_QZSSL6_message_data_t *qzssL6Data)
{
  Serial.println(F("New QZSS-L6 data received:"));

  Serial.print(F("Message version:      "));
  Serial.println(qzssL6Data->payload[0]);
  
  Serial.print(F("Satellite Identifier: "));
  Serial.println(qzssL6Data->payload[1]);
  
  Serial.print(F("Carrier / Noise:      "));
  double cno = (0.00390625 * ((double)qzssL6Data->payload[2])) + ((double)qzssL6Data->payload[3]);
  Serial.println(cno, 1);
  
  Serial.print(F("Bit Errors Corrected: "));
  Serial.println(qzssL6Data->payload[9]);
  
  uint16_t chInfo = (((uint16_t)qzssL6Data->payload[11]) << 8) | qzssL6Data->payload[10];
  uint16_t errStatus = ((chInfo >> 12) & 0x3);
  Serial.print(F("Receiver Channel:     "));
  Serial.println((chInfo >> 8) & 0x3);
  Serial.print(F("Message Name:         L6"));
  Serial.println(((chInfo >> 10) & 0x1) == 0 ? F("D") : F("E"));
  Serial.print(F("Error Status:         "));
  if (errStatus == 1)
    Serial.println("error-free");
  else if (errStatus == 2)
    Serial.println("erroneous");
  else
    Serial.println("unknown");
  Serial.print(F("Channel Name:         "));
  Serial.println(((chInfo >> 14) & 0x3) == 0 ? F("A") : F("B"));
  
  Serial.println();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  Serial.begin(115200);
  Serial.println(F("NEO-D9C Example"));

  Wire.begin(); //Start I2C

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin and configure the NEO-D9C QZSS-L6 receiver

  //myQZSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  // For the initial NEO-D9C's: connect using address 0x42; set the assumeSuccess parameter to true
  while (myQZSS.begin(Wire, 0x42, 1100, true) == false)
  // For newer NEO-D9C's: use address 0x43; leave assumeSuccess set to false (default)
  //while (myQZSS.begin(Wire, 0x43) == false)
  {
    Serial.println(F("u-blox NEO-D9C not detected at selected I2C address. Please check wiring and I2C address."));
    delay(2000);
  }
  Serial.println(F("u-blox NEO-D9C connected"));

  uint8_t ok = myQZSS.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_QZSSL6_I2C,   1);    // Output QZSS-L6 message on the I2C port 

  Serial.print(F("QZSS-L6: I2C configuration "));
  Serial.println(OK(ok));

  if (ok) ok = myQZSS.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_QZSSL6_UART1, 1);    // Output QZSS-L6 message on UART1
  if (ok) ok = myQZSS.setVal32(UBLOX_CFG_UART1_BAUDRATE,            38400); // Match UART1 baudrate with ZED

  Serial.print(F("QZSS-L6: UART1 configuration "));
  Serial.println(OK(ok));

  if (ok) ok = myQZSS.setVal8(UBLOX_CFG_UART2OUTPROT_UBX,            1);    // Enable UBX output on UART2
  if (ok) ok = myQZSS.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_QZSSL6_UART2, 1);    // Output QZSS-L6 message on UART2
  if (ok) ok = myQZSS.setVal32(UBLOX_CFG_UART2_BAUDRATE,            38400); // Match UART2 baudrate with ZED

  Serial.print(F("QZSS-L6: UART2 configuration "));
  Serial.println(OK(ok));

  myQZSS.setRXMQZSSL6messageCallbackPtr(&printRXMQZSSL6); // Call printRXMQZSSL6 when new QZSS-L6 data arrives
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  myQZSS.checkUblox(); // Check for the arrival of new QZSS-L6 data and process it.
  myQZSS.checkCallbacks(); // Check if any QZSS-L6 callbacks are waiting to be processed.
}
