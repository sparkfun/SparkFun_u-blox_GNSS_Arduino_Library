/*
  Display the relative position of a NEO-M8P-2 rover
  By: Paul Clark
  SparkFun Electronics
  Date: March 20th, 2023
  License: MIT. See license file for more information.

  This example shows how to query the module for RELPOS information in the NED frame.
  It assumes you already have RTCM correction data being fed to the receiver on UART1.
  Connect UART1 TX on the base to UART1 RX on the rover. Also connect GND to GND.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  NEO-M8P RTK: https://www.sparkfun.com/products/15005

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a RedBoard Qwiic or BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Connect UART1 TX on the base to UART1 RX on the rover. Also connect GND to GND.
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("u-blox NEO-M8P Rover Example");

  Wire.begin();

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  //myGNSS.factoryDefault(); delay(5000);

  // By default, UART1 Protocol In is set to NMEA + UBX + RTCM3. No changes are necessary. But we can manually configure the port if required.
  Serial.print(myGNSS.setPortInput(COM_PORT_UART1, COM_TYPE_UBX | COM_TYPE_RTCM3)); //Enable UBX and RTCM3 input on UART1
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR
}

void loop()
{
  // The data from getRELPOSNED (UBX-NAV-RELPOSNED) is returned in UBX_NAV_RELPOSNED_t packetUBXNAVRELPOSNED
  // Please see u-blox_structs.h for the full definition of UBX_NAV_RELPOSNED_t
  // You can either read the data from packetUBXNAVRELPOSNED directly
  // or can use the helper functions: getRelPosN/E/D; getRelPosAccN/E/D
  // Note that NEO-M8P RELPOSNED is different to ZED-F9P. It does not contain the relative length and heading.
  if (myGNSS.getRELPOSNED() == true)
  {
    Serial.print("relPosN: ");
    Serial.println(myGNSS.getRelPosN(), 4); // Use the helper functions to get the rel. pos. as m
    Serial.print("relPosE: ");
    Serial.println(myGNSS.getRelPosE(), 4);
    Serial.print("relPosD: ");
    Serial.println(myGNSS.getRelPosD(), 4);

    Serial.print("relPosHPN: ");
    Serial.println((float)myGNSS.packetUBXNAVRELPOSNED->data.relPosHPN / 10, 1); //High-precision component. Convert to mm
    Serial.print("relPosHPE: ");
    Serial.println((float)myGNSS.packetUBXNAVRELPOSNED->data.relPosHPE / 10, 1);
    Serial.print("relPosHPD: ");
    Serial.println((float)myGNSS.packetUBXNAVRELPOSNED->data.relPosHPD / 10, 1);

    Serial.print("accN: ");
    Serial.println(myGNSS.getRelPosAccN(), 4); // Use the helper functions to get the rel. pos. accuracy as m
    Serial.print("accE: ");
    Serial.println(myGNSS.getRelPosAccE(), 4);
    Serial.print("accD: ");
    Serial.println(myGNSS.getRelPosAccD(), 4);

    Serial.print("gnssFixOk: ");
    if (myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.gnssFixOK == true)
      Serial.println("x");
    else
      Serial.println("");

    Serial.print("diffSolution: ");
    if (myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.diffSoln == true)
      Serial.println("x");
    else
      Serial.println("");

    Serial.print("relPosValid: ");
    if (myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.relPosValid == true)
      Serial.println("x");
    else
      Serial.println("");

    Serial.print("carrier Solution Type: ");
    if (myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.carrSoln == 0)
      Serial.println("None");
    else if (myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.carrSoln == 1)
      Serial.println("Float");
    else if (myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.carrSoln == 2)
      Serial.println("Fixed");

    Serial.print("isMoving: ");
    if (myGNSS.packetUBXNAVRELPOSNED->data.flags.bits.isMoving == true)
      Serial.println("x");
    else
      Serial.println("");
  }
  else
    Serial.println("RELPOS request failed");

  Serial.println("");
}
