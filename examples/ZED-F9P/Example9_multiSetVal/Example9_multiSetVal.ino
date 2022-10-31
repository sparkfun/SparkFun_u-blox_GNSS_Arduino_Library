/*
  Send UBX binary commands to enable RTCM sentences on u-blox ZED-F9P module
  Based on Example7 By: Nathan Seidle
  SparkFun Electronics
  Updated by Paul Clark to demonstrate setVal8/16/32, newCfgValset8/16/32, addCfgValset8/16/32 and sendCfgValset8/16/32
  Date: July 1st, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  u-blox changed how to configure their modules in 2019. As of version 23 of the UBX protocol the
  UBX-CFG commands are deprecated; they still work, they just recommend using VALSET, VALGET, and VALDEL
  commands instead. This example shows how to use this new command structure.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a RedBoard Qwiic or BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println("u-blox multi setVal example");

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  //myGNSS.enableDebugging(); //Enable debug messages over Serial (default)
  //myGNSS.enableDebugging(SerialUSB); //Enable debug messages over Serial USB

  bool setValueSuccess = true;

  //These key values are hard coded. You can obtain them from the ZED-F9P interface description doc
  //or from u-center's Messages->CFG->VALSET window. Keys must be 32-bit.
  //Choose setVal8, setVal16 or setVal32 depending on the required value data width (1, 2 or 4 bytes)
  //L, U1, I1, E1 and X1 values are 8-bit
  //U2, I2, E2 and X2 values are 16-bit
  //U4, I4, R4, E4, X4 values are 32-bit

  setValueSuccess &= myGNSS.setVal8(UBLOX_CFG_NMEA_HIGHPREC, 0); //Enable high precision NMEA (value is 8-bit (L / U1))
  //setValueSuccess &= myGNSS.setVal16(UBLOX_CFG_RATE_MEAS, 200); //Set measurement rate to 100ms (10Hz update rate) (value is 16-bit (U2))
  //setValueSuccess &= myGNSS.setVal16(UBLOX_CFG_RATE_MEAS, 200, VAL_LAYER_RAM); //Set rate setting in RAM only, instead of "ALL" (RAM, BBR and Flash)
  setValueSuccess &= myGNSS.setVal16(UBLOX_CFG_RATE_MEAS, 1000); //Set measurement rate to 1000ms (1Hz update rate) (value is 16-bit (U2))

  //Below is the original way we enabled a single RTCM message on the I2C port. After that, we show how to do the same
  //but with multiple messages all in one go using newCfgValset, addCfgValset and sendCfgValset.
  //Original: myGNSS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1); //Enable message 1005 to output through I2C port, message every second

  //If we will be sending a large number of key IDs and values, packetCfg could fill up before the CFG_VALSET is sent...
  //There are three possible solutions:
  //  Increase the space available by calling myGNSS.setPacketCfgPayloadSize
  //  Monitor how much space is remaining by calling myGNSS.getCfgValsetSpaceRemaining. Call myGNSS.sendCfgValset(); before packetCfg becomes full.
  //  Call myGNSS.autoSendCfgValsetAtSpaceRemaining(16); . This will cause the existing CFG_VALSET to be send automatically and a new one created when packetCfg has less than 16 bytes remaining.
  myGNSS.autoSendCfgValsetAtSpaceRemaining(16); // Trigger an auto-send when packetCfg has less than 16 bytes are remaining

  //Begin with newCfgValset
  setValueSuccess &= myGNSS.newCfgValset(); // Defaults to configuring the setting in Flash, RAM and BBR
  //setValueSuccess &= myGNSS.newCfgValset(VAL_LAYER_RAM); //Set this and the following settings in RAM only instead of Flash/RAM/BBR

  // Add KeyIDs and Values
  setValueSuccess &= myGNSS.addCfgValset8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C, 1); //Set output rate of msg 1005 over the I2C port to once per measurement (value is 8-bit (U1))
  setValueSuccess &= myGNSS.addCfgValset8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_I2C, 1); //Set output rate of msg 1077 over the I2C port to once per measurement (value is 8-bit (U1))
  setValueSuccess &= myGNSS.addCfgValset8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_I2C, 1); //Set output rate of msg 1087 over the I2C port to once per measurement (value is 8-bit (U1))
  setValueSuccess &= myGNSS.addCfgValset8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_I2C, 1); //Set output rate of msg 1127 over the I2C port to once per measurement (value is 8-bit (U1))
  setValueSuccess &= myGNSS.addCfgValset8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_I2C, 1); //Set output rate of msg 1097 over the I2C port to once per measurement (value is 8-bit (U1))
  setValueSuccess &= myGNSS.addCfgValset8(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_I2C, 10); //Set output rate of msg 1230 over the I2C port to once every 10 measurements (value is 8-bit (U1))

  // Send the packet using sendCfgValset
  setValueSuccess &= myGNSS.sendCfgValset();

  if (setValueSuccess == true)
  {
    Serial.println("Values were successfully set");
  }
  else
    Serial.println("Value set failed");
}

void loop()
{
}
