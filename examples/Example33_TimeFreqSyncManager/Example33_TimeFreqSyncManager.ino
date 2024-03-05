/*
  Configure Time & Frequency Sync manager (UBX-CFG-SMGR)  
  By: Danylo Ulianych
  SparkFun Electronics
  Date: March 6th, 2024
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example reads / sets UBX-CFG-SMGR configuration and polls for UBX messages.
  Works only with Time & Frequency Sync products like LEA-M8F, etc.
  
  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;


void printUbxCfgSmgr(const UBX_CFG_SMGR_data_t& smgr) {
  Serial.printf("\nUBX-CFG-SMGR:");
  Serial.printf("\n version %u (0x%02x)", smgr.version, smgr.version);
  Serial.printf("\n minGNSSFix %u (0x%02x)", smgr.minGNSSFix, smgr.minGNSSFix);
  Serial.printf("\n maxFreqChangeRate %u (0x%02x)", smgr.maxFreqChangeRate, smgr.maxFreqChangeRate);
  Serial.printf("\n maxPhaseCorrRate %u (0x%02x)", smgr.maxPhaseCorrRate, smgr.maxPhaseCorrRate);
  Serial.printf("\n freqTolerance %u (0x%02x)", smgr.freqTolerance, smgr.freqTolerance);
  Serial.printf("\n timeTolerance %u (0x%02x)", smgr.timeTolerance, smgr.timeTolerance);
  Serial.printf("\n messageCfg:");
  Serial.printf("\n   measInternal: %u", smgr.messageCfg.measInternal);
  Serial.printf("\n   measGNSS: %u", smgr.messageCfg.measGNSS);
  Serial.printf("\n   measEXTINT0: %u", smgr.messageCfg.measEXTINT0);
  Serial.printf("\n   measEXTINT1: %u", smgr.messageCfg.measEXTINT1);
  Serial.printf("\n maxSlewRate %u (0x%02x)", smgr.maxSlewRate, smgr.maxSlewRate);
  Serial.printf("\n flags:");
  Serial.printf("\n   disableInternal: %u", smgr.flags.disableInternal);
  Serial.printf("\n   disableExternal: %u", smgr.flags.disableExternal);
  Serial.printf("\n   preferenceMode: %u", smgr.flags.preferenceMode);
  Serial.printf("\n   enableGNSS: %u", smgr.flags.enableGNSS);
  Serial.printf("\n   enableEXTINT0: %u", smgr.flags.enableEXTINT0);
  Serial.printf("\n   enableEXTINT1: %u", smgr.flags.enableEXTINT1);
  Serial.printf("\n   enableHostMeasInt: %u", smgr.flags.enableHostMeasInt);
  Serial.printf("\n   enableHostMeasExt: %u", smgr.flags.enableHostMeasExt);
  Serial.printf("\n   useAnyFix: %u", smgr.flags.useAnyFix);
  Serial.printf("\n   disableMaxSlewRate: %u", smgr.flags.disableMaxSlewRate);
  Serial.printf("\n   issueFreqWarn: %u", smgr.flags.issueFreqWarn);
  Serial.printf("\n   issueTimeWarn: %u", smgr.flags.issueTimeWarn);
  Serial.printf("\n   TPCoherent: %u", smgr.flags.TPCoherent);
  Serial.printf("\n   disableOffset: %u", smgr.flags.disableOffset);
  Serial.println("\n");
}


void setup()
{
  Serial.begin(115200);
  while (!Serial);  // wait for Serial ready
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false)
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX);  // ignore NMEA messages

  // setPacketCfgPayloadSize tells the library how many bytes our customPayload can hold.
  // It is more memory-efficient to call setPacketCfgPayloadSize before .begin (to avoid creating a new buffer, copying across
  // the contents of the old buffer and then deleting the old buffer). But let's call it here just to prove that we can.
  myGNSS.setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  UBX_CFG_SMGR_data_t cfgSmgrPayload;

  // The next line creates and initialises the packet information which wraps around the payload
  ubxPacket customCfg = {0, 0, 0, 0, 0, (uint8_t*) &cfgSmgrPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  // The structure of ubxPacket is:
  // uint8_t cls           : The message Class
  // uint8_t id            : The message ID
  // uint16_t len          : Length of the payload. Does not include cls, id, or checksum bytes
  // uint16_t counter      : Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
  // uint16_t startingSpot : The counter value needed to go past before we begin recording into payload array
  // uint8_t *payload      : The payload
  // uint8_t checksumA     : Given to us by the module. Checked against the rolling calculated A/B checksums.
  // uint8_t checksumB
  // sfe_ublox_packet_validity_e valid            : Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
  // sfe_ublox_packet_validity_e classAndIDmatch  : Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID

  // sendCommand will return:
  // SFE_UBLOX_STATUS_DATA_RECEIVED if the data we requested was read / polled successfully
  // SFE_UBLOX_STATUS_DATA_SENT     if the data we sent was writted successfully (ACK'd)
  // Other values indicate errors. Please see the sfe_ublox_status_e enum for further details.

  // Referring to the u-blox M8 Receiver Description and Protocol Specification we see that
  // the navigation rate is configured using the UBX-CFG-RATE message. So let's load our
  // custom packet with the correct information so we can read (poll / get) the current settings.

  customCfg.cls = UBX_CLASS_CFG; // This is the message Class
  customCfg.id = UBX_CFG_SMGR; // This is the message ID
  customCfg.len = 0; // Setting the len (length) to zero let's us poll the current settings
  customCfg.startingSpot = 0; // Always set the startingSpot to zero (unless you really know what you are doing)

  // We also need to tell sendCommand how long it should wait for a reply
  uint16_t maxWait = 250; // Wait for up to 250ms (Serial may need a lot longer e.g. 1100)

  // Now let's read the current UBX-CFG-SMGR settings. The results will be loaded into customCfg.
  if (myGNSS.sendCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
  {
    Serial.println(F("sendCommand (poll / get) failed! Freezing..."));
    while (1)
      ;
  }

  printUbxCfgSmgr(cfgSmgrPayload);

  cfgSmgrPayload.minGNSSFix = 5;  // update the min no. of GNSS fixes to start freq/phase sync
  cfgSmgrPayload.flags.useAnyFix = 1;  // use any fix

  // Now let's set the updated settings.
  if (myGNSS.sendCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_SENT) // We are expecting data and an ACK
  {
    Serial.println(F("sendCommand set failed! Freezing..."));
    while (1)
      ;
  }

  Serial.println("UBX-CFG-SMGR successfully updated");

  myGNSS.setOutputPort(Serial);
}

void loop()
{
  myGNSS.checkUblox(); //See if new UBX data is available. Process bytes as they come in.

  delay(250); //Don't pound too hard on the I2C bus
}
