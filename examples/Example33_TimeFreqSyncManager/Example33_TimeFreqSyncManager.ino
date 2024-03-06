/*
  Configure Time & Frequency Sync manager (UBX-CFG-SMGR)  
  By: Danylo Ulianych
  SparkFun Electronics
  Date: March 6th, 2024
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example reads / sets UBX-CFG-SMGR configuration and prints UBX-TIM-SMEAS messages.
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


// Callback: printTIMSMEASdata will be called when new TIM SMEA data arrives
// See u-blox_structs.h for the full definition of UBX_TIM_SMEAS_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoTIMTM2callback
//        /                  _____  This _must_ be UBX_TIM_SMEAS_data_t
//        |                 /                 _____ You can use any name you like for the struct
//        |                 |                /
//        |                 |                |
void printTIMSMEASdata(UBX_TIM_SMEAS_data_t smea)
{
    Serial.print("UBX-TIM-SMEAS:");
    Serial.printf("\n version: %u", smea.version);
    Serial.printf("\n numMeas: %u", smea.numMeas);
    Serial.printf("\n iTOW: %lu", smea.iTOW);
    for (int i = 0; i < smea.numMeas; i++) {
      Serial.printf("\n sourceId %u:", smea.data[i].sourceId);
      Serial.printf("\n   flags:");
      Serial.printf("\n      freqValid %u", smea.data[i].flags.bits.freqValid);
      Serial.printf("\n      phaseValid %u", smea.data[i].flags.bits.phaseValid);
      Serial.printf("\n   phaseOffsetFrac %d", smea.data[i].phaseOffsetFrac);
      Serial.printf("\n   phaseUncFrac %u", smea.data[i].phaseUncFrac);
      Serial.printf("\n   phaseOffset %ld", smea.data[i].phaseOffset);
      Serial.printf("\n   phaseUnc %lu", smea.data[i].phaseUnc);
      Serial.printf("\n   freqOffset %ld", smea.data[i].freqOffset);
      Serial.printf("\n   freqUnc %lu", smea.data[i].freqUnc);
    }
}


UBX_CFG_SMGR_data_t convertRawBufToCfgSmgr(const ubxPacket* msg) {
  UBX_CFG_SMGR_data_t smgr;
  if (msg->len < sizeof(UBX_CFG_SMGR_data_t)) {
    Serial.printf("Payload message size (%zu) is too small to be converted to UBX_CFG_SMGR_data_t\n", msg->len);
    return smgr;
  }

  smgr.version = SFE_UBLOX_GNSS::extractByte(msg, 0);
  smgr.minGNSSFix = SFE_UBLOX_GNSS::extractByte(msg, 1);
  smgr.maxFreqChangeRate = SFE_UBLOX_GNSS::extractInt(msg, 2);
  smgr.maxPhaseCorrRate = SFE_UBLOX_GNSS::extractInt(msg, 4);
  smgr.freqTolerance = SFE_UBLOX_GNSS::extractInt(msg, 8);
  smgr.timeTolerance = SFE_UBLOX_GNSS::extractInt(msg, 10);
  smgr.messageCfg.all = SFE_UBLOX_GNSS::extractInt(msg, 12);
  smgr.maxSlewRate = SFE_UBLOX_GNSS::extractInt(msg, 14);
  smgr.flags.all = SFE_UBLOX_GNSS::extractLong(msg, 16);

  return smgr;
}


void printUbxCfgSmgr(const UBX_CFG_SMGR_data_t& smgr) {
  Serial.printf("\nUBX-CFG-SMGR:");
  Serial.printf("\n version %u (0x%02x)", smgr.version, smgr.version);
  Serial.printf("\n minGNSSFix %u (0x%02x)", smgr.minGNSSFix, smgr.minGNSSFix);
  Serial.printf("\n maxFreqChangeRate %u (0x%02x)", smgr.maxFreqChangeRate, smgr.maxFreqChangeRate);
  Serial.printf("\n maxPhaseCorrRate %u (0x%02x)", smgr.maxPhaseCorrRate, smgr.maxPhaseCorrRate);
  Serial.printf("\n freqTolerance %u (0x%02x)", smgr.freqTolerance, smgr.freqTolerance);
  Serial.printf("\n timeTolerance %u (0x%02x)", smgr.timeTolerance, smgr.timeTolerance);
  Serial.printf("\n messageCfg:");
  Serial.printf("\n   measInternal: %u", smgr.messageCfg.bits.measInternal);
  Serial.printf("\n   measGNSS: %u", smgr.messageCfg.bits.measGNSS);
  Serial.printf("\n   measEXTINT0: %u", smgr.messageCfg.bits.measEXTINT0);
  Serial.printf("\n   measEXTINT1: %u", smgr.messageCfg.bits.measEXTINT1);
  Serial.printf("\n maxSlewRate %u (0x%02x)", smgr.maxSlewRate, smgr.maxSlewRate);
  Serial.printf("\n flags:");
  Serial.printf("\n   disableInternal: %u", smgr.flags.bits.disableInternal);
  Serial.printf("\n   disableExternal: %u", smgr.flags.bits.disableExternal);
  Serial.printf("\n   preferenceMode: %u", smgr.flags.bits.preferenceMode);
  Serial.printf("\n   enableGNSS: %u", smgr.flags.bits.enableGNSS);
  Serial.printf("\n   enableEXTINT0: %u", smgr.flags.bits.enableEXTINT0);
  Serial.printf("\n   enableEXTINT1: %u", smgr.flags.bits.enableEXTINT1);
  Serial.printf("\n   enableHostMeasInt: %u", smgr.flags.bits.enableHostMeasInt);
  Serial.printf("\n   enableHostMeasExt: %u", smgr.flags.bits.enableHostMeasExt);
  Serial.printf("\n   useAnyFix: %u", smgr.flags.bits.useAnyFix);
  Serial.printf("\n   disableMaxSlewRate: %u", smgr.flags.bits.disableMaxSlewRate);
  Serial.printf("\n   issueFreqWarn: %u", smgr.flags.bits.issueFreqWarn);
  Serial.printf("\n   issueTimeWarn: %u", smgr.flags.bits.issueTimeWarn);
  Serial.printf("\n   TPCoherent: %u", smgr.flags.bits.TPCoherent);
  Serial.printf("\n   disableOffset: %u", smgr.flags.bits.disableOffset);
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

  uint8_t customPayload[MAX_PAYLOAD_SIZE]; // This array holds the payload data bytes. MAX_PAYLOAD_SIZE defaults to 256. The CFG_RATE payload is only 6 bytes!

  // The next line creates and initialises the packet information which wraps around the payload
  ubxPacket customCfg = {0, 0, 0, 0, 0, customPayload, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

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

  UBX_CFG_SMGR_data_t cfgSmgrPayload = convertRawBufToCfgSmgr(&customCfg);

  printUbxCfgSmgr(cfgSmgrPayload);

  cfgSmgrPayload.minGNSSFix = 5;  // update the min no. of GNSS fixes to start freq/phase sync
  cfgSmgrPayload.flags.bits.useAnyFix = 1;  // use any fix

  // update the raw payload buffer
  memmove(customPayload, &cfgSmgrPayload, sizeof(UBX_CFG_SMGR_data_t));

  // Now let's set the updated settings.
  if (myGNSS.sendCommand(&customCfg, maxWait) != SFE_UBLOX_STATUS_DATA_SENT) // We are expecting data and an ACK
  {
    Serial.println(F("sendCommand set failed! Freezing..."));
    while (1)
      ;
  }

  Serial.println("UBX-CFG-SMGR successfully updated");

  myGNSS.setAutoTIMSMEAcallback(&printTIMSMEASdata);

  // Enable info/warns messages
  // myGNSS.setVal8(UBLOX_CFG_INFMSG_UBX_I2C, 1);
}

void loop()
{
  myGNSS.checkUblox(); //See if new UBX data is available. Process bytes as they come in.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  delay(250); //Don't pound too hard on the I2C bus
}
