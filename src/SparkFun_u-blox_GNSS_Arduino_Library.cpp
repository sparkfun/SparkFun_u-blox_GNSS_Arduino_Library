/*
  This is a library written for the u-blox ZED-F9P and NEO-M8P-2
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/16481
  https://www.sparkfun.com/products/15136
  https://www.sparkfun.com/products/15005
  https://www.sparkfun.com/products/15733
  https://www.sparkfun.com/products/15193
  https://www.sparkfun.com/products/15210

  Original version by Nathan Seidle @ SparkFun Electronics, September 6th, 2018
  v2.0 rework by Paul Clark @ SparkFun Electronics, December 31st, 2020

  This library handles configuring and handling the responses
  from a u-blox GPS module. Works with most modules from u-blox including
  the Zed-F9P, NEO-M8P-2, NEO-M9N, ZOE-M8Q, SAM-M8Q, and many others.

  https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.13

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  The MIT License (MIT)
  Copyright (c) 2016 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

SFE_UBLOX_GNSS::SFE_UBLOX_GNSS(void)
{
  // Constructor
  if (debugPin >= 0)
  {
    pinMode((uint8_t)debugPin, OUTPUT);
    digitalWrite((uint8_t)debugPin, HIGH);
  }

  _logNMEA.all = 0;                             // Default to passing no NMEA messages to the file buffer
  _processNMEA.all = SFE_UBLOX_FILTER_NMEA_ALL; // Default to passing all NMEA messages to processNMEA

  // Support for platforms like ESP32 which do not support multiple I2C restarts
  // If _i2cStopRestart is true, endTransmission will always use a stop. If false, a restart will be used where needed.
#if defined(ARDUINO_ARCH_ESP32)
  _i2cStopRestart = true; // Always use a stop
#else
  _i2cStopRestart = false; // Use a restart where needed
#endif
}

SFE_UBLOX_GNSS::~SFE_UBLOX_GNSS(void)
{
  // Destructor

  end(); // Delete all allocated memory - excluding payloadCfg, payloadAuto and spiBuffer

  if (payloadCfg != NULL)
  {
    delete[] payloadCfg; // Created with new[]
    payloadCfg = NULL;   // Redundant?
  }

  if (payloadAuto != NULL)
  {
    delete[] payloadAuto; // Created with new[]
    payloadAuto = NULL;   // Redundant?
  }

  if (spiBuffer != NULL)
  {
    delete[] spiBuffer; // Created with new[]
    spiBuffer = NULL;   // Redundant?
  }
}

// Stop all automatic message processing. Free all used RAM
void SFE_UBLOX_GNSS::end(void)
{
  // Note: payloadCfg is not deleted

  // Note: payloadAuto is not deleted

  // Note: spiBuffer is not deleted

  if (ubxFileBuffer != NULL) // Check if RAM has been allocated for the file buffer
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("end: the file buffer has been deleted. You will need to call setFileBufferSize before .begin to create a new one."));
    }
#endif
    delete[] ubxFileBuffer; // Created with new[]
    ubxFileBuffer = NULL;   // Redundant?
    fileBufferSize = 0;     // Reset file buffer size. User will have to call setFileBufferSize again
    fileBufferMaxAvail = 0;
  }

  if (moduleSWVersion != NULL)
  {
    delete moduleSWVersion; // Created with new moduleSWVersion_t
    moduleSWVersion = NULL; // Redundant?
  }

  if (currentGeofenceParams != NULL)
  {
    delete currentGeofenceParams; // Created with new geofenceParams_t
    currentGeofenceParams = NULL; // Redundant?
  }

  if (packetUBXNAVTIMELS != NULL)
  {
    delete packetUBXNAVTIMELS; // Created with new UBX_NAV_TIMELS_t
    packetUBXNAVTIMELS = NULL; // Redundant?
  }

  if (packetUBXNAVPOSECEF != NULL)
  {
    if (packetUBXNAVPOSECEF->callbackData != NULL)
    {
      delete packetUBXNAVPOSECEF->callbackData; // Created with new UBX_NAV_POSECEF_data_t
    }
    delete packetUBXNAVPOSECEF; // Created with new UBX_NAV_POSECEF_t
    packetUBXNAVPOSECEF = NULL; // Redundant?
  }

  if (packetUBXNAVSTATUS != NULL)
  {
    if (packetUBXNAVSTATUS->callbackData != NULL)
    {
      delete packetUBXNAVSTATUS->callbackData;
    }
    delete packetUBXNAVSTATUS;
    packetUBXNAVSTATUS = NULL; // Redundant?
  }

  if (packetUBXNAVDOP != NULL)
  {
    if (packetUBXNAVDOP->callbackData != NULL)
    {
      delete packetUBXNAVDOP->callbackData;
    }
    delete packetUBXNAVDOP;
    packetUBXNAVDOP = NULL; // Redundant?
  }

  if (packetUBXNAVATT != NULL)
  {
    if (packetUBXNAVATT->callbackData != NULL)
    {
      delete packetUBXNAVATT->callbackData;
    }
    delete packetUBXNAVATT;
    packetUBXNAVATT = NULL; // Redundant?
  }

  if (packetUBXNAVPVT != NULL)
  {
    if (packetUBXNAVPVT->callbackData != NULL)
    {
      delete packetUBXNAVPVT->callbackData;
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("end: packetUBXNAVPVT->callbackData has been deleted"));
      }
#endif
    }
    delete packetUBXNAVPVT;
    packetUBXNAVPVT = NULL; // Redundant?
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("end: packetUBXNAVPVT has been deleted"));
    }
#endif
  }

  if (packetUBXNAVODO != NULL)
  {
    if (packetUBXNAVODO->callbackData != NULL)
    {
      delete packetUBXNAVODO->callbackData;
    }
    delete packetUBXNAVODO;
    packetUBXNAVODO = NULL; // Redundant?
  }

  if (packetUBXNAVVELECEF != NULL)
  {
    if (packetUBXNAVVELECEF->callbackData != NULL)
    {
      delete packetUBXNAVVELECEF->callbackData;
    }
    delete packetUBXNAVVELECEF;
    packetUBXNAVVELECEF = NULL; // Redundant?
  }

  if (packetUBXNAVVELNED != NULL)
  {
    if (packetUBXNAVVELNED->callbackData != NULL)
    {
      delete packetUBXNAVVELNED->callbackData;
    }
    delete packetUBXNAVVELNED;
    packetUBXNAVVELNED = NULL; // Redundant?
  }

  if (packetUBXNAVHPPOSECEF != NULL)
  {
    if (packetUBXNAVHPPOSECEF->callbackData != NULL)
    {
      delete packetUBXNAVHPPOSECEF->callbackData;
    }
    delete packetUBXNAVHPPOSECEF;
    packetUBXNAVHPPOSECEF = NULL; // Redundant?
  }

  if (packetUBXNAVHPPOSLLH != NULL)
  {
    if (packetUBXNAVHPPOSLLH->callbackData != NULL)
    {
      delete packetUBXNAVHPPOSLLH->callbackData;
    }
    delete packetUBXNAVHPPOSLLH;
    packetUBXNAVHPPOSLLH = NULL; // Redundant?
  }

  if (packetUBXNAVPVAT != NULL)
  {
    if (packetUBXNAVPVAT->callbackData != NULL)
    {
      delete packetUBXNAVPVAT->callbackData;
    }
    delete packetUBXNAVPVAT;
    packetUBXNAVPVAT = NULL; // Redundant?
  }

  if (packetUBXNAVTIMEUTC != NULL)
  {
    if (packetUBXNAVTIMEUTC->callbackData != NULL)
    {
      delete packetUBXNAVTIMEUTC->callbackData;
    }
    delete packetUBXNAVTIMEUTC;
    packetUBXNAVTIMEUTC = NULL; // Redundant?
  }

  if (packetUBXNAVCLOCK != NULL)
  {
    if (packetUBXNAVCLOCK->callbackData != NULL)
    {
      delete packetUBXNAVCLOCK->callbackData;
    }
    delete packetUBXNAVCLOCK;
    packetUBXNAVCLOCK = NULL; // Redundant?
  }

  if (packetUBXNAVSVIN != NULL)
  {
    if (packetUBXNAVSVIN->callbackData != NULL)
    {
      delete packetUBXNAVSVIN->callbackData;
    }
    delete packetUBXNAVSVIN;
    packetUBXNAVSVIN = NULL; // Redundant?
  }

  if (packetUBXNAVSAT != NULL)
  {
    if (packetUBXNAVSAT->callbackData != NULL)
    {
      delete packetUBXNAVSAT->callbackData;
    }
    delete packetUBXNAVSAT;
    packetUBXNAVSAT = NULL; // Redundant?
  }

  if (packetUBXNAVRELPOSNED != NULL)
  {
    if (packetUBXNAVRELPOSNED->callbackData != NULL)
    {
      delete packetUBXNAVRELPOSNED->callbackData;
    }
    delete packetUBXNAVRELPOSNED;
    packetUBXNAVRELPOSNED = NULL; // Redundant?
  }

  if (packetUBXNAVAOPSTATUS != NULL)
  {
    if (packetUBXNAVAOPSTATUS->callbackData != NULL)
    {
      delete packetUBXNAVAOPSTATUS->callbackData;
    }
    delete packetUBXNAVAOPSTATUS;
    packetUBXNAVAOPSTATUS = NULL; // Redundant?
  }

  if (packetUBXNAVEOE != NULL)
  {
    if (packetUBXNAVEOE->callbackData != NULL)
    {
      delete packetUBXNAVEOE->callbackData;
    }
    delete packetUBXNAVEOE;
    packetUBXNAVEOE = NULL; // Redundant?
  }

  if (packetUBXRXMPMP != NULL)
  {
    if (packetUBXRXMPMP->callbackData != NULL)
    {
      delete packetUBXRXMPMP->callbackData;
    }
    delete packetUBXRXMPMP;
    packetUBXRXMPMP = NULL; // Redundant?
  }

  if (packetUBXRXMPMPmessage != NULL)
  {
    if (packetUBXRXMPMPmessage->callbackData != NULL)
    {
      delete packetUBXRXMPMPmessage->callbackData;
    }
    delete packetUBXRXMPMPmessage;
    packetUBXRXMPMPmessage = NULL; // Redundant?
  }

  if (packetUBXRXMQZSSL6message != NULL)
  {
    if (packetUBXRXMQZSSL6message->callbackData != NULL)
    {
      delete[] packetUBXRXMQZSSL6message->callbackData;
    }
    delete packetUBXRXMQZSSL6message;
    packetUBXRXMQZSSL6message = NULL; // Redundant?
  }

  if (packetUBXRXMCOR != NULL)
  {
    if (packetUBXRXMCOR->callbackData != NULL)
    {
      delete packetUBXRXMCOR->callbackData;
    }
    delete packetUBXRXMCOR;
    packetUBXRXMCOR = NULL; // Redundant?
  }

  if (packetUBXRXMSFRBX != NULL)
  {
    if (packetUBXRXMSFRBX->callbackData != NULL)
    {
      delete packetUBXRXMSFRBX->callbackData;
    }
    delete packetUBXRXMSFRBX;
    packetUBXRXMSFRBX = NULL; // Redundant?
  }

  if (packetUBXRXMRAWX != NULL)
  {
    if (packetUBXRXMRAWX->callbackData != NULL)
    {
      delete packetUBXRXMRAWX->callbackData;
    }
    delete packetUBXRXMRAWX;
    packetUBXRXMRAWX = NULL; // Redundant?
  }

  if (packetUBXCFGRATE != NULL)
  {
    delete packetUBXCFGRATE;
    packetUBXCFGRATE = NULL; // Redundant?
  }

  if (packetUBXTIMTM2 != NULL)
  {
    if (packetUBXTIMTM2->callbackData != NULL)
    {
      delete packetUBXTIMTM2->callbackData;
    }
    delete packetUBXTIMTM2;
    packetUBXTIMTM2 = NULL; // Redundant?
  }

  if (packetUBXESFALG != NULL)
  {
    if (packetUBXESFALG->callbackData != NULL)
    {
      delete packetUBXESFALG->callbackData;
    }
    delete packetUBXESFALG;
    packetUBXESFALG = NULL; // Redundant?
  }

  if (packetUBXESFSTATUS != NULL)
  {
    if (packetUBXESFSTATUS->callbackData != NULL)
    {
      delete packetUBXESFSTATUS->callbackData;
    }
    delete packetUBXESFSTATUS;
    packetUBXESFSTATUS = NULL; // Redundant?
  }

  if (packetUBXESFINS != NULL)
  {
    if (packetUBXESFINS->callbackData != NULL)
    {
      delete packetUBXESFINS->callbackData;
    }
    delete packetUBXESFINS;
    packetUBXESFINS = NULL; // Redundant?
  }

  if (packetUBXESFMEAS != NULL)
  {
    if (packetUBXESFMEAS->callbackData != NULL)
    {
      delete packetUBXESFMEAS->callbackData;
    }
    delete packetUBXESFMEAS;
    packetUBXESFMEAS = NULL; // Redundant?
  }

  if (packetUBXESFRAW != NULL)
  {
    if (packetUBXESFRAW->callbackData != NULL)
    {
      delete packetUBXESFRAW->callbackData;
    }
    delete packetUBXESFRAW;
    packetUBXESFRAW = NULL; // Redundant?
  }

  if (packetUBXMGAACK != NULL)
  {
    delete packetUBXMGAACK;
    packetUBXMGAACK = NULL; // Redundant?
  }

  if (packetUBXMGADBD != NULL)
  {
    delete packetUBXMGADBD;
    packetUBXMGADBD = NULL; // Redundant?
  }

  if (packetUBXHNRATT != NULL)
  {
    if (packetUBXHNRATT->callbackData != NULL)
    {
      delete packetUBXHNRATT->callbackData;
    }
    delete packetUBXHNRATT;
    packetUBXHNRATT = NULL; // Redundant?
  }

  if (packetUBXHNRINS != NULL)
  {
    if (packetUBXHNRINS->callbackData != NULL)
    {
      delete packetUBXHNRINS->callbackData;
    }
    delete packetUBXHNRINS;
    packetUBXHNRINS = NULL; // Redundant?
  }

  if (packetUBXHNRPVT != NULL)
  {
    if (packetUBXHNRPVT->callbackData != NULL)
    {
      delete packetUBXHNRPVT->callbackData;
    }
    delete packetUBXHNRPVT;
    packetUBXHNRPVT = NULL; // Redundant?
  }

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  if (storageNMEAGPGGA != NULL)
  {
    if (storageNMEAGPGGA->callbackCopy != NULL)
    {
      delete storageNMEAGPGGA->callbackCopy;
    }
    delete storageNMEAGPGGA;
    storageNMEAGPGGA = NULL; // Redundant?
  }

  if (storageNMEAGNGGA != NULL)
  {
    if (storageNMEAGNGGA->callbackCopy != NULL)
    {
      delete storageNMEAGNGGA->callbackCopy;
    }
    delete storageNMEAGNGGA;
    storageNMEAGNGGA = NULL; // Redundant?
  }

  if (storageNMEAGPVTG != NULL)
  {
    if (storageNMEAGPVTG->callbackCopy != NULL)
    {
      delete storageNMEAGPVTG->callbackCopy;
    }
    delete storageNMEAGPVTG;
    storageNMEAGPVTG = NULL; // Redundant?
  }

  if (storageNMEAGNVTG != NULL)
  {
    if (storageNMEAGNVTG->callbackCopy != NULL)
    {
      delete storageNMEAGNVTG->callbackCopy;
    }
    delete storageNMEAGNVTG;
    storageNMEAGNVTG = NULL; // Redundant?
  }

  if (storageNMEAGPRMC != NULL)
  {
    if (storageNMEAGPRMC->callbackCopy != NULL)
    {
      delete storageNMEAGPRMC->callbackCopy;
    }
    delete storageNMEAGPRMC;
    storageNMEAGPRMC = NULL; // Redundant?
  }

  if (storageNMEAGNRMC != NULL)
  {
    if (storageNMEAGNRMC->callbackCopy != NULL)
    {
      delete storageNMEAGNRMC->callbackCopy;
    }
    delete storageNMEAGNRMC;
    storageNMEAGNRMC = NULL; // Redundant?
  }

  if (storageNMEAGPZDA != NULL)
  {
    if (storageNMEAGPZDA->callbackCopy != NULL)
    {
      delete storageNMEAGPZDA->callbackCopy;
    }
    delete storageNMEAGPZDA;
    storageNMEAGPZDA = NULL; // Redundant?
  }

  if (storageNMEAGNZDA != NULL)
  {
    if (storageNMEAGNZDA->callbackCopy != NULL)
    {
      delete storageNMEAGNZDA->callbackCopy;
    }
    delete storageNMEAGNZDA;
    storageNMEAGNZDA = NULL; // Redundant?
  }
#endif
}

// Allow the user to change packetCfgPayloadSize. Handy if you want to process big messages like RAWX
// This can be called before .begin if required / desired
bool SFE_UBLOX_GNSS::setPacketCfgPayloadSize(size_t payloadSize)
{
  bool success = true;

  if ((payloadSize == 0) && (payloadCfg != NULL))
  {
    // Zero payloadSize? Dangerous! But we'll free the memory anyway...
    delete[] payloadCfg; // Created with new[]
    payloadCfg = NULL;   // Redundant?
    packetCfg.payload = payloadCfg;
    packetCfgPayloadSize = payloadSize;
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setPacketCfgPayloadSize: Zero payloadSize!"));
  }

  else if (payloadCfg == NULL) // Memory has not yet been allocated - so use new
  {
    payloadCfg = new uint8_t[payloadSize];
    packetCfg.payload = payloadCfg;
    if (payloadCfg == NULL)
    {
      success = false;
      packetCfgPayloadSize = 0;
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        _debugSerial->println(F("setPacketCfgPayloadSize: RAM alloc failed!"));
    }
    else
      packetCfgPayloadSize = payloadSize;
  }

  else // Memory has already been allocated - so resize
  {
    uint8_t *newPayload = new uint8_t[payloadSize];

    if (newPayload == NULL) // Check if the alloc was successful
    {
      success = false;                                           // Report failure. Don't change payloadCfg, packetCfg.payload or packetCfgPayloadSize
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        _debugSerial->println(F("setPacketCfgPayloadSize: RAM resize failed!"));
    }
    else
    {
      memcpy(newPayload, payloadCfg, payloadSize <= packetCfgPayloadSize ? payloadSize : packetCfgPayloadSize); // Copy as much existing data as we can
      delete[] payloadCfg;                                                                                      // Free payloadCfg. Created with new[]
      payloadCfg = newPayload;                                                                                  // Point to the newPayload
      packetCfg.payload = payloadCfg;                                                                           // Update the packet pointer
      packetCfgPayloadSize = payloadSize;                                                                       // Update the packet payload size
    }
  }

  return (success);
}

// Return the number of free bytes remaining in packetCfgPayload
size_t SFE_UBLOX_GNSS::getPacketCfgSpaceRemaining()
{
  return (packetCfgPayloadSize - packetCfg.len);
}

// Initialize the I2C port
bool SFE_UBLOX_GNSS::begin(TwoWire &wirePort, uint8_t deviceAddress, uint16_t maxWait, bool assumeSuccess)
{
  commType = COMM_TYPE_I2C;
  _i2cPort = &wirePort; // Grab which port the user wants us to use
  _signsOfLife = false; // Clear the _signsOfLife flag. It will be set true if valid traffic is seen.

  // We expect caller to begin their I2C port, with the speed of their choice external to the library
  // But if they forget, we start the hardware here.

  // We're moving away from the practice of starting Wire hardware in a library. This is to avoid cross platform issues.
  // ie, there are some platforms that don't handle multiple starts to the wire hardware. Also, every time you start the wire
  // hardware the clock speed reverts back to 100kHz regardless of previous Wire.setClocks().
  //_i2cPort->begin();

  _gpsI2Caddress = deviceAddress; // Store the I2C address from user

  // New in v2.0: allocate memory for the packetCfg payload here - if required. (The user may have called setPacketCfgPayloadSize already)
  if (packetCfgPayloadSize == 0)
    setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  // New in v2.0: allocate memory for the file buffer - if required. (The user should have called setFileBufferSize already)
  createFileBuffer();

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - second attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - third attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess && _signsOfLife) // Advanced users can assume success if required. Useful if the port is outputting messages at high navigation rate.
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: third attempt failed. Assuming success..."));
    }
#endif
    return (true);
  }

  return (connected);
}

// Initialize the Serial port
bool SFE_UBLOX_GNSS::begin(Stream &serialPort, uint16_t maxWait, bool assumeSuccess)
{
  commType = COMM_TYPE_SERIAL;
  _serialPort = &serialPort; // Grab which port the user wants us to use
  _signsOfLife = false;      // Clear the _signsOfLife flag. It will be set true if valid traffic is seen.

  // New in v2.0: allocate memory for the packetCfg payload here - if required. (The user may have called setPacketCfgPayloadSize already)
  if (packetCfgPayloadSize == 0)
    setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  // New in v2.0: allocate memory for the file buffer - if required. (The user should have called setFileBufferSize already)
  createFileBuffer();

  // Get rid of any stale serial data already in the processor's RX buffer
  while (_serialPort->available())
    _serialPort->read();

  // If assumeSuccess is true, the user must really want begin to succeed. So, let's empty the module's serial transmit buffer too!
  // Keep discarding new serial data until we see a gap of 2ms - hopefully indicating that the module's TX buffer is empty.
  if (assumeSuccess)
  {
    unsigned long startTime = millis();
    unsigned long lastActivity = startTime;
    bool keepGoing = true;
    while (keepGoing && (millis() < (startTime + (unsigned long)maxWait)))
    {
      while (_serialPort->available()) // Discard any new data
      {
        _serialPort->read();
        lastActivity = millis();
      }

      if (millis() > (lastActivity + (unsigned long)2)) // Check if we have seen no new data for at least 2ms
        keepGoing = false;
    }
  }

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - second attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - third attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess && _signsOfLife) // Advanced users can assume success if required. Useful if the port is outputting messages at high navigation rate.
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: third attempt failed. Assuming success..."));
    }
#endif
    return (true);
  }

  return (connected);
}

// Initialize for SPI
bool SFE_UBLOX_GNSS::begin(SPIClass &spiPort, uint8_t csPin, uint32_t spiSpeed, uint16_t maxWait, bool assumeSuccess)
{
  commType = COMM_TYPE_SPI;
  _spiPort = &spiPort;
  _csPin = csPin;
  _spiSpeed = spiSpeed;
  _signsOfLife = false; // Clear the _signsOfLife flag. It will be set true if valid traffic is seen.

  // Initialize the chip select pin
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);

  // New in v2.0: allocate memory for the packetCfg payload here - if required. (The user may have called setPacketCfgPayloadSize already)
  if (packetCfgPayloadSize == 0)
    setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  createFileBuffer();

  // Create the SPI buffer
  if (spiBuffer == NULL) // Memory has not yet been allocated - so use new
  {
    spiBuffer = new uint8_t[getSpiTransactionSize()];
  }

  if (spiBuffer == NULL)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->print(F("begin (SPI): memory allocation failed for SPI Buffer!"));
      return (false);
    }
  }
  else
  {
    // Initialize/clear the SPI buffer - fill it with 0xFF as this is what is received from the UBLOX module if there's no data to be processed
    for (uint8_t i = 0; i < getSpiTransactionSize(); i++)
    {
      spiBuffer[i] = 0xFF;
    }
  }

  // Call isConnected up to three times - tests on the NEO-M8U show the CFG RATE poll occasionally being ignored
  bool connected = isConnected(maxWait);

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - second attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if (!connected)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: isConnected - third attempt"));
    }
#endif
    connected = isConnected(maxWait);
  }

  if ((!connected) && assumeSuccess && _signsOfLife) // Advanced users can assume success if required. Useful if the port is outputting messages at high navigation rate.
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("begin: third attempt failed. Assuming success..."));
    }
#endif
    return (true);
  }

  return (connected);
}

// Allow the user to change I2C polling wait (the minimum interval between I2C data requests - to avoid pounding the bus)
// i2cPollingWait defaults to 100ms and is adjusted automatically when setNavigationFrequency()
// or setHNRNavigationRate() are called. But if the user is using callbacks, it might be advantageous
// to be able to set the polling wait manually.
void SFE_UBLOX_GNSS::setI2CpollingWait(uint8_t newPollingWait_ms)
{
  i2cPollingWait = newPollingWait_ms;
}

// Allow the user to change SPI polling wait
// (the minimum interval between SPI data requests when no data is available - to avoid pounding the bus)
void SFE_UBLOX_GNSS::setSPIpollingWait(uint8_t newPollingWait_ms)
{
  spiPollingWait = newPollingWait_ms;
}

// Sets the global size for I2C transactions
// Most platforms use 32 bytes (the default) but this allows users to increase the transaction
// size if the platform supports it
// Note: If the transaction size is set larger than the platforms buffer size, bad things will happen.
void SFE_UBLOX_GNSS::setI2CTransactionSize(uint8_t transactionSize)
{
  if (transactionSize < 8)
    transactionSize = 8; // Ensure transactionSize is at least 8 bytes otherwise sendI2cCommand will have problems!

  i2cTransactionSize = transactionSize;
}
uint8_t SFE_UBLOX_GNSS::getI2CTransactionSize(void)
{
  return (i2cTransactionSize);
}

// Sets the global size for the SPI buffer/transactions.
// Call this **before** begin()!
// Note: if the buffer size is too small, incoming characters may be lost if the message sent
// is larger than this buffer. If too big, you may run out of SRAM on constrained architectures!
void SFE_UBLOX_GNSS::setSpiTransactionSize(uint8_t transactionSize)
{
  if (spiBuffer == NULL)
  {
    spiTransactionSize = transactionSize;
  }
  else
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("setSpiTransactionSize: you need to call setSpiTransactionSize _before_ begin!"));
    }
#endif
  }
}
uint8_t SFE_UBLOX_GNSS::getSpiTransactionSize(void)
{
  return (spiTransactionSize);
}

// Sets the size of maxNMEAByteCount
void SFE_UBLOX_GNSS::setMaxNMEAByteCount(int8_t newMax)
{
  maxNMEAByteCount = newMax;
}
int8_t SFE_UBLOX_GNSS::getMaxNMEAByteCount(void)
{
  return (maxNMEAByteCount);
}

// Returns true if I2C device ack's
bool SFE_UBLOX_GNSS::isConnected(uint16_t maxWait)
{
  if (commType == COMM_TYPE_I2C)
  {
    _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress);
    if (_i2cPort->endTransmission() != 0)
      return false; // Sensor did not ack
  }

  // Query port configuration to see whether we get a meaningful response
  // We could simply request the config for any port but, just for giggles, let's request the config for most appropriate port
  if (commType == COMM_TYPE_I2C)
    return (getPortSettingsInternal(COM_PORT_I2C, maxWait));
  else if (commType == COMM_TYPE_SERIAL)
    return (getPortSettingsInternal(COM_PORT_UART1, maxWait)); // Could be UART2 - but this is just a response check
  else                                                         // if (commType == COMM_TYPE_SPI)
    return (getPortSettingsInternal(COM_PORT_SPI, maxWait));
}

// Enable or disable the printing of sent/response HEX values.
// Use this in conjunction with 'Transport Logging' from the Universal Reader Assistant to see what they're doing that we're not
void SFE_UBLOX_GNSS::enableDebugging(Stream &debugPort, bool printLimitedDebug)
{
  _debugSerial = &debugPort; // Grab which port the user wants us to use for debugging
  if (printLimitedDebug == false)
  {
    _printDebug = true; // Should we print the commands we send? Good for debugging
  }
  else
  {
    _printLimitedDebug = true; // Should we print limited debug messages? Good for debugging high navigation rates
  }
}
void SFE_UBLOX_GNSS::disableDebugging(void)
{
  _printDebug = false; // Turn off extra print statements
  _printLimitedDebug = false;
}

// Safely print messages
void SFE_UBLOX_GNSS::debugPrint(char *message)
{
  if (_printDebug == true)
  {
    _debugSerial->print(message);
  }
}
// Safely print messages
void SFE_UBLOX_GNSS::debugPrintln(char *message)
{
  if (_printDebug == true)
  {
    _debugSerial->println(message);
  }
}

const char *SFE_UBLOX_GNSS::statusString(sfe_ublox_status_e stat)
{
  switch (stat)
  {
  case SFE_UBLOX_STATUS_SUCCESS:
    return "Success";
    break;
  case SFE_UBLOX_STATUS_FAIL:
    return "General Failure";
    break;
  case SFE_UBLOX_STATUS_CRC_FAIL:
    return "CRC Fail";
    break;
  case SFE_UBLOX_STATUS_TIMEOUT:
    return "Timeout";
    break;
  case SFE_UBLOX_STATUS_COMMAND_NACK:
    return "Command not acknowledged (NACK)";
    break;
  case SFE_UBLOX_STATUS_OUT_OF_RANGE:
    return "Out of range";
    break;
  case SFE_UBLOX_STATUS_INVALID_ARG:
    return "Invalid Arg";
    break;
  case SFE_UBLOX_STATUS_INVALID_OPERATION:
    return "Invalid operation";
    break;
  case SFE_UBLOX_STATUS_MEM_ERR:
    return "Memory Error";
    break;
  case SFE_UBLOX_STATUS_HW_ERR:
    return "Hardware Error";
    break;
  case SFE_UBLOX_STATUS_DATA_SENT:
    return "Data Sent";
    break;
  case SFE_UBLOX_STATUS_DATA_RECEIVED:
    return "Data Received";
    break;
  case SFE_UBLOX_STATUS_I2C_COMM_FAILURE:
    return "I2C Comm Failure";
    break;
  case SFE_UBLOX_STATUS_DATA_OVERWRITTEN:
    return "Data Packet Overwritten";
    break;
  default:
    return "Unknown Status";
    break;
  }
  return "None";
}

// Check for the arrival of new I2C/Serial/SPI data

// Allow the user to disable the "7F" check (e.g.) when logging RAWX data
void SFE_UBLOX_GNSS::disableUBX7Fcheck(bool disabled)
{
  ubx7FcheckDisabled = disabled;
}

// Called regularly to check for available bytes on the user' specified port
bool SFE_UBLOX_GNSS::checkUblox(uint8_t requestedClass, uint8_t requestedID)
{
  return checkUbloxInternal(&packetCfg, requestedClass, requestedID);
}

// PRIVATE: Called regularly to check for available bytes on the user' specified port
bool SFE_UBLOX_GNSS::checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (commType == COMM_TYPE_I2C)
    return (checkUbloxI2C(incomingUBX, requestedClass, requestedID));
  else if (commType == COMM_TYPE_SERIAL)
    return (checkUbloxSerial(incomingUBX, requestedClass, requestedID));
  else if (commType == COMM_TYPE_SPI)
    return (checkUbloxSpi(incomingUBX, requestedClass, requestedID));
  return false;
}

// Polls I2C for data, passing any new bytes to process()
// Returns true if new bytes are available
bool SFE_UBLOX_GNSS::checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (millis() - lastCheck >= i2cPollingWait)
  {
    // Get the number of bytes available from the module
    uint16_t bytesAvailable = 0;
    _i2cPort->beginTransmission(_gpsI2Caddress);
    _i2cPort->write(0xFD);                               // 0xFD (MSB) and 0xFE (LSB) are the registers that contain number of bytes available
    uint8_t i2cError = _i2cPort->endTransmission(false); // Always send a restart command. Do not release the bus. ESP32 supports this.
    if (i2cError != 0)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial->print(F("checkUbloxI2C: I2C error: endTransmission returned "));
        _debugSerial->println(i2cError);
      }
#endif
      return (false); // Sensor did not ACK
    }

    // Forcing requestFrom to use a restart would be unwise. If bytesAvailable is zero, we want to surrender the bus.
    uint8_t bytesReturned = _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, static_cast<uint8_t>(2));
    if (bytesReturned != 2)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial->print(F("checkUbloxI2C: I2C error: requestFrom 0xFD returned "));
        _debugSerial->println(bytesReturned);
      }
#endif
      return (false); // Sensor did not return 2 bytes
    }
    else // if (_i2cPort->available())
    {
      uint8_t msb = _i2cPort->read();
      uint8_t lsb = _i2cPort->read();
      // if (lsb == 0xFF)
      // {
      //   //I believe this is a u-blox bug. Device should never present an 0xFF.
      //   if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      //   {
      //     _debugSerial->print(F("checkUbloxI2C: u-blox bug? Length lsb is 0xFF. i2cPollingWait is "));
      //     _debugSerial->println(i2cPollingWait);
      //   }
      //   if (debugPin >= 0)
      //   {
      //     digitalWrite((uint8_t)debugPin, LOW);
      //     delay(10);
      //     digitalWrite((uint8_t)debugPin, HIGH);
      //   }
      //   lastCheck = millis(); //Put off checking to avoid I2C bus traffic
      //   return (false);
      // }
      // if (msb == 0xFF)
      // {
      //   //I believe this is a u-blox bug. Device should never present an 0xFF.
      //   if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      //   {
      //     _debugSerial->print(F("checkUbloxI2C: u-blox bug? Length msb is 0xFF. i2cPollingWait is "));
      //     _debugSerial->println(i2cPollingWait);
      //   }
      //   if (debugPin >= 0)
      //   {
      //     digitalWrite((uint8_t)debugPin, LOW);
      //     delay(10);
      //     digitalWrite((uint8_t)debugPin, HIGH);
      //   }
      //   lastCheck = millis(); //Put off checking to avoid I2C bus traffic
      //   return (false);
      // }
      bytesAvailable = (uint16_t)msb << 8 | lsb;
    }

    if (bytesAvailable == 0)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("checkUbloxI2C: OK, zero bytes available"));
      }
#endif
      lastCheck = millis(); // Put off checking to avoid I2C bus traffic
      return (false);
    }

    // Check for undocumented bit error. We found this doing logic scans.
    // This error is rare but if we incorrectly interpret the first bit of the two 'data available' bytes as 1
    // then we have far too many bytes to check. May be related to I2C setup time violations: https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40
    if (bytesAvailable & ((uint16_t)1 << 15))
    {
      // Clear the MSbit
      bytesAvailable &= ~((uint16_t)1 << 15);

      // if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      // {
      //   _debugSerial->print(F("checkUbloxI2C: Bytes available error: "));
      //   _debugSerial->println(bytesAvailable);
      //   if (debugPin >= 0)
      //   {
      //     digitalWrite((uint8_t)debugPin, LOW);
      //     delay(10);
      //     digitalWrite((uint8_t)debugPin, HIGH);
      //   }
      // }
    }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (bytesAvailable > 100)
    {
      if (_printDebug == true)
      {
        _debugSerial->print(F("checkUbloxI2C: Large packet of "));
        _debugSerial->print(bytesAvailable);
        _debugSerial->println(F(" bytes received"));
      }
    }
    else
    {
      if (_printDebug == true)
      {
        _debugSerial->print(F("checkUbloxI2C: Reading "));
        _debugSerial->print(bytesAvailable);
        _debugSerial->println(F(" bytes"));
      }
    }
#endif

    while (bytesAvailable)
    {
      // From the u-blox integration manual:
      // "There are two forms of DDC read transfer. The "random access" form includes a peripheral register
      //  address and thus allows any register to be read. The second "current address" form omits the
      //  register address. If this second form is used, then an address pointer in the receiver is used to
      //  determine which register to read. This address pointer will increment after each read unless it
      //  is already pointing at register 0xFF, the highest addressable register, in which case it remains
      //  unaltered."
      // This means that after reading bytesAvailable from 0xFD and 0xFE, the address pointer will already be
      // pointing at 0xFF, so we do not need to write it here. The next four lines can be commented.
      //_i2cPort->beginTransmission(_gpsI2Caddress);
      //_i2cPort->write(0xFF);                     //0xFF is the register to read data from
      // if (_i2cPort->endTransmission(false) != 0) //Send a restart command. Do not release bus.
      //  return (false);                          //Sensor did not ACK

      // Limit to 32 bytes or whatever the buffer limit is for given platform
      uint16_t bytesToRead = bytesAvailable; // 16-bit
      if (bytesToRead > i2cTransactionSize)  // Limit for i2cTransactionSize is 8-bit
        bytesToRead = i2cTransactionSize;

      // TRY_AGAIN:

      // Here it would be desireable to use a restart where possible / supported, but only if there will be multiple reads.
      // However, if an individual requestFrom fails, we could end up leaving the bus hanging.
      // On balance, it is probably safest to not use restarts here.
      uint8_t bytesReturned = _i2cPort->requestFrom((uint8_t)_gpsI2Caddress, (uint8_t)bytesToRead);
      if ((uint16_t)bytesReturned == bytesToRead)
      {
        for (uint16_t x = 0; x < bytesToRead; x++)
        {
          uint8_t incoming = _i2cPort->read(); // Grab the actual character

          // Check to see if the first read is 0x7F. If it is, the module is not ready to respond. Stop, wait, and try again.
          // Note: the integration manual says:
          //"If there is no data awaiting transmission from the receiver, then this register will deliver the value 0xFF,
          //  which cannot be the first byte of a valid message."
          // But it can be the first byte waiting to be read from the buffer if we have already read part of the message.
          // Therefore I think this check needs to be commented.
          //  if (x == 0)
          //  {
          //    if ((incoming == 0x7F) && (ubx7FcheckDisabled == false))
          //    {
          //      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          //      {
          //        _debugSerial->println(F("checkUbloxU2C: u-blox error, module not ready with data (7F error)"));
          //      }
          //      delay(5); //In logic analyzation, the module starting responding after 1.48ms
          //      if (debugPin >= 0)
          //      {
          //        digitalWrite((uint8_t)debugPin, LOW);
          //        delay(10);
          //        digitalWrite((uint8_t)debugPin, HIGH);
          //      }
          //      goto TRY_AGAIN;
          //    }
          //  }

          process(incoming, incomingUBX, requestedClass, requestedID); // Process this valid character
        }
      }
      else
        return (false); // Sensor did not respond

      bytesAvailable -= bytesToRead;
    }
  }

  return (true);

} // end checkUbloxI2C()

// Checks Serial for data, passing any new bytes to process()
bool SFE_UBLOX_GNSS::checkUbloxSerial(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  while (_serialPort->available())
  {
    process(_serialPort->read(), incomingUBX, requestedClass, requestedID);
  }
  return (true);

} // end checkUbloxSerial()

// Checks SPI for data, passing any new bytes to process()
bool SFE_UBLOX_GNSS::checkUbloxSpi(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  // Process the contents of the SPI buffer if not empty!
  for (uint8_t i = 0; i < spiBufferIndex; i++)
  {
    process(spiBuffer[i], incomingUBX, requestedClass, requestedID);
  }
  spiBufferIndex = 0;

  _spiPort->beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  uint8_t byteReturned = _spiPort->transfer(0xFF);

  // Note to future self: I think the 0xFF check might cause problems when attempting to process (e.g.) RAWX data
  // which could legitimately contain 0xFF within the data stream. But the currentSentence check will certainly help!

  // If we are not receiving a sentence (currentSentence == NONE) and the byteReturned is 0xFF,
  // i.e. the module has no data for us, then delay for
  if ((byteReturned == 0xFF) && (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NONE))
  {
    digitalWrite(_csPin, HIGH);
    _spiPort->endTransaction();
    delay(spiPollingWait);
    return (true);
  }

  while ((byteReturned != 0xFF) || (currentSentence != SFE_UBLOX_SENTENCE_TYPE_NONE))
  {
    process(byteReturned, incomingUBX, requestedClass, requestedID);
    byteReturned = _spiPort->transfer(0xFF);
  }
  digitalWrite(_csPin, HIGH);
  _spiPort->endTransaction();
  return (true);

} // end checkUbloxSpi()

// PRIVATE: Check if we have storage allocated for an incoming "automatic" message
bool SFE_UBLOX_GNSS::checkAutomatic(uint8_t Class, uint8_t ID)
{
  bool result = false;
  switch (Class)
  {
  case UBX_CLASS_NAV:
  {
    switch (ID)
    {
    case UBX_NAV_POSECEF:
      if (packetUBXNAVPOSECEF != NULL)
        result = true;
      break;
    case UBX_NAV_STATUS:
      if (packetUBXNAVSTATUS != NULL)
        result = true;
      break;
    case UBX_NAV_DOP:
      if (packetUBXNAVDOP != NULL)
        result = true;
      break;
    case UBX_NAV_ATT:
      if (packetUBXNAVATT != NULL)
        result = true;
      break;
    case UBX_NAV_PVT:
      if (packetUBXNAVPVT != NULL)
        result = true;
      break;
    case UBX_NAV_ODO:
      if (packetUBXNAVODO != NULL)
        result = true;
      break;
    case UBX_NAV_VELECEF:
      if (packetUBXNAVVELECEF != NULL)
        result = true;
      break;
    case UBX_NAV_VELNED:
      if (packetUBXNAVVELNED != NULL)
        result = true;
      break;
    case UBX_NAV_HPPOSECEF:
      if (packetUBXNAVHPPOSECEF != NULL)
        result = true;
      break;
    case UBX_NAV_HPPOSLLH:
      if (packetUBXNAVHPPOSLLH != NULL)
        result = true;
      break;
    case UBX_NAV_PVAT:
      if (packetUBXNAVPVAT != NULL)
        result = true;
      break;
    case UBX_NAV_TIMEUTC:
      if (packetUBXNAVTIMEUTC != NULL)
        result = true;
      break;
    case UBX_NAV_CLOCK:
      if (packetUBXNAVCLOCK != NULL)
        result = true;
      break;
    case UBX_NAV_TIMELS:
      if (packetUBXNAVTIMELS != NULL)
        result = true;
      break;
    case UBX_NAV_SVIN:
      if (packetUBXNAVSVIN != NULL)
        result = true;
      break;
    case UBX_NAV_SAT:
      if (packetUBXNAVSAT != NULL)
        result = true;
      break;
    case UBX_NAV_RELPOSNED:
      if (packetUBXNAVRELPOSNED != NULL)
        result = true;
      break;
    case UBX_NAV_AOPSTATUS:
      if (packetUBXNAVAOPSTATUS != NULL)
        result = true;
      break;
    case UBX_NAV_EOE:
      if (packetUBXNAVEOE != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_RXM:
  {
    switch (ID)
    {
    case UBX_RXM_SFRBX:
      if (packetUBXRXMSFRBX != NULL)
        result = true;
      break;
    case UBX_RXM_RAWX:
      if (packetUBXRXMRAWX != NULL)
        result = true;
      break;
    case UBX_RXM_PMP:
      if ((packetUBXRXMPMP != NULL) || (packetUBXRXMPMPmessage != NULL))
        result = true;
      break;
    case UBX_RXM_QZSSL6:
      if (packetUBXRXMQZSSL6message != NULL)
        result = true;
      break;
    case UBX_RXM_COR:
      if (packetUBXRXMCOR != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_CFG:
  {
    switch (ID)
    {
    case UBX_CFG_PRT:
      if (packetUBXCFGPRT != NULL)
        result = true;
      break;
    case UBX_CFG_RATE:
      if (packetUBXCFGRATE != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_TIM:
  {
    switch (ID)
    {
    case UBX_TIM_TM2:
      if (packetUBXTIMTM2 != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_ESF:
  {
    switch (ID)
    {
    case UBX_ESF_ALG:
      if (packetUBXESFALG != NULL)
        result = true;
      break;
    case UBX_ESF_INS:
      if (packetUBXESFINS != NULL)
        result = true;
      break;
    case UBX_ESF_MEAS:
      if (packetUBXESFMEAS != NULL)
        result = true;
      break;
    case UBX_ESF_RAW:
      if (packetUBXESFRAW != NULL)
        result = true;
      break;
    case UBX_ESF_STATUS:
      if (packetUBXESFSTATUS != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_MGA:
  {
    switch (ID)
    {
    case UBX_MGA_ACK_DATA0:
      if (packetUBXMGAACK != NULL)
        result = true;
      break;
    case UBX_MGA_DBD:
      if (packetUBXMGADBD != NULL)
        result = true;
      break;
    }
  }
  break;
  case UBX_CLASS_HNR:
  {
    switch (ID)
    {
    case UBX_HNR_PVT:
      if (packetUBXHNRPVT != NULL)
        result = true;
      break;
    case UBX_HNR_ATT:
      if (packetUBXHNRATT != NULL)
        result = true;
      break;
    case UBX_HNR_INS:
      if (packetUBXHNRINS != NULL)
        result = true;
      break;
    }
  }
  break;
  }
  return (result);
}

// PRIVATE: Calculate how much RAM is needed to store the payload for a given automatic message
uint16_t SFE_UBLOX_GNSS::getMaxPayloadSize(uint8_t Class, uint8_t ID)
{
  uint16_t maxSize = 0;
  switch (Class)
  {
  case UBX_CLASS_NAV:
  {
    switch (ID)
    {
    case UBX_NAV_POSECEF:
      maxSize = UBX_NAV_POSECEF_LEN;
      break;
    case UBX_NAV_STATUS:
      maxSize = UBX_NAV_STATUS_LEN;
      break;
    case UBX_NAV_DOP:
      maxSize = UBX_NAV_DOP_LEN;
      break;
    case UBX_NAV_ATT:
      maxSize = UBX_NAV_ATT_LEN;
      break;
    case UBX_NAV_PVT:
      maxSize = UBX_NAV_PVT_LEN;
      break;
    case UBX_NAV_ODO:
      maxSize = UBX_NAV_ODO_LEN;
      break;
    case UBX_NAV_VELECEF:
      maxSize = UBX_NAV_VELECEF_LEN;
      break;
    case UBX_NAV_VELNED:
      maxSize = UBX_NAV_VELNED_LEN;
      break;
    case UBX_NAV_HPPOSECEF:
      maxSize = UBX_NAV_HPPOSECEF_LEN;
      break;
    case UBX_NAV_HPPOSLLH:
      maxSize = UBX_NAV_HPPOSLLH_LEN;
      break;
    case UBX_NAV_PVAT:
      maxSize = UBX_NAV_PVAT_LEN;
      break;
    case UBX_NAV_TIMEUTC:
      maxSize = UBX_NAV_TIMEUTC;
      break;
    case UBX_NAV_CLOCK:
      maxSize = UBX_NAV_CLOCK_LEN;
      break;
    case UBX_NAV_TIMELS:
      maxSize = UBX_NAV_TIMELS_LEN;
      break;
    case UBX_NAV_SVIN:
      maxSize = UBX_NAV_SVIN_LEN;
      break;
    case UBX_NAV_SAT:
      maxSize = UBX_NAV_SAT_MAX_LEN;
      break;
    case UBX_NAV_RELPOSNED:
      maxSize = UBX_NAV_RELPOSNED_LEN_F9;
      break;
    case UBX_NAV_AOPSTATUS:
      maxSize = UBX_NAV_AOPSTATUS_LEN;
      break;
    case UBX_NAV_EOE:
      maxSize = UBX_NAV_EOE_LEN;
      break;
    }
  }
  break;
  case UBX_CLASS_RXM:
  {
    switch (ID)
    {
    case UBX_RXM_SFRBX:
      maxSize = UBX_RXM_SFRBX_MAX_LEN;
      break;
    case UBX_RXM_RAWX:
      maxSize = UBX_RXM_RAWX_MAX_LEN;
      break;
    case UBX_RXM_PMP:
      maxSize = UBX_RXM_PMP_MAX_LEN;
      break;
    case UBX_RXM_QZSSL6:
      maxSize = UBX_RXM_QZSSL6_MAX_LEN;
      break;
    case UBX_RXM_COR:
      maxSize = UBX_RXM_COR_LEN;
      break;
    }
  }
  break;
  case UBX_CLASS_CFG:
  {
    switch (ID)
    {
    case UBX_CFG_PRT:
      maxSize = UBX_CFG_PRT_LEN;
      break;
    case UBX_CFG_RATE:
      maxSize = UBX_CFG_RATE_LEN;
      break;
    }
  }
  break;
  case UBX_CLASS_TIM:
  {
    switch (ID)
    {
    case UBX_TIM_TM2:
      maxSize = UBX_TIM_TM2_LEN;
      break;
    }
  }
  break;
  case UBX_CLASS_ESF:
  {
    switch (ID)
    {
    case UBX_ESF_ALG:
      maxSize = UBX_ESF_ALG_LEN;
      break;
    case UBX_ESF_INS:
      maxSize = UBX_ESF_INS_LEN;
      break;
    case UBX_ESF_MEAS:
      maxSize = UBX_ESF_MEAS_MAX_LEN;
      break;
    case UBX_ESF_RAW:
      maxSize = UBX_ESF_RAW_MAX_LEN;
      break;
    case UBX_ESF_STATUS:
      maxSize = UBX_ESF_STATUS_MAX_LEN;
      break;
    }
  }
  break;
  case UBX_CLASS_MGA:
  {
    switch (ID)
    {
    case UBX_MGA_ACK_DATA0:
      maxSize = UBX_MGA_ACK_DATA0_LEN;
      break;
    case UBX_MGA_DBD:
      maxSize = UBX_MGA_DBD_LEN; // UBX_MGA_DBD_LEN is actually a maximum length. The packets could be shorter than this.
      break;
    }
  }
  break;
  case UBX_CLASS_HNR:
  {
    switch (ID)
    {
    case UBX_HNR_PVT:
      maxSize = UBX_HNR_PVT_LEN;
      break;
    case UBX_HNR_ATT:
      maxSize = UBX_HNR_ATT_LEN;
      break;
    case UBX_HNR_INS:
      maxSize = UBX_HNR_INS_LEN;
      break;
    }
  }
  break;
  }
  return (maxSize);
}

// Processes NMEA and UBX binary sentences one byte at a time
// Take a given byte and file it into the proper array
void SFE_UBLOX_GNSS::process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  if (_outputPort != NULL)
    _outputPort->write(incoming); // Echo this byte to the serial port
  if ((currentSentence == SFE_UBLOX_SENTENCE_TYPE_NONE) || (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NMEA))
  {
    if (incoming == UBX_SYNCH_1) // UBX binary frames start with 0xB5, aka μ
    {
      // This is the start of a binary sentence. Reset flags.
      // We still don't know the response class
      ubxFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_UBX;
      // Reset the packetBuf.counter even though we will need to reset it again when ubxFrameCounter == 2
      packetBuf.counter = 0;
      ignoreThisPayload = false; // We should not ignore this payload - yet
      // Store data in packetBuf until we know if we have a requested class and ID match
      activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;
    }
    else if (incoming == '$')
    {
      nmeaByteCounter = 0; // Reset the NMEA byte counter
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NMEA;
    }
    else if (incoming == 0xD3) // RTCM frames start with 0xD3
    {
      rtcmFrameCounter = 0;
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_RTCM;
    }
    else
    {
      // This character is unknown or we missed the previous start of a sentence
    }
  }

  // Depending on the sentence, pass the character to the individual processor
  if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_UBX)
  {
    // Decide what type of response this is
    if ((ubxFrameCounter == 0) && (incoming != UBX_SYNCH_1))      // ISO 'μ'
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;             // Something went wrong. Reset.
    else if ((ubxFrameCounter == 1) && (incoming != UBX_SYNCH_2)) // ASCII 'b'
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;             // Something went wrong. Reset.
    // Note to future self:
    // There may be some duplication / redundancy in the next few lines as processUBX will also
    // load information into packetBuf, but we'll do it here too for clarity
    else if (ubxFrameCounter == 2) // Class
    {
      // Record the class in packetBuf until we know what to do with it
      packetBuf.cls = incoming; // (Duplication)
      rollingChecksumA = 0;     // Reset our rolling checksums here (not when we receive the 0xB5)
      rollingChecksumB = 0;
      packetBuf.counter = 0;                                   // Reset the packetBuf.counter (again)
      packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // Reset the packet validity (redundant?)
      packetBuf.startingSpot = incomingUBX->startingSpot;      // Copy the startingSpot
    }
    else if (ubxFrameCounter == 3) // ID
    {
      // Record the ID in packetBuf until we know what to do with it
      packetBuf.id = incoming; // (Duplication)
      // We can now identify the type of response
      // If the packet we are receiving is not an ACK then check for a class and ID match
      if (packetBuf.cls != UBX_CLASS_ACK)
      {
        // This is not an ACK so check for a class and ID match
        if ((packetBuf.cls == requestedClass) && (packetBuf.id == requestedID))
        {
          // This is not an ACK and we have a class and ID match
          // So start diverting data into incomingUBX (usually packetCfg)
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
          incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
          incomingUBX->id = packetBuf.id;
          incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
        }
        // This is not an ACK and we do not have a complete class and ID match
        // So let's check if this is an "automatic" message which has its own storage defined
        else if (checkAutomatic(packetBuf.cls, packetBuf.id))
        {
          // This is not the message we were expecting but it has its own storage and so we should process it anyway.
          // We'll try to use packetAuto to buffer the message (so it can't overwrite anything in packetCfg).
          // We need to allocate memory for the packetAuto payload (payloadAuto) - and delete it once
          // reception is complete.
          uint16_t maxPayload = getMaxPayloadSize(packetBuf.cls, packetBuf.id); // Calculate how much RAM we need
          if (maxPayload == 0)
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->print(F("process: getMaxPayloadSize returned ZERO!! Class: 0x"));
              _debugSerial->print(packetBuf.cls);
              _debugSerial->print(F(" ID: 0x"));
              _debugSerial->println(packetBuf.id);
            }
#endif
          }
          if (payloadAuto != NULL) // Check if memory is already allocated - this should be impossible!
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->println(F("process: memory is already allocated for payloadAuto! Deleting..."));
            }
#endif
            delete[] payloadAuto; // Created with new[]
            payloadAuto = NULL;   // Redundant?
            packetAuto.payload = payloadAuto;
          }
          payloadAuto = new uint8_t[maxPayload]; // Allocate RAM for payloadAuto
          packetAuto.payload = payloadAuto;
          if (payloadAuto == NULL) // Check if the alloc failed
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->print(F("process: memory allocation failed for \"automatic\" message: Class: 0x"));
              _debugSerial->print(packetBuf.cls, HEX);
              _debugSerial->print(F(" ID: 0x"));
              _debugSerial->println(packetBuf.id, HEX);
              _debugSerial->println(F("process: \"automatic\" message could overwrite data"));
            }
#endif
            // The RAM allocation failed so fall back to using incomingUBX (usually packetCfg) even though we risk overwriting data
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETCFG;
            incomingUBX->cls = packetBuf.cls; // Copy the class and ID into incomingUBX (usually packetCfg)
            incomingUBX->id = packetBuf.id;
            incomingUBX->counter = packetBuf.counter; // Copy over the .counter too
          }
          else
          {
            // The RAM allocation was successful so we start diverting data into packetAuto and process it
            activePacketBuffer = SFE_UBLOX_PACKET_PACKETAUTO;
            packetAuto.cls = packetBuf.cls; // Copy the class and ID into packetAuto
            packetAuto.id = packetBuf.id;
            packetAuto.counter = packetBuf.counter;           // Copy over the .counter too
            packetAuto.startingSpot = packetBuf.startingSpot; // And the starting spot? (Probably redundant)
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if (_printDebug == true)
            {
              _debugSerial->print(F("process: incoming \"automatic\" message: Class: 0x"));
              _debugSerial->print(packetBuf.cls, HEX);
              _debugSerial->print(F(" ID: 0x"));
              _debugSerial->println(packetBuf.id, HEX);
            }
#endif
          }
        }
        else
        {
          // This is not an ACK and we do not have a class and ID match
          // so we should keep diverting data into packetBuf and ignore the payload
          ignoreThisPayload = true;
        }
      }
      else
      {
        // This is an ACK so it is to early to do anything with it
        // We need to wait until we have received the length and data bytes
        // So we should keep diverting data into packetBuf
      }
    }
    else if (ubxFrameCounter == 4) // Length LSB
    {
      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len = incoming; // (Duplication)
    }
    else if (ubxFrameCounter == 5) // Length MSB
    {
      // We should save the length in packetBuf even if activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG
      packetBuf.len |= incoming << 8; // (Duplication)
    }
    else if (ubxFrameCounter == 6) // This should be the first byte of the payload unless .len is zero
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial->print(F("process: ZERO LENGTH packet received: Class: 0x"));
          _debugSerial->print(packetBuf.cls, HEX);
          _debugSerial->print(F(" ID: 0x"));
          _debugSerial->println(packetBuf.id, HEX);
        }
#endif
        // If length is zero (!) this will be the first byte of the checksum so record it
        packetBuf.checksumA = incoming;
      }
      else
      {
        // The length is not zero so record this byte in the payload
        packetBuf.payload[0] = incoming;
      }
    }
    else if (ubxFrameCounter == 7) // This should be the second byte of the payload unless .len is zero or one
    {
      if (packetBuf.len == 0) // Check if length is zero (hopefully this is impossible!)
      {
        // If length is zero (!) this will be the second byte of the checksum so record it
        packetBuf.checksumB = incoming;
      }
      else if (packetBuf.len == 1) // Check if length is one
      {
        // The length is one so this is the first byte of the checksum
        packetBuf.checksumA = incoming;
      }
      else // Length is >= 2 so this must be a payload byte
      {
        packetBuf.payload[1] = incoming;
      }
      // Now that we have received two payload bytes, we can check for a matching ACK/NACK
      if ((activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF) // If we are not already processing a data packet
          && (packetBuf.cls == UBX_CLASS_ACK)                // and if this is an ACK/NACK
          && (packetBuf.payload[0] == requestedClass)        // and if the class matches
          && (packetBuf.payload[1] == requestedID))          // and if the ID matches
      {
        if (packetBuf.len == 2) // Check if .len is 2
        {
          // Then this is a matching ACK so copy it into packetAck
          activePacketBuffer = SFE_UBLOX_PACKET_PACKETACK;
          packetAck.cls = packetBuf.cls;
          packetAck.id = packetBuf.id;
          packetAck.len = packetBuf.len;
          packetAck.counter = packetBuf.counter;
          packetAck.payload[0] = packetBuf.payload[0];
          packetAck.payload[1] = packetBuf.payload[1];
        }
        else // Length is not 2 (hopefully this is impossible!)
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial->print(F("process: ACK received with .len != 2: Class: 0x"));
            _debugSerial->print(packetBuf.payload[0], HEX);
            _debugSerial->print(F(" ID: 0x"));
            _debugSerial->print(packetBuf.payload[1], HEX);
            _debugSerial->print(F(" len: "));
            _debugSerial->println(packetBuf.len);
          }
#endif
        }
      }
    }

    // Divert incoming into the correct buffer
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETACK)
      processUBX(incoming, &packetAck, requestedClass, requestedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
      processUBX(incoming, incomingUBX, requestedClass, requestedID);
    else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETBUF)
      processUBX(incoming, &packetBuf, requestedClass, requestedID);
    else // if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
      processUBX(incoming, &packetAuto, requestedClass, requestedID);

    // Finally, increment the frame counter
    ubxFrameCounter++;
  }
  else if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_NMEA) // Process incoming NMEA mesages. Selectively log if desired.
  {
    if ((nmeaByteCounter == 0) && (incoming != '$'))
    {
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset. (Almost certainly redundant!)
    }
    else if ((nmeaByteCounter == 1) && (incoming != 'G'))
    {
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset.
    }
    else if ((nmeaByteCounter >= 0) && (nmeaByteCounter <= 5))
    {
      nmeaAddressField[nmeaByteCounter] = incoming; // Store the start character and NMEA address field
    }

    if (nmeaByteCounter == 5)
    {
      if (!_signsOfLife) // If _signsOfLife is not already true, set _signsOfLife to true if the NMEA header is valid
      {
        _signsOfLife = isNMEAHeaderValid();
      }

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      // Check if we have automatic storage for this message
      if (isThisNMEAauto())
      {
        uint8_t *lengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *nmeaPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();
        *lengthPtr = 6;                           // Set the working copy length
        memset(nmeaPtr, 0, nmeaMaxLength);        // Clear the working copy
        memcpy(nmeaPtr, &nmeaAddressField[0], 6); // Copy the start character and address field into the working copy
      }
      else
#endif
      {
        // if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        // {
        //   _debugSerial->println(F("process: non-auto NMEA message"));
        // }
      }

      // We've just received the end of the address field. Check if it is selected for logging
      if (logThisNMEA())
      {
        storeFileBytes(&nmeaAddressField[0], 6); // Add start character and address field to the file buffer
      }
      // Check if it should be passed to processNMEA
      if (processThisNMEA())
      {
        processNMEA(nmeaAddressField[0]); // Process the start character and address field
        processNMEA(nmeaAddressField[1]);
        processNMEA(nmeaAddressField[2]);
        processNMEA(nmeaAddressField[3]);
        processNMEA(nmeaAddressField[4]);
        processNMEA(nmeaAddressField[5]);
      }
    }

    if ((nmeaByteCounter > 5) || (nmeaByteCounter < 0)) // Should we add incoming to the file buffer and/or pass it to processNMEA?
    {
#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      if (isThisNMEAauto())
      {
        uint8_t *lengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *nmeaPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();
        if (*lengthPtr < nmeaMaxLength)
        {
          *(nmeaPtr + *lengthPtr) = incoming; // Store the character
          *lengthPtr = *lengthPtr + 1;        // Increment the length
          if (*lengthPtr == nmeaMaxLength)
          {
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->println(F("process: NMEA buffer is full!"));
            }
          }
        }
      }
#endif
      if (logThisNMEA())
        storeFileBytes(&incoming, 1); // Add incoming to the file buffer
      if (processThisNMEA())
        processNMEA(incoming); // Pass incoming to processNMEA
    }

    if (incoming == '*')
      nmeaByteCounter = -5; // We are expecting * plus two checksum bytes plus CR and LF

    nmeaByteCounter++; // Increment the byte counter

    if (nmeaByteCounter == maxNMEAByteCount)          // Check if we have processed too many bytes
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Something went wrong. Reset.

    if (nmeaByteCounter == 0) // Check if we are done
    {
#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
      if (isThisNMEAauto())
      {
        uint8_t *workingLengthPtr = getNMEAWorkingLengthPtr(); // Get a pointer to the working copy length
        uint8_t *workingNMEAPtr = getNMEAWorkingNMEAPtr();     // Get a pointer to the working copy NMEA data
        uint8_t nmeaMaxLength = getNMEAMaxLength();

        // Check the checksum: the checksum is the exclusive-OR of all characters between the $ and the *
        uint8_t nmeaChecksum = 0;
        uint8_t charsChecked = 1; // Start after the $
        uint8_t thisChar = '\0';
        while ((charsChecked < (nmeaMaxLength - 1)) && (charsChecked < ((*workingLengthPtr) - 4)) && (thisChar != '*'))
        {
          thisChar = *(workingNMEAPtr + charsChecked); // Get a char from the working copy
          if (thisChar != '*')                         // Ex-or the char into the checksum - but not if it is the '*'
            nmeaChecksum ^= thisChar;
          charsChecked++; // Increment the counter
        }
        if (thisChar == '*') // Make sure we found the *
        {
          uint8_t expectedChecksum1 = (nmeaChecksum >> 4) + '0';
          if (expectedChecksum1 >= ':') // Handle Hex correctly
            expectedChecksum1 += 'A' - ':';
          uint8_t expectedChecksum2 = (nmeaChecksum & 0x0F) + '0';
          if (expectedChecksum2 >= ':') // Handle Hex correctly
            expectedChecksum2 += 'A' - ':';
          if ((expectedChecksum1 == *(workingNMEAPtr + charsChecked)) && (expectedChecksum2 == *(workingNMEAPtr + charsChecked + 1)))
          {
            uint8_t *completeLengthPtr = getNMEACompleteLengthPtr();    // Get a pointer to the complete copy length
            uint8_t *completeNMEAPtr = getNMEACompleteNMEAPtr();        // Get a pointer to the complete copy NMEA data
            memset(completeNMEAPtr, 0, nmeaMaxLength);                  // Clear the previous complete copy
            memcpy(completeNMEAPtr, workingNMEAPtr, *workingLengthPtr); // Copy the working copy into the complete copy
            *completeLengthPtr = *workingLengthPtr;                     // Update the length
            nmeaAutomaticFlags *flagsPtr = getNMEAFlagsPtr();           // Get a pointer to the flags
            nmeaAutomaticFlags flagsCopy = *flagsPtr;
            flagsCopy.flags.bits.completeCopyValid = 1; // Set the complete copy valid flag
            flagsCopy.flags.bits.completeCopyRead = 0;  // Clear the complete copy read flag
            *flagsPtr = flagsCopy;                      // Update the flags
            // Callback
            if (doesThisNMEAHaveCallback()) // Do we need to copy the data into the callback copy?
            {
              if (flagsCopy.flags.bits.callbackCopyValid == 0) // Has the callback copy valid flag been cleared (by checkCallbacks)
              {
                uint8_t *callbackLengthPtr = getNMEACallbackLengthPtr();    // Get a pointer to the callback copy length
                uint8_t *callbackNMEAPtr = getNMEACallbackNMEAPtr();        // Get a pointer to the callback copy NMEA data
                memset(callbackNMEAPtr, 0, nmeaMaxLength);                  // Clear the previous callback copy
                memcpy(callbackNMEAPtr, workingNMEAPtr, *workingLengthPtr); // Copy the working copy into the callback copy
                *callbackLengthPtr = *workingLengthPtr;                     // Update the length
                flagsCopy.flags.bits.callbackCopyValid = 1;                 // Set the callback copy valid flag
                *flagsPtr = flagsCopy;                                      // Update the flags
              }
            }
          }
          else
          {
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->print(F("process: NMEA checksum fail (2)! Expected "));
              _debugSerial->write(expectedChecksum1);
              _debugSerial->write(expectedChecksum2);
              _debugSerial->print(F(" Got "));
              _debugSerial->write(*(workingNMEAPtr + charsChecked));
              _debugSerial->write(*(workingNMEAPtr + charsChecked + 1));
              _debugSerial->println();
            }
          }
        }
        else
        {
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial->println(F("process: NMEA checksum fail (1)!"));
          }
        }
      }
#endif
      currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // All done!
    }
  }
  else if (currentSentence == SFE_UBLOX_SENTENCE_TYPE_RTCM)
  {
    currentSentence = processRTCMframe(incoming, &rtcmFrameCounter); // Deal with RTCM bytes
  }
}

// PRIVATE: Return true if we should add this NMEA message to the file buffer for logging
bool SFE_UBLOX_GNSS::logThisNMEA()
{
  if (_logNMEA.bits.all == 1)
    return (true);
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M') && (_logNMEA.bits.UBX_NMEA_DTM == 1))
    return (true);
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GAQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GBQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GBS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_GGA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L') && (_logNMEA.bits.UBX_NMEA_GLL == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GLQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GNQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GNS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GPQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q') && (_logNMEA.bits.UBX_NMEA_GQQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S') && (_logNMEA.bits.UBX_NMEA_GRS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_GSA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T') && (_logNMEA.bits.UBX_NMEA_GST == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V') && (_logNMEA.bits.UBX_NMEA_GSV == 1))
      return (true);
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M') && (_logNMEA.bits.UBX_NMEA_RLM == 1))
    return (true);
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C') && (_logNMEA.bits.UBX_NMEA_RMC == 1))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T') && (_logNMEA.bits.UBX_NMEA_TXT == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W') && (_logNMEA.bits.UBX_NMEA_VLW == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G') && (_logNMEA.bits.UBX_NMEA_VTG == 1))
    return (true);
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A') && (_logNMEA.bits.UBX_NMEA_ZDA == 1))
    return (true);
  return (false);
}

// PRIVATE: Return true if the NMEA header is valid
bool SFE_UBLOX_GNSS::isNMEAHeaderValid()
{
  if (nmeaAddressField[0] != '*')
    return (false);
  if (nmeaAddressField[1] != 'G')
    return (false);
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M'))
    return (true);
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A'))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L'))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q'))
      return (true);
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T'))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V'))
      return (true);
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M'))
    return (true);
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C'))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T'))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W'))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G'))
    return (true);
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A'))
    return (true);
  return (false);
}

// PRIVATE: Return true if we should pass this NMEA message to processNMEA
bool SFE_UBLOX_GNSS::processThisNMEA()
{
  if (_processNMEA.bits.all == 1)
    return (true);
  if ((nmeaAddressField[3] == 'D') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'M') && (_processNMEA.bits.UBX_NMEA_DTM == 1))
    return (true);
  if (nmeaAddressField[3] == 'G')
  {
    if ((nmeaAddressField[4] == 'A') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GAQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GBQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'B') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GBS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'G') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_GGA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'L') && (_processNMEA.bits.UBX_NMEA_GLL == 1))
      return (true);
    if ((nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GLQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GNQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'N') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GNS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'P') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GPQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'Q') && (nmeaAddressField[5] == 'Q') && (_processNMEA.bits.UBX_NMEA_GQQ == 1))
      return (true);
    if ((nmeaAddressField[4] == 'R') && (nmeaAddressField[5] == 'S') && (_processNMEA.bits.UBX_NMEA_GRS == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_GSA == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'T') && (_processNMEA.bits.UBX_NMEA_GST == 1))
      return (true);
    if ((nmeaAddressField[4] == 'S') && (nmeaAddressField[5] == 'V') && (_processNMEA.bits.UBX_NMEA_GSV == 1))
      return (true);
  }
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'M') && (_processNMEA.bits.UBX_NMEA_RLM == 1))
    return (true);
  if ((nmeaAddressField[3] == 'R') && (nmeaAddressField[4] == 'M') && (nmeaAddressField[5] == 'C') && (_processNMEA.bits.UBX_NMEA_RMC == 1))
    return (true);
  if ((nmeaAddressField[3] == 'T') && (nmeaAddressField[4] == 'X') && (nmeaAddressField[5] == 'T') && (_processNMEA.bits.UBX_NMEA_TXT == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'L') && (nmeaAddressField[5] == 'W') && (_processNMEA.bits.UBX_NMEA_VLW == 1))
    return (true);
  if ((nmeaAddressField[3] == 'V') && (nmeaAddressField[4] == 'T') && (nmeaAddressField[5] == 'G') && (_processNMEA.bits.UBX_NMEA_VTG == 1))
    return (true);
  if ((nmeaAddressField[3] == 'Z') && (nmeaAddressField[4] == 'D') && (nmeaAddressField[5] == 'A') && (_processNMEA.bits.UBX_NMEA_ZDA == 1))
    return (true);
  return (false);
}

// This is the default or generic NMEA processor. We're only going to pipe the data to serial port so we can see it.
// User could overwrite this function to pipe characters to nmea.process(c) of tinyGPS or MicroNMEA
// Or user could pipe each character to a buffer, radio, etc.
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  // If user has assigned an output port then pipe the characters there
  if (_nmeaOutputPort != NULL)
    _nmeaOutputPort->write(incoming); // Echo this byte to the serial port
}

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
// Check if the NMEA message (in nmeaAddressField) is "auto" (i.e. has RAM allocated for it)
bool SFE_UBLOX_GNSS::isThisNMEAauto()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPGGA != NULL)
      return true;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNGGA != NULL)
      return true;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPVTG != NULL)
      return true;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNVTG != NULL)
      return true;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPRMC != NULL)
      return true;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNRMC != NULL)
      return true;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPZDA != NULL)
      return true;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNZDA != NULL)
      return true;
  }

  return false;
}

// Do we need to copy the data into the callback copy?
bool SFE_UBLOX_GNSS::doesThisNMEAHaveCallback()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPGGA != NULL)
      if (storageNMEAGPGGA->callbackCopy != NULL)
        if ((storageNMEAGPGGA->callbackPointer != NULL) || (storageNMEAGPGGA->callbackPointerPtr != NULL))
          return true;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNGGA != NULL)
      if (storageNMEAGNGGA->callbackCopy != NULL)
        if ((storageNMEAGNGGA->callbackPointer != NULL) || (storageNMEAGNGGA->callbackPointerPtr != NULL))
          return true;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPVTG != NULL)
      if (storageNMEAGPVTG->callbackCopy != NULL)
        if ((storageNMEAGPVTG->callbackPointer != NULL) || (storageNMEAGPVTG->callbackPointerPtr != NULL))
          return true;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNVTG != NULL)
      if (storageNMEAGNVTG->callbackCopy != NULL)
        if ((storageNMEAGNVTG->callbackPointer != NULL) || (storageNMEAGNVTG->callbackPointerPtr != NULL))
          return true;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPRMC != NULL)
      if (storageNMEAGPRMC->callbackCopy != NULL)
        if ((storageNMEAGPRMC->callbackPointer != NULL) || (storageNMEAGPRMC->callbackPointerPtr != NULL))
          return true;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNRMC != NULL)
      if (storageNMEAGNRMC->callbackCopy != NULL)
        if ((storageNMEAGNRMC->callbackPointer != NULL) || (storageNMEAGNRMC->callbackPointerPtr != NULL))
          return true;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGPZDA != NULL)
      if (storageNMEAGPZDA->callbackCopy != NULL)
        if ((storageNMEAGPZDA->callbackPointer != NULL) || (storageNMEAGPZDA->callbackPointerPtr != NULL))
          return true;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    if (storageNMEAGNZDA != NULL)
      if (storageNMEAGNZDA->callbackCopy != NULL)
        if ((storageNMEAGNZDA->callbackPointer != NULL) || (storageNMEAGNZDA->callbackPointerPtr != NULL))
          return true;
  }

  return false;
}

// Get a pointer to the working copy length
uint8_t *SFE_UBLOX_GNSS::getNMEAWorkingLengthPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->workingCopy.length;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->workingCopy.length;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->workingCopy.length;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->workingCopy.length;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->workingCopy.length;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->workingCopy.length;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->workingCopy.length;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->workingCopy.length;
  }

  return NULL;
}

// Get a pointer to the working copy NMEA data
uint8_t *SFE_UBLOX_GNSS::getNMEAWorkingNMEAPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->workingCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->workingCopy.nmea[0];
  }

  return NULL;
}

// Get a pointer to the complete copy length
uint8_t *SFE_UBLOX_GNSS::getNMEACompleteLengthPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->completeCopy.length;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->completeCopy.length;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->completeCopy.length;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->completeCopy.length;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->completeCopy.length;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->completeCopy.length;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->completeCopy.length;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->completeCopy.length;
  }

  return NULL;
}

// Get a pointer to the complete copy NMEA data
uint8_t *SFE_UBLOX_GNSS::getNMEACompleteNMEAPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->completeCopy.nmea[0];
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->completeCopy.nmea[0];
  }

  return NULL;
}

// Get a pointer to the callback copy length
uint8_t *SFE_UBLOX_GNSS::getNMEACallbackLengthPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->callbackCopy->length;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->callbackCopy->length;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->callbackCopy->length;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->callbackCopy->length;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->callbackCopy->length;
  }

  return NULL;
}

// Get a pointer to the callback copy NMEA data
uint8_t *SFE_UBLOX_GNSS::getNMEACallbackNMEAPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->callbackCopy->nmea[0];
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->callbackCopy->nmea[0];
  }

  return NULL;
}

// Get the maximum length of this NMEA message
uint8_t SFE_UBLOX_GNSS::getNMEAMaxLength()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_GGA_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_GGA_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_VTG_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_VTG_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_RMC_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_RMC_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_ZDA_MAX_LENGTH;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return NMEA_ZDA_MAX_LENGTH;
  }

  return 0;
}

// Get a pointer to the automatic NMEA flags
nmeaAutomaticFlags *SFE_UBLOX_GNSS::getNMEAFlagsPtr()
{
  char thisNMEA[] = "GPGGA";
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPGGA->automaticFlags;
  }

  strcpy(thisNMEA, "GNGGA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNGGA->automaticFlags;
  }

  strcpy(thisNMEA, "GPVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPVTG->automaticFlags;
  }

  strcpy(thisNMEA, "GNVTG");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNVTG->automaticFlags;
  }

  strcpy(thisNMEA, "GPRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPRMC->automaticFlags;
  }

  strcpy(thisNMEA, "GNRMC");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNRMC->automaticFlags;
  }

  strcpy(thisNMEA, "GPZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGPZDA->automaticFlags;
  }

  strcpy(thisNMEA, "GNZDA");
  if (memcmp(thisNMEA, &nmeaAddressField[1], 5) == 0)
  {
    return &storageNMEAGNZDA->automaticFlags;
  }

  return NULL;
}
#endif

// We need to be able to identify an RTCM packet and then the length
// so that we know when the RTCM message is completely received and we then start
// listening for other sentences (like NMEA or UBX)
// RTCM packet structure is very odd. I never found RTCM STANDARD 10403.2 but
// http://d1.amobbs.com/bbs_upload782111/files_39/ourdev_635123CK0HJT.pdf is good
// https://dspace.cvut.cz/bitstream/handle/10467/65205/F3-BP-2016-Shkalikava-Anastasiya-Prenos%20polohove%20informace%20prostrednictvim%20datove%20site.pdf?sequence=-1
// Lead me to: https://forum.u-blox.com/index.php/4348/how-to-read-rtcm-messages-from-neo-m8p
// RTCM 3.2 bytes look like this:
// Byte 0: Always 0xD3
// Byte 1: 6-bits of zero
// Byte 2: 10-bits of length of this packet including the first two-ish header bytes, + 6.
// byte 3 + 4 bits: Msg type 12 bits
// Example: D3 00 7C 43 F0 ... / 0x7C = 124+6 = 130 bytes in this packet, 0x43F = Msg type 1087
SFE_UBLOX_GNSS::sfe_ublox_sentence_types_e SFE_UBLOX_GNSS::processRTCMframe(uint8_t incoming, uint16_t *rtcmFrameCounter)
{
  static uint16_t rtcmLen = 0;

  if (*rtcmFrameCounter == 1)
  {
    rtcmLen = (incoming & 0x03) << 8; // Get the last two bits of this byte. Bits 8&9 of 10-bit length
  }
  else if (*rtcmFrameCounter == 2)
  {
    rtcmLen |= incoming; // Bits 0-7 of packet length
    rtcmLen += 6;        // There are 6 additional bytes of what we presume is header, msgType, CRC, and stuff
  }
  /*else if (rtcmFrameCounter == 3)
  {
    rtcmMsgType = incoming << 4; //Message Type, MS 4 bits
  }
  else if (rtcmFrameCounter == 4)
  {
    rtcmMsgType |= (incoming >> 4); //Message Type, bits 0-7
  }*/

  *rtcmFrameCounter = *rtcmFrameCounter + 1;

  processRTCM(incoming); // Here is where we expose this byte to the user

  // Reset and start looking for next sentence type when done
  return (*rtcmFrameCounter == rtcmLen) ? SFE_UBLOX_SENTENCE_TYPE_NONE : SFE_UBLOX_SENTENCE_TYPE_RTCM;
}

// This function is called for each byte of an RTCM frame
// Ths user can overwrite this function and process the RTCM frame as they please
// Bytes can be piped to Serial or other interface. The consumer could be a radio or the internet (Ntrip broadcaster)
void SFE_UBLOX_GNSS::processRTCM(uint8_t incoming)
{
  // Radio.sendReliable((String)incoming); //An example of passing this byte to a radio

  //_debugSerial->write(incoming); //An example of passing this byte out the serial port

  // Debug printing
  //   _debugSerial->print(F(" "));
  //   if(incoming < 0x10) _debugSerial->print(F("0"));
  //   _debugSerial->print(incoming, HEX);
  //   if(rtcmFrameCounter % 16 == 0) _debugSerial->println();

  (void)incoming; // Do something with incoming just to get rid of the pesky compiler warning!
}

// Given a character, file it away into the uxb packet structure
// Set valid to VALID or NOT_VALID once sentence is completely received and passes or fails CRC
// The payload portion of the packet can be 100s of bytes but the max array size is packetCfgPayloadSize bytes.
// startingSpot can be set so we only record a subset of bytes within a larger packet.
void SFE_UBLOX_GNSS::processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID)
{
  // If incomingUBX is a user-defined custom packet, then the payload size could be different to packetCfgPayloadSize.
  // TO DO: update this to prevent an overrun when receiving an automatic message
  //        and the incomingUBX payload size is smaller than packetCfgPayloadSize.
  uint16_t maximum_payload_size;
  if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETCFG)
    maximum_payload_size = packetCfgPayloadSize;
  else if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
  {
    // Calculate maximum payload size once Class and ID have been received
    // (This check is probably redundant as activePacketBuffer can only be SFE_UBLOX_PACKET_PACKETAUTO
    //  when ubxFrameCounter >= 3)
    // if (incomingUBX->counter >= 2)
    //{
    maximum_payload_size = getMaxPayloadSize(incomingUBX->cls, incomingUBX->id);
    if (maximum_payload_size == 0)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial->print(F("processUBX: getMaxPayloadSize returned ZERO!! Class: 0x"));
        _debugSerial->print(incomingUBX->cls);
        _debugSerial->print(F(" ID: 0x"));
        _debugSerial->println(incomingUBX->id);
      }
#endif
    }
    //}
    // else
    //  maximum_payload_size = 2;
  }
  else
    maximum_payload_size = 2;

  bool overrun = false;

  // Add all incoming bytes to the rolling checksum
  // Stop at len+4 as this is the checksum bytes to that should not be added to the rolling checksum
  if (incomingUBX->counter < incomingUBX->len + 4)
    addToChecksum(incoming);

  if (incomingUBX->counter == 0)
  {
    incomingUBX->cls = incoming;
  }
  else if (incomingUBX->counter == 1)
  {
    incomingUBX->id = incoming;
  }
  else if (incomingUBX->counter == 2) // Len LSB
  {
    incomingUBX->len = incoming;
  }
  else if (incomingUBX->counter == 3) // Len MSB
  {
    incomingUBX->len |= incoming << 8;
  }
  else if (incomingUBX->counter == incomingUBX->len + 4) // ChecksumA
  {
    incomingUBX->checksumA = incoming;
  }
  else if (incomingUBX->counter == incomingUBX->len + 5) // ChecksumB
  {
    incomingUBX->checksumB = incoming;

    currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // We're done! Reset the sentence to being looking for a new start char

    // Validate this sentence
    if ((incomingUBX->checksumA == rollingChecksumA) && (incomingUBX->checksumB == rollingChecksumB))
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_VALID; // Flag the packet as valid
      _signsOfLife = true;                                  // The checksum is valid, so set the _signsOfLife flag

      // Let's check if the class and ID match the requestedClass and requestedID
      // Remember - this could be a data packet or an ACK packet
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_VALID; // If we have a match, set the classAndIDmatch flag to valid
      }

      // If this is a NACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->id == UBX_ACK_NACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_NOTACKNOWLEDGED; // If we have a match, set the classAndIDmatch flag to NOTACKNOWLEDGED
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("processUBX: NACK received: Requested Class: 0x"));
          _debugSerial->print(incomingUBX->payload[0], HEX);
          _debugSerial->print(F(" Requested ID: 0x"));
          _debugSerial->println(incomingUBX->payload[1], HEX);
        }
#endif
      }

      // This is not an ACK and we do not have a complete class and ID match
      // So let's check for an "automatic" message arriving
      else if (checkAutomatic(incomingUBX->cls, incomingUBX->id))
      {
        // This isn't the message we are looking for...
        // Let's say so and leave incomingUBX->classAndIDmatch _unchanged_
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("processUBX: incoming \"automatic\" message: Class: 0x"));
          _debugSerial->print(incomingUBX->cls, HEX);
          _debugSerial->print(F(" ID: 0x"));
          _debugSerial->println(incomingUBX->id, HEX);
        }
#endif
      }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->print(F("Incoming: Size: "));
        _debugSerial->print(incomingUBX->len);
        _debugSerial->print(F(" Received: "));
        printPacket(incomingUBX);

        if (incomingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetCfg now valid"));
        }
        if (packetAck.valid == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetAck now valid"));
        }
        if (incomingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetCfg classAndIDmatch"));
        }
        if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID)
        {
          _debugSerial->println(F("packetAck classAndIDmatch"));
        }
      }
#endif

      // We've got a valid packet, now do something with it but only if ignoreThisPayload is false
      if (ignoreThisPayload == false)
      {
        processUBXpacket(incomingUBX);
      }
    }
    else // Checksum failure
    {
      incomingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID;

      // Let's check if the class and ID match the requestedClass and requestedID.
      // This is potentially risky as we are saying that we saw the requested Class and ID
      // but that the packet checksum failed. Potentially it could be the class or ID bytes
      // that caused the checksum error!
      if ((incomingUBX->cls == requestedClass) && (incomingUBX->id == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }
      // If this is an ACK then let's check if the class and ID match the requestedClass and requestedID
      else if ((incomingUBX->cls == UBX_CLASS_ACK) && (incomingUBX->payload[0] == requestedClass) && (incomingUBX->payload[1] == requestedID))
      {
        incomingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_VALID; // If we have a match, set the classAndIDmatch flag to not valid
      }

      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        // Drive an external pin to allow for easier logic analyzation
        if (debugPin >= 0)
        {
          digitalWrite((uint8_t)debugPin, LOW);
          delay(10);
          digitalWrite((uint8_t)debugPin, HIGH);
        }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        _debugSerial->print(F("Checksum failed:"));
        _debugSerial->print(F(" checksumA: "));
        _debugSerial->print(incomingUBX->checksumA);
        _debugSerial->print(F(" checksumB: "));
        _debugSerial->print(incomingUBX->checksumB);

        _debugSerial->print(F(" rollingChecksumA: "));
        _debugSerial->print(rollingChecksumA);
        _debugSerial->print(F(" rollingChecksumB: "));
        _debugSerial->print(rollingChecksumB);
        _debugSerial->println();
#endif
      }
    }

    // Now that the packet is complete and has been processed, we need to delete the memory
    // allocated for packetAuto
    if (activePacketBuffer == SFE_UBLOX_PACKET_PACKETAUTO)
    {
      delete[] payloadAuto; // Created with new[]
      payloadAuto = NULL;   // Redundant?
      packetAuto.payload = payloadAuto;
    }
  }
  else // Load this byte into the payload array
  {
    // If an automatic packet comes in asynchronously, we need to fudge the startingSpot
    uint16_t startingSpot = incomingUBX->startingSpot;
    if (checkAutomatic(incomingUBX->cls, incomingUBX->id))
      startingSpot = 0;
    // Check if this is payload data which should be ignored
    if (ignoreThisPayload == false)
    {
      // Begin recording if counter goes past startingSpot
      if ((incomingUBX->counter - 4) >= startingSpot)
      {
        // Check to see if we have room for this byte
        if (((incomingUBX->counter - 4) - startingSpot) < maximum_payload_size) // If counter = 208, starting spot = 200, we're good to record.
        {
          incomingUBX->payload[(incomingUBX->counter - 4) - startingSpot] = incoming; // Store this byte into payload array
        }
        else
        {
          overrun = true;
        }
      }
    }
  }

  // incomingUBX->counter should never reach maximum_payload_size + class + id + len[2] + checksum[2]
  if (overrun || ((incomingUBX->counter == maximum_payload_size + 6) && (ignoreThisPayload == false)))
  {
    // Something has gone very wrong
    currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE; // Reset the sentence to being looking for a new start char
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      if (overrun)
        _debugSerial->print(F("processUBX: buffer overrun detected!"));
      else
        _debugSerial->print(F("processUBX: counter hit maximum_payload_size + 6!"));
      _debugSerial->print(F(" activePacketBuffer: "));
      _debugSerial->print(activePacketBuffer);
      _debugSerial->print(F(" maximum_payload_size: "));
      _debugSerial->println(maximum_payload_size);
    }
#endif
  }

  // Increment the counter
  incomingUBX->counter++;
}

// Once a packet has been received and validated, identify this packet's class/id and update internal flags
void SFE_UBLOX_GNSS::processUBXpacket(ubxPacket *msg)
{
  switch (msg->cls)
  {
  case UBX_CLASS_NAV:
    if (msg->id == UBX_NAV_POSECEF && msg->len == UBX_NAV_POSECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPOSECEF != NULL)
      {
        packetUBXNAVPOSECEF->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPOSECEF->data.ecefX = extractSignedLong(msg, 4);
        packetUBXNAVPOSECEF->data.ecefY = extractSignedLong(msg, 8);
        packetUBXNAVPOSECEF->data.ecefZ = extractSignedLong(msg, 12);
        packetUBXNAVPOSECEF->data.pAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPOSECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPOSECEF->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPOSECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPOSECEF->callbackData->iTOW, &packetUBXNAVPOSECEF->data.iTOW, sizeof(UBX_NAV_POSECEF_data_t));
          packetUBXNAVPOSECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPOSECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_STATUS && msg->len == UBX_NAV_STATUS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSTATUS != NULL)
      {
        packetUBXNAVSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVSTATUS->data.gpsFix = extractByte(msg, 4);
        packetUBXNAVSTATUS->data.flags.all = extractByte(msg, 5);
        packetUBXNAVSTATUS->data.fixStat.all = extractByte(msg, 6);
        packetUBXNAVSTATUS->data.flags2.all = extractByte(msg, 7);
        packetUBXNAVSTATUS->data.ttff = extractLong(msg, 8);
        packetUBXNAVSTATUS->data.msss = extractLong(msg, 12);

        // Mark all datums as fresh (not read before)
        packetUBXNAVSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSTATUS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSTATUS->callbackData->iTOW, &packetUBXNAVSTATUS->data.iTOW, sizeof(UBX_NAV_STATUS_data_t));
          packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_DOP && msg->len == UBX_NAV_DOP_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVDOP != NULL)
      {
        packetUBXNAVDOP->data.iTOW = extractLong(msg, 0);
        packetUBXNAVDOP->data.gDOP = extractInt(msg, 4);
        packetUBXNAVDOP->data.pDOP = extractInt(msg, 6);
        packetUBXNAVDOP->data.tDOP = extractInt(msg, 8);
        packetUBXNAVDOP->data.vDOP = extractInt(msg, 10);
        packetUBXNAVDOP->data.hDOP = extractInt(msg, 12);
        packetUBXNAVDOP->data.nDOP = extractInt(msg, 14);
        packetUBXNAVDOP->data.eDOP = extractInt(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVDOP->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVDOP->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVDOP->callbackData->iTOW, &packetUBXNAVDOP->data.iTOW, sizeof(UBX_NAV_DOP_data_t));
          packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVDOP->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_ATT && msg->len == UBX_NAV_ATT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVATT != NULL)
      {
        packetUBXNAVATT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVATT->data.version = extractByte(msg, 4);
        packetUBXNAVATT->data.roll = extractSignedLong(msg, 8);
        packetUBXNAVATT->data.pitch = extractSignedLong(msg, 12);
        packetUBXNAVATT->data.heading = extractSignedLong(msg, 16);
        packetUBXNAVATT->data.accRoll = extractLong(msg, 20);
        packetUBXNAVATT->data.accPitch = extractLong(msg, 24);
        packetUBXNAVATT->data.accHeading = extractLong(msg, 28);

        // Mark all datums as fresh (not read before)
        packetUBXNAVATT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVATT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVATT->callbackData->iTOW, &packetUBXNAVATT->data.iTOW, sizeof(UBX_NAV_ATT_data_t));
          packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVATT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_PVT && msg->len == UBX_NAV_PVT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPVT != NULL)
      {
        packetUBXNAVPVT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPVT->data.year = extractInt(msg, 4);
        packetUBXNAVPVT->data.month = extractByte(msg, 6);
        packetUBXNAVPVT->data.day = extractByte(msg, 7);
        packetUBXNAVPVT->data.hour = extractByte(msg, 8);
        packetUBXNAVPVT->data.min = extractByte(msg, 9);
        packetUBXNAVPVT->data.sec = extractByte(msg, 10);
        packetUBXNAVPVT->data.valid.all = extractByte(msg, 11);
        packetUBXNAVPVT->data.tAcc = extractLong(msg, 12);
        packetUBXNAVPVT->data.nano = extractSignedLong(msg, 16); // Includes milliseconds
        packetUBXNAVPVT->data.fixType = extractByte(msg, 20);
        packetUBXNAVPVT->data.flags.all = extractByte(msg, 21);
        packetUBXNAVPVT->data.flags2.all = extractByte(msg, 22);
        packetUBXNAVPVT->data.numSV = extractByte(msg, 23);
        packetUBXNAVPVT->data.lon = extractSignedLong(msg, 24);
        packetUBXNAVPVT->data.lat = extractSignedLong(msg, 28);
        packetUBXNAVPVT->data.height = extractSignedLong(msg, 32);
        packetUBXNAVPVT->data.hMSL = extractSignedLong(msg, 36);
        packetUBXNAVPVT->data.hAcc = extractLong(msg, 40);
        packetUBXNAVPVT->data.vAcc = extractLong(msg, 44);
        packetUBXNAVPVT->data.velN = extractSignedLong(msg, 48);
        packetUBXNAVPVT->data.velE = extractSignedLong(msg, 52);
        packetUBXNAVPVT->data.velD = extractSignedLong(msg, 56);
        packetUBXNAVPVT->data.gSpeed = extractSignedLong(msg, 60);
        packetUBXNAVPVT->data.headMot = extractSignedLong(msg, 64);
        packetUBXNAVPVT->data.sAcc = extractLong(msg, 68);
        packetUBXNAVPVT->data.headAcc = extractLong(msg, 72);
        packetUBXNAVPVT->data.pDOP = extractInt(msg, 76);
        packetUBXNAVPVT->data.flags3.all = extractByte(msg, 78);
        packetUBXNAVPVT->data.headVeh = extractSignedLong(msg, 84);
        packetUBXNAVPVT->data.magDec = extractSignedInt(msg, 88);
        packetUBXNAVPVT->data.magAcc = extractInt(msg, 90);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
        packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPVT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPVT->callbackData->iTOW, &packetUBXNAVPVT->data.iTOW, sizeof(UBX_NAV_PVT_data_t));
          packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPVT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_ODO && msg->len == UBX_NAV_ODO_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVODO != NULL)
      {
        packetUBXNAVODO->data.version = extractByte(msg, 0);
        packetUBXNAVODO->data.iTOW = extractLong(msg, 4);
        packetUBXNAVODO->data.distance = extractLong(msg, 8);
        packetUBXNAVODO->data.totalDistance = extractLong(msg, 12);
        packetUBXNAVODO->data.distanceStd = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVODO->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVODO->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVODO->callbackData->version, &packetUBXNAVODO->data.version, sizeof(UBX_NAV_ODO_data_t));
          packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVODO->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_VELECEF && msg->len == UBX_NAV_VELECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVVELECEF != NULL)
      {
        packetUBXNAVVELECEF->data.iTOW = extractLong(msg, 0);
        packetUBXNAVVELECEF->data.ecefVX = extractSignedLong(msg, 4);
        packetUBXNAVVELECEF->data.ecefVY = extractSignedLong(msg, 8);
        packetUBXNAVVELECEF->data.ecefVZ = extractSignedLong(msg, 12);
        packetUBXNAVVELECEF->data.sAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVVELECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVVELECEF->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVVELECEF->callbackData->iTOW, &packetUBXNAVVELECEF->data.iTOW, sizeof(UBX_NAV_VELECEF_data_t));
          packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVVELECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_VELNED && msg->len == UBX_NAV_VELNED_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVVELNED != NULL)
      {
        packetUBXNAVVELNED->data.iTOW = extractLong(msg, 0);
        packetUBXNAVVELNED->data.velN = extractSignedLong(msg, 4);
        packetUBXNAVVELNED->data.velE = extractSignedLong(msg, 8);
        packetUBXNAVVELNED->data.velD = extractSignedLong(msg, 12);
        packetUBXNAVVELNED->data.speed = extractLong(msg, 16);
        packetUBXNAVVELNED->data.gSpeed = extractLong(msg, 20);
        packetUBXNAVVELNED->data.heading = extractSignedLong(msg, 24);
        packetUBXNAVVELNED->data.sAcc = extractLong(msg, 28);
        packetUBXNAVVELNED->data.cAcc = extractLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXNAVVELNED->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVVELNED->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVVELNED->callbackData->iTOW, &packetUBXNAVVELNED->data.iTOW, sizeof(UBX_NAV_VELNED_data_t));
          packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVVELNED->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_HPPOSECEF && msg->len == UBX_NAV_HPPOSECEF_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVHPPOSECEF != NULL)
      {
        packetUBXNAVHPPOSECEF->data.version = extractByte(msg, 0);
        packetUBXNAVHPPOSECEF->data.iTOW = extractLong(msg, 4);
        packetUBXNAVHPPOSECEF->data.ecefX = extractSignedLong(msg, 8);
        packetUBXNAVHPPOSECEF->data.ecefY = extractSignedLong(msg, 12);
        packetUBXNAVHPPOSECEF->data.ecefZ = extractSignedLong(msg, 16);
        packetUBXNAVHPPOSECEF->data.ecefXHp = extractSignedChar(msg, 20);
        packetUBXNAVHPPOSECEF->data.ecefYHp = extractSignedChar(msg, 21);
        packetUBXNAVHPPOSECEF->data.ecefZHp = extractSignedChar(msg, 22);
        packetUBXNAVHPPOSECEF->data.flags.all = extractByte(msg, 23);
        packetUBXNAVHPPOSECEF->data.pAcc = extractLong(msg, 24);

        // Mark all datums as fresh (not read before)
        packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVHPPOSECEF->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVHPPOSECEF->callbackData->version, &packetUBXNAVHPPOSECEF->data.version, sizeof(UBX_NAV_HPPOSECEF_data_t));
          packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_HPPOSLLH && msg->len == UBX_NAV_HPPOSLLH_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVHPPOSLLH != NULL)
      {
        packetUBXNAVHPPOSLLH->data.version = extractByte(msg, 0);
        packetUBXNAVHPPOSLLH->data.flags.all = extractByte(msg, 3);
        packetUBXNAVHPPOSLLH->data.iTOW = extractLong(msg, 4);
        packetUBXNAVHPPOSLLH->data.lon = extractSignedLong(msg, 8);
        packetUBXNAVHPPOSLLH->data.lat = extractSignedLong(msg, 12);
        packetUBXNAVHPPOSLLH->data.height = extractSignedLong(msg, 16);
        packetUBXNAVHPPOSLLH->data.hMSL = extractSignedLong(msg, 20);
        packetUBXNAVHPPOSLLH->data.lonHp = extractSignedChar(msg, 24);
        packetUBXNAVHPPOSLLH->data.latHp = extractSignedChar(msg, 25);
        packetUBXNAVHPPOSLLH->data.heightHp = extractSignedChar(msg, 26);
        packetUBXNAVHPPOSLLH->data.hMSLHp = extractSignedChar(msg, 27);
        packetUBXNAVHPPOSLLH->data.hAcc = extractLong(msg, 28);
        packetUBXNAVHPPOSLLH->data.vAcc = extractLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVHPPOSLLH->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVHPPOSLLH->callbackData->version, &packetUBXNAVHPPOSLLH->data.version, sizeof(UBX_NAV_HPPOSLLH_data_t));
          packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_PVAT && msg->len == UBX_NAV_PVAT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVPVAT != NULL)
      {
        packetUBXNAVPVAT->data.iTOW = extractLong(msg, 0);
        packetUBXNAVPVAT->data.version = extractByte(msg, 4);
        packetUBXNAVPVAT->data.valid.all = extractByte(msg, 5);
        packetUBXNAVPVAT->data.year = extractInt(msg, 6);
        packetUBXNAVPVAT->data.month = extractByte(msg, 8);
        packetUBXNAVPVAT->data.day = extractByte(msg, 9);
        packetUBXNAVPVAT->data.hour = extractByte(msg, 10);
        packetUBXNAVPVAT->data.min = extractByte(msg, 11);
        packetUBXNAVPVAT->data.sec = extractByte(msg, 12);
        packetUBXNAVPVAT->data.tAcc = extractLong(msg, 16);
        packetUBXNAVPVAT->data.nano = extractSignedLong(msg, 20); // Includes milliseconds
        packetUBXNAVPVAT->data.fixType = extractByte(msg, 24);
        packetUBXNAVPVAT->data.flags.all = extractByte(msg, 25);
        packetUBXNAVPVAT->data.flags2.all = extractByte(msg, 26);
        packetUBXNAVPVAT->data.numSV = extractByte(msg, 27);
        packetUBXNAVPVAT->data.lon = extractSignedLong(msg, 28);
        packetUBXNAVPVAT->data.lat = extractSignedLong(msg, 32);
        packetUBXNAVPVAT->data.height = extractSignedLong(msg, 36);
        packetUBXNAVPVAT->data.hMSL = extractSignedLong(msg, 40);
        packetUBXNAVPVAT->data.hAcc = extractLong(msg, 44);
        packetUBXNAVPVAT->data.vAcc = extractLong(msg, 48);
        packetUBXNAVPVAT->data.velN = extractSignedLong(msg, 52);
        packetUBXNAVPVAT->data.velE = extractSignedLong(msg, 56);
        packetUBXNAVPVAT->data.velD = extractSignedLong(msg, 60);
        packetUBXNAVPVAT->data.gSpeed = extractSignedLong(msg, 64);
        packetUBXNAVPVAT->data.sAcc = extractLong(msg, 68);
        packetUBXNAVPVAT->data.vehRoll = extractSignedLong(msg, 72);
        packetUBXNAVPVAT->data.vehPitch = extractSignedLong(msg, 76);
        packetUBXNAVPVAT->data.vehHeading = extractSignedLong(msg, 80);
        packetUBXNAVPVAT->data.motHeading = extractSignedLong(msg, 84);
        packetUBXNAVPVAT->data.accRoll = extractInt(msg, 88);
        packetUBXNAVPVAT->data.accPitch = extractInt(msg, 90);
        packetUBXNAVPVAT->data.accHeading = extractInt(msg, 92);
        packetUBXNAVPVAT->data.magDec = extractSignedInt(msg, 94);
        packetUBXNAVPVAT->data.magAcc = extractInt(msg, 96);
        packetUBXNAVPVAT->data.errEllipseOrient = extractInt(msg, 98);
        packetUBXNAVPVAT->data.errEllipseMajor = extractLong(msg, 100);
        packetUBXNAVPVAT->data.errEllipseMinor = extractLong(msg, 104);

        // Mark all datums as fresh (not read before)
        packetUBXNAVPVAT->moduleQueried.moduleQueried1.all = 0xFFFFFFFF;
        packetUBXNAVPVAT->moduleQueried.moduleQueried2.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVPVAT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVPVAT->callbackData->iTOW, &packetUBXNAVPVAT->data.iTOW, sizeof(UBX_NAV_PVAT_data_t));
          packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVPVAT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_TIMEUTC && msg->len == UBX_NAV_TIMEUTC_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVTIMEUTC != NULL)
      {
        packetUBXNAVTIMEUTC->data.iTOW = extractLong(msg, 0);
        packetUBXNAVTIMEUTC->data.tAcc = extractLong(msg, 4);
        packetUBXNAVTIMEUTC->data.nano = extractSignedLong(msg, 8);
        packetUBXNAVTIMEUTC->data.year = extractInt(msg, 12);
        packetUBXNAVTIMEUTC->data.month = extractByte(msg, 14);
        packetUBXNAVTIMEUTC->data.day = extractByte(msg, 15);
        packetUBXNAVTIMEUTC->data.hour = extractByte(msg, 16);
        packetUBXNAVTIMEUTC->data.min = extractByte(msg, 17);
        packetUBXNAVTIMEUTC->data.sec = extractByte(msg, 18);
        packetUBXNAVTIMEUTC->data.valid.all = extractByte(msg, 19);

        // Mark all datums as fresh (not read before)
        packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVTIMEUTC->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVTIMEUTC->callbackData->iTOW, &packetUBXNAVTIMEUTC->data.iTOW, sizeof(UBX_NAV_TIMEUTC_data_t));
          packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_CLOCK && msg->len == UBX_NAV_CLOCK_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVCLOCK != NULL)
      {
        packetUBXNAVCLOCK->data.iTOW = extractLong(msg, 0);
        packetUBXNAVCLOCK->data.clkB = extractSignedLong(msg, 4);
        packetUBXNAVCLOCK->data.clkD = extractSignedLong(msg, 8);
        packetUBXNAVCLOCK->data.tAcc = extractLong(msg, 12);
        packetUBXNAVCLOCK->data.fAcc = extractLong(msg, 16);

        // Mark all datums as fresh (not read before)
        packetUBXNAVCLOCK->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVCLOCK->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVCLOCK->callbackData->iTOW, &packetUBXNAVCLOCK->data.iTOW, sizeof(UBX_NAV_CLOCK_data_t));
          packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVCLOCK->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_TIMELS && msg->len == UBX_NAV_TIMELS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVTIMELS != NULL)
      {
        packetUBXNAVTIMELS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVTIMELS->data.version = extractByte(msg, 4);
        packetUBXNAVTIMELS->data.srcOfCurrLs = extractByte(msg, 8);
        packetUBXNAVTIMELS->data.currLs = extractSignedChar(msg, 9);
        packetUBXNAVTIMELS->data.srcOfLsChange = extractByte(msg, 10);
        packetUBXNAVTIMELS->data.lsChange = extractSignedChar(msg, 11);
        packetUBXNAVTIMELS->data.timeToLsEvent = extractSignedLong(msg, 12);
        packetUBXNAVTIMELS->data.dateOfLsGpsWn = extractInt(msg, 16);
        packetUBXNAVTIMELS->data.dateOfLsGpsDn = extractInt(msg, 18);
        packetUBXNAVTIMELS->data.valid.all = extractSignedChar(msg, 23);

        // Mark all datums as fresh (not read before)
        packetUBXNAVTIMELS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;
      }
    }
    else if (msg->id == UBX_NAV_SVIN && msg->len == UBX_NAV_SVIN_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSVIN != NULL)
      {
        packetUBXNAVSVIN->data.version = extractByte(msg, 0);
        packetUBXNAVSVIN->data.iTOW = extractLong(msg, 4);
        packetUBXNAVSVIN->data.dur = extractLong(msg, 8);
        packetUBXNAVSVIN->data.meanX = extractSignedLong(msg, 12);
        packetUBXNAVSVIN->data.meanY = extractSignedLong(msg, 16);
        packetUBXNAVSVIN->data.meanZ = extractSignedLong(msg, 20);
        packetUBXNAVSVIN->data.meanXHP = extractSignedChar(msg, 24);
        packetUBXNAVSVIN->data.meanYHP = extractSignedChar(msg, 25);
        packetUBXNAVSVIN->data.meanZHP = extractSignedChar(msg, 26);
        packetUBXNAVSVIN->data.meanAcc = extractLong(msg, 28);
        packetUBXNAVSVIN->data.obs = extractLong(msg, 32);
        packetUBXNAVSVIN->data.valid = extractSignedChar(msg, 36);
        packetUBXNAVSVIN->data.active = extractSignedChar(msg, 37);

        // Mark all datums as fresh (not read before)
        packetUBXNAVSVIN->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSVIN->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSVIN->callbackData->version, &packetUBXNAVSVIN->data.version, sizeof(UBX_NAV_SVIN_data_t));
          packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSVIN->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_SAT) // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVSAT != NULL)
      {
        packetUBXNAVSAT->data.header.iTOW = extractLong(msg, 0);
        packetUBXNAVSAT->data.header.version = extractByte(msg, 4);
        packetUBXNAVSAT->data.header.numSvs = extractByte(msg, 5);

        // The NAV SAT message could contain data for 255 SVs max. (numSvs is uint8_t. UBX_NAV_SAT_MAX_BLOCKS is 255)
        for (uint16_t i = 0; (i < UBX_NAV_SAT_MAX_BLOCKS) && (i < ((uint16_t)packetUBXNAVSAT->data.header.numSvs)) && ((i * 12) < (msg->len - 8)); i++)
        {
          uint16_t offset = (i * 12) + 8;
          packetUBXNAVSAT->data.blocks[i].gnssId = extractByte(msg, offset + 0);
          packetUBXNAVSAT->data.blocks[i].svId = extractByte(msg, offset + 1);
          packetUBXNAVSAT->data.blocks[i].cno = extractByte(msg, offset + 2);
          packetUBXNAVSAT->data.blocks[i].elev = extractSignedChar(msg, offset + 3);
          packetUBXNAVSAT->data.blocks[i].azim = extractSignedInt(msg, offset + 4);
          packetUBXNAVSAT->data.blocks[i].prRes = extractSignedInt(msg, offset + 6);
          packetUBXNAVSAT->data.blocks[i].flags.all = extractLong(msg, offset + 8);
        }

        // Mark all datums as fresh (not read before)
        packetUBXNAVSAT->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVSAT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVSAT->callbackData->header.iTOW, &packetUBXNAVSAT->data.header.iTOW, sizeof(UBX_NAV_SAT_data_t));
          packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVSAT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_RELPOSNED && ((msg->len == UBX_NAV_RELPOSNED_LEN) || (msg->len == UBX_NAV_RELPOSNED_LEN_F9)))
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVRELPOSNED != NULL)
      {
        // Note:
        //   RELPOSNED on the M8 is only 40 bytes long
        //   RELPOSNED on the F9 is 64 bytes long and contains much more information

        packetUBXNAVRELPOSNED->data.version = extractByte(msg, 0);
        packetUBXNAVRELPOSNED->data.refStationId = extractInt(msg, 2);
        packetUBXNAVRELPOSNED->data.iTOW = extractLong(msg, 4);
        packetUBXNAVRELPOSNED->data.relPosN = extractSignedLong(msg, 8);
        packetUBXNAVRELPOSNED->data.relPosE = extractSignedLong(msg, 12);
        packetUBXNAVRELPOSNED->data.relPosD = extractSignedLong(msg, 16);

        if (msg->len == UBX_NAV_RELPOSNED_LEN)
        {
          // The M8 version does not contain relPosLength or relPosHeading
          packetUBXNAVRELPOSNED->data.relPosLength = 0;
          packetUBXNAVRELPOSNED->data.relPosHeading = 0;
          packetUBXNAVRELPOSNED->data.relPosHPN = extractSignedChar(msg, 20);
          packetUBXNAVRELPOSNED->data.relPosHPE = extractSignedChar(msg, 21);
          packetUBXNAVRELPOSNED->data.relPosHPD = extractSignedChar(msg, 22);
          packetUBXNAVRELPOSNED->data.relPosHPLength = 0; // The M8 version does not contain relPosHPLength
          packetUBXNAVRELPOSNED->data.accN = extractLong(msg, 24);
          packetUBXNAVRELPOSNED->data.accE = extractLong(msg, 28);
          packetUBXNAVRELPOSNED->data.accD = extractLong(msg, 32);
          // The M8 version does not contain accLength or accHeading
          packetUBXNAVRELPOSNED->data.accLength = 0;
          packetUBXNAVRELPOSNED->data.accHeading = 0;
          packetUBXNAVRELPOSNED->data.flags.all = extractLong(msg, 36);
        }
        else
        {
          packetUBXNAVRELPOSNED->data.relPosLength = extractSignedLong(msg, 20);
          packetUBXNAVRELPOSNED->data.relPosHeading = extractSignedLong(msg, 24);
          packetUBXNAVRELPOSNED->data.relPosHPN = extractSignedChar(msg, 32);
          packetUBXNAVRELPOSNED->data.relPosHPE = extractSignedChar(msg, 33);
          packetUBXNAVRELPOSNED->data.relPosHPD = extractSignedChar(msg, 34);
          packetUBXNAVRELPOSNED->data.relPosHPLength = extractSignedChar(msg, 35);
          packetUBXNAVRELPOSNED->data.accN = extractLong(msg, 36);
          packetUBXNAVRELPOSNED->data.accE = extractLong(msg, 40);
          packetUBXNAVRELPOSNED->data.accD = extractLong(msg, 44);
          packetUBXNAVRELPOSNED->data.accLength = extractLong(msg, 48);
          packetUBXNAVRELPOSNED->data.accHeading = extractLong(msg, 52);
          packetUBXNAVRELPOSNED->data.flags.all = extractLong(msg, 60);
        }

        // Mark all datums as fresh (not read before)
        packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVRELPOSNED->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVRELPOSNED->callbackData->version, &packetUBXNAVRELPOSNED->data.version, sizeof(UBX_NAV_RELPOSNED_data_t));
          packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_AOPSTATUS && msg->len == UBX_NAV_AOPSTATUS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVAOPSTATUS != NULL)
      {
        packetUBXNAVAOPSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXNAVAOPSTATUS->data.aopCfg.all = extractByte(msg, 4);
        packetUBXNAVAOPSTATUS->data.status = extractByte(msg, 5);

        // Mark all datums as fresh (not read before)
        packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVAOPSTATUS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVAOPSTATUS->callbackData->iTOW, &packetUBXNAVAOPSTATUS->data.iTOW, sizeof(UBX_NAV_AOPSTATUS_data_t));
          packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_NAV_EOE && msg->len == UBX_NAV_EOE_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXNAVEOE != NULL)
      {
        packetUBXNAVEOE->data.iTOW = extractLong(msg, 0);

        // Mark all datums as fresh (not read before)
        packetUBXNAVEOE->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXNAVEOE->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXNAVEOE->callbackData->iTOW, &packetUBXNAVEOE->data.iTOW, sizeof(UBX_NAV_EOE_data_t));
          packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXNAVEOE->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    break;
  case UBX_CLASS_RXM:
    if (msg->id == UBX_RXM_PMP)
    // Note: length is variable with version 0x01
    // Note: the field positions depend on the version
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it.
      // By default, new PMP data will always overwrite 'old' data (data which is valid but which has not yet been read by the callback).
      // To prevent this, uncomment the line two lines below
      if ((packetUBXRXMPMP != NULL) && (packetUBXRXMPMP->callbackData != NULL)
          //&& (packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMPMP->callbackData->version = extractByte(msg, 0);
        packetUBXRXMPMP->callbackData->numBytesUserData = extractInt(msg, 2);
        packetUBXRXMPMP->callbackData->timeTag = extractLong(msg, 4);
        packetUBXRXMPMP->callbackData->uniqueWord[0] = extractLong(msg, 8);
        packetUBXRXMPMP->callbackData->uniqueWord[1] = extractLong(msg, 12);
        packetUBXRXMPMP->callbackData->serviceIdentifier = extractInt(msg, 16);
        packetUBXRXMPMP->callbackData->spare = extractByte(msg, 18);
        packetUBXRXMPMP->callbackData->uniqueWordBitErrors = extractByte(msg, 19);

        if (packetUBXRXMPMP->callbackData->version == 0x00)
        {
          packetUBXRXMPMP->callbackData->fecBits = extractInt(msg, 524);
          packetUBXRXMPMP->callbackData->ebno = extractByte(msg, 526);
        }
        else // if (packetUBXRXMPMP->data.version == 0x01)
        {
          packetUBXRXMPMP->callbackData->fecBits = extractInt(msg, 20);
          packetUBXRXMPMP->callbackData->ebno = extractByte(msg, 22);
        }

        uint16_t userDataStart = (packetUBXRXMPMP->callbackData->version == 0x00) ? 20 : 24;
        uint16_t userDataLength = (packetUBXRXMPMP->callbackData->version == 0x00) ? 504 : (packetUBXRXMPMP->callbackData->numBytesUserData);
        for (uint16_t i = 0; (i < userDataLength) && (i < 504); i++)
        {
          packetUBXRXMPMP->callbackData->userData[i] = extractByte(msg, i + userDataStart);
        }

        packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }

      // Full PMP message, including Class, ID and checksum
      // By default, new PMP data will always overwrite 'old' data (data which is valid but which has not yet been read by the callback).
      // To prevent this, uncomment the line two lines below
      if ((packetUBXRXMPMPmessage != NULL) && (packetUBXRXMPMPmessage->callbackData != NULL)
          //&& (packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMPMPmessage->callbackData->sync1 = UBX_SYNCH_1;
        packetUBXRXMPMPmessage->callbackData->sync2 = UBX_SYNCH_2;
        packetUBXRXMPMPmessage->callbackData->cls = UBX_CLASS_RXM;
        packetUBXRXMPMPmessage->callbackData->ID = UBX_RXM_PMP;
        packetUBXRXMPMPmessage->callbackData->lengthLSB = msg->len & 0xFF;
        packetUBXRXMPMPmessage->callbackData->lengthMSB = msg->len >> 8;

        memcpy(packetUBXRXMPMPmessage->callbackData->payload, msg->payload, msg->len);

        packetUBXRXMPMPmessage->callbackData->checksumA = msg->checksumA;
        packetUBXRXMPMPmessage->callbackData->checksumB = msg->checksumB;

        packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }
    }
    else if (msg->id == UBX_RXM_QZSSL6)
    // Note: length is variable with version 0x01
    // Note: the field positions depend on the version
    {
      // Full QZSSL6 message, including Class, ID and checksum
      for (int ch = 0; ch < UBX_RXM_QZSSL6_NUM_CHANNELS; ch++)
      {
        if (0 == (packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid & (1 << ch)))
        {

          packetUBXRXMQZSSL6message->callbackData[ch].sync1 = UBX_SYNCH_1;
          packetUBXRXMQZSSL6message->callbackData[ch].sync2 = UBX_SYNCH_2;
          packetUBXRXMQZSSL6message->callbackData[ch].cls = UBX_CLASS_RXM;
          packetUBXRXMQZSSL6message->callbackData[ch].ID = UBX_RXM_QZSSL6;
          packetUBXRXMQZSSL6message->callbackData[ch].lengthLSB = msg->len & 0xFF;
          packetUBXRXMQZSSL6message->callbackData[ch].lengthMSB = msg->len >> 8;

          memcpy(packetUBXRXMQZSSL6message->callbackData[ch].payload, msg->payload, msg->len);

          packetUBXRXMQZSSL6message->callbackData[ch].checksumA = msg->checksumA;
          packetUBXRXMQZSSL6message->callbackData[ch].checksumB = msg->checksumB;

          packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid |= (1 << ch);
          break; // abort when added
        }
      }
    }
    else if (msg->id == UBX_RXM_COR)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if ((packetUBXRXMCOR != NULL) && (packetUBXRXMCOR->callbackData != NULL)
          //&& (packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid == false) // <=== Uncomment this line to prevent new data from overwriting 'old'
      )
      {
        packetUBXRXMCOR->callbackData->version = extractByte(msg, 0);
        packetUBXRXMCOR->callbackData->ebno = extractByte(msg, 1);
        packetUBXRXMCOR->callbackData->statusInfo.all = extractLong(msg, 4);
        packetUBXRXMCOR->callbackData->msgType = extractInt(msg, 8);
        packetUBXRXMCOR->callbackData->msgSubType = extractInt(msg, 10);

        packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid = true; // Mark the data as valid
      }
    }
    else if (msg->id == UBX_RXM_SFRBX)
    // Note: length is variable
    // Note: on protocol version 17: numWords is (0..16)
    //       on protocol version 18+: numWords is (0..10)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXRXMSFRBX != NULL)
      {
        packetUBXRXMSFRBX->data.gnssId = extractByte(msg, 0);
        packetUBXRXMSFRBX->data.svId = extractByte(msg, 1);
        packetUBXRXMSFRBX->data.freqId = extractByte(msg, 3);
        packetUBXRXMSFRBX->data.numWords = extractByte(msg, 4);
        packetUBXRXMSFRBX->data.chn = extractByte(msg, 5);
        packetUBXRXMSFRBX->data.version = extractByte(msg, 6);

        for (uint8_t i = 0; (i < UBX_RXM_SFRBX_MAX_WORDS) && (i < packetUBXRXMSFRBX->data.numWords) && ((i * 4) < (msg->len - 8)); i++)
        {
          packetUBXRXMSFRBX->data.dwrd[i] = extractLong(msg, 8 + (i * 4));
        }

        // Mark all datums as fresh (not read before)
        packetUBXRXMSFRBX->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXRXMSFRBX->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXRXMSFRBX->callbackData->gnssId, &packetUBXRXMSFRBX->data.gnssId, sizeof(UBX_RXM_SFRBX_data_t));
          packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXRXMSFRBX->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_RXM_RAWX)
    // Note: length is variable
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXRXMRAWX != NULL)
      {
        for (uint8_t i = 0; i < 8; i++)
        {
          packetUBXRXMRAWX->data.header.rcvTow[i] = extractByte(msg, i);
        }
        packetUBXRXMRAWX->data.header.week = extractInt(msg, 8);
        packetUBXRXMRAWX->data.header.leapS = extractSignedChar(msg, 10);
        packetUBXRXMRAWX->data.header.numMeas = extractByte(msg, 11);
        packetUBXRXMRAWX->data.header.recStat.all = extractByte(msg, 12);
        packetUBXRXMRAWX->data.header.version = extractByte(msg, 13);

        for (uint8_t i = 0; (i < UBX_RXM_RAWX_MAX_BLOCKS) && (i < packetUBXRXMRAWX->data.header.numMeas) && ((((uint16_t)i) * 32) < (msg->len - 16)); i++)
        {
          uint16_t offset = (((uint16_t)i) * 32) + 16;
          for (uint8_t j = 0; j < 8; j++)
          {
            packetUBXRXMRAWX->data.blocks[i].prMes[j] = extractByte(msg, offset + j);
            packetUBXRXMRAWX->data.blocks[i].cpMes[j] = extractByte(msg, offset + 8 + j);
            if (j < 4)
              packetUBXRXMRAWX->data.blocks[i].doMes[j] = extractByte(msg, offset + 16 + j);
          }
          packetUBXRXMRAWX->data.blocks[i].gnssId = extractByte(msg, offset + 20);
          packetUBXRXMRAWX->data.blocks[i].svId = extractByte(msg, offset + 21);
          packetUBXRXMRAWX->data.blocks[i].sigId = extractByte(msg, offset + 22);
          packetUBXRXMRAWX->data.blocks[i].freqId = extractByte(msg, offset + 23);
          packetUBXRXMRAWX->data.blocks[i].lockTime = extractInt(msg, offset + 24);
          packetUBXRXMRAWX->data.blocks[i].cno = extractByte(msg, offset + 26);
          packetUBXRXMRAWX->data.blocks[i].prStdev = extractByte(msg, offset + 27);
          packetUBXRXMRAWX->data.blocks[i].cpStdev = extractByte(msg, offset + 28);
          packetUBXRXMRAWX->data.blocks[i].doStdev = extractByte(msg, offset + 29);
          packetUBXRXMRAWX->data.blocks[i].trkStat.all = extractByte(msg, offset + 30);
        }

        // Mark all datums as fresh (not read before)
        packetUBXRXMRAWX->moduleQueried = true;

        // Check if we need to copy the data for the callback
        if ((packetUBXRXMRAWX->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXRXMRAWX->callbackData->header.rcvTow[0], &packetUBXRXMRAWX->data.header.rcvTow[0], sizeof(UBX_RXM_RAWX_data_t));
          packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXRXMRAWX->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    break;
  case UBX_CLASS_CFG:
    if (msg->id == UBX_CFG_PRT && msg->len == UBX_CFG_PRT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXCFGPRT != NULL)
      {
        packetUBXCFGPRT->data.portID = extractByte(msg, 0);
        packetUBXCFGPRT->data.txReady.all = extractInt(msg, 2);
        packetUBXCFGPRT->data.mode = extractLong(msg, 4);
        packetUBXCFGPRT->data.baudRate = extractLong(msg, 8);
        packetUBXCFGPRT->data.inProtoMask.all = extractInt(msg, 12);
        packetUBXCFGPRT->data.outProtoMask.all = extractInt(msg, 14);
        packetUBXCFGPRT->data.flags = extractInt(msg, 16);

        // Mark data as valid
        packetUBXCFGPRT->dataValid = true;
      }
    }
    else if (msg->id == UBX_CFG_RATE && msg->len == UBX_CFG_RATE_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXCFGRATE != NULL)
      {
        packetUBXCFGRATE->data.measRate = extractInt(msg, 0);
        packetUBXCFGRATE->data.navRate = extractInt(msg, 2);
        packetUBXCFGRATE->data.timeRef = extractInt(msg, 4);

        // Mark all datums as fresh (not read before)
        packetUBXCFGRATE->moduleQueried.moduleQueried.all = 0xFFFFFFFF;
      }
    }
    break;
  case UBX_CLASS_TIM:
    if (msg->id == UBX_TIM_TM2 && msg->len == UBX_TIM_TM2_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXTIMTM2 != NULL)
      {
        packetUBXTIMTM2->data.ch = extractByte(msg, 0);
        packetUBXTIMTM2->data.flags.all = extractByte(msg, 1);
        packetUBXTIMTM2->data.count = extractInt(msg, 2);
        packetUBXTIMTM2->data.wnR = extractInt(msg, 4);
        packetUBXTIMTM2->data.wnF = extractInt(msg, 6);
        packetUBXTIMTM2->data.towMsR = extractLong(msg, 8);
        packetUBXTIMTM2->data.towSubMsR = extractLong(msg, 12);
        packetUBXTIMTM2->data.towMsF = extractLong(msg, 16);
        packetUBXTIMTM2->data.towSubMsF = extractLong(msg, 20);
        packetUBXTIMTM2->data.accEst = extractLong(msg, 24);

        // Mark all datums as fresh (not read before)
        packetUBXTIMTM2->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXTIMTM2->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXTIMTM2->callbackData->ch, &packetUBXTIMTM2->data.ch, sizeof(UBX_TIM_TM2_data_t));
          packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXTIMTM2->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    break;
  case UBX_CLASS_ESF:
    if (msg->id == UBX_ESF_ALG && msg->len == UBX_ESF_ALG_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFALG != NULL)
      {
        packetUBXESFALG->data.iTOW = extractLong(msg, 0);
        packetUBXESFALG->data.version = extractByte(msg, 4);
        packetUBXESFALG->data.flags.all = extractByte(msg, 5);
        packetUBXESFALG->data.error.all = extractByte(msg, 6);
        packetUBXESFALG->data.yaw = extractLong(msg, 8);
        packetUBXESFALG->data.pitch = extractSignedInt(msg, 12);
        packetUBXESFALG->data.roll = extractSignedInt(msg, 14);

        // Mark all datums as fresh (not read before)
        packetUBXESFALG->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFALG->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFALG->callbackData->iTOW, &packetUBXESFALG->data.iTOW, sizeof(UBX_ESF_ALG_data_t));
          packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFALG->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_INS && msg->len == UBX_ESF_INS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFINS != NULL)
      {
        packetUBXESFINS->data.bitfield0.all = extractLong(msg, 0);
        packetUBXESFINS->data.iTOW = extractLong(msg, 8);
        packetUBXESFINS->data.xAngRate = extractSignedLong(msg, 12);
        packetUBXESFINS->data.yAngRate = extractSignedLong(msg, 16);
        packetUBXESFINS->data.zAngRate = extractSignedLong(msg, 20);
        packetUBXESFINS->data.xAccel = extractSignedLong(msg, 24);
        packetUBXESFINS->data.yAccel = extractSignedLong(msg, 28);
        packetUBXESFINS->data.zAccel = extractSignedLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXESFINS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFINS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFINS->callbackData->bitfield0.all, &packetUBXESFINS->data.bitfield0.all, sizeof(UBX_ESF_INS_data_t));
          packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFINS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_MEAS)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFMEAS != NULL)
      {
        packetUBXESFMEAS->data.timeTag = extractLong(msg, 0);
        packetUBXESFMEAS->data.flags.all = extractInt(msg, 4);
        packetUBXESFMEAS->data.id = extractInt(msg, 6);
        for (uint16_t i = 0; (i < DEF_MAX_NUM_ESF_MEAS) && (i < packetUBXESFMEAS->data.flags.bits.numMeas) && ((i * 4) < (msg->len - 8)); i++)
        {
          packetUBXESFMEAS->data.data[i].data.all = extractLong(msg, 8 + (i * 4));
        }
        if ((uint16_t)msg->len > (uint16_t)(8 + (packetUBXESFMEAS->data.flags.bits.numMeas * 4)))
          packetUBXESFMEAS->data.calibTtag = extractLong(msg, 8 + (packetUBXESFMEAS->data.flags.bits.numMeas * 4));

        // Check if we need to copy the data for the callback
        if ((packetUBXESFMEAS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFMEAS->callbackData->timeTag, &packetUBXESFMEAS->data.timeTag, sizeof(UBX_ESF_MEAS_data_t));
          packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFMEAS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_RAW)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFRAW != NULL)
      {
        packetUBXESFRAW->data.numEsfRawBlocks = (msg->len - 4) / 8; // Record how many blocks were received. Could be 7 or 70 (ZED-F9R vs. NEO-M8U)
        for (uint16_t i = 0; (i < (DEF_NUM_SENS * DEF_MAX_NUM_ESF_RAW_REPEATS)) && ((i * 8) < (msg->len - 4)); i++)
        {
          packetUBXESFRAW->data.data[i].data.all = extractLong(msg, 4 + (i * 8));
          packetUBXESFRAW->data.data[i].sTag = extractLong(msg, 8 + (i * 8));
        }

        // Check if we need to copy the data for the callback
        if ((packetUBXESFRAW->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFRAW->callbackData->data[0].data.all, &packetUBXESFRAW->data.data[0].data.all, sizeof(UBX_ESF_RAW_data_t));
          packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFRAW->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_ESF_STATUS)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXESFSTATUS != NULL)
      {
        packetUBXESFSTATUS->data.iTOW = extractLong(msg, 0);
        packetUBXESFSTATUS->data.version = extractByte(msg, 4);
        packetUBXESFSTATUS->data.fusionMode = extractByte(msg, 12);
        packetUBXESFSTATUS->data.numSens = extractByte(msg, 15);
        for (uint16_t i = 0; (i < DEF_NUM_SENS) && (i < packetUBXESFSTATUS->data.numSens) && ((i * 4) < (msg->len - 16)); i++)
        {
          packetUBXESFSTATUS->data.status[i].sensStatus1.all = extractByte(msg, 16 + (i * 4) + 0);
          packetUBXESFSTATUS->data.status[i].sensStatus2.all = extractByte(msg, 16 + (i * 4) + 1);
          packetUBXESFSTATUS->data.status[i].freq = extractByte(msg, 16 + (i * 4) + 2);
          packetUBXESFSTATUS->data.status[i].faults.all = extractByte(msg, 16 + (i * 4) + 3);
        }

        // Mark all datums as fresh (not read before)
        packetUBXESFSTATUS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXESFSTATUS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXESFSTATUS->callbackData->iTOW, &packetUBXESFSTATUS->data.iTOW, sizeof(UBX_ESF_STATUS_data_t));
          packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXESFSTATUS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    break;
  case UBX_CLASS_MGA:
    if (msg->id == UBX_MGA_ACK_DATA0 && msg->len == UBX_MGA_ACK_DATA0_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXMGAACK != NULL)
      {
        // Calculate how many ACKs are already stored in the ring buffer
        uint8_t ackBufferContains;
        if (packetUBXMGAACK->head >= packetUBXMGAACK->tail) // Check if wrap-around has occurred
        {
          // Wrap-around has not occurred so do a simple subtraction
          ackBufferContains = packetUBXMGAACK->head - packetUBXMGAACK->tail;
        }
        else
        {
          // Wrap-around has occurred so do a simple subtraction but add in the buffer length (UBX_MGA_ACK_RINGBUFFER_LEN)
          ackBufferContains = ((uint8_t)(((uint16_t)packetUBXMGAACK->head + (uint16_t)UBX_MGA_ACK_DATA0_RINGBUFFER_LEN) - (uint16_t)packetUBXMGAACK->tail));
        }
        // Have we got space to store this ACK?
        if (ackBufferContains < (UBX_MGA_ACK_DATA0_RINGBUFFER_LEN - 1))
        {
          // Yes, we have, so store it
          packetUBXMGAACK->data[packetUBXMGAACK->head].type = extractByte(msg, 0);
          packetUBXMGAACK->data[packetUBXMGAACK->head].version = extractByte(msg, 1);
          packetUBXMGAACK->data[packetUBXMGAACK->head].infoCode = extractByte(msg, 2);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgId = extractByte(msg, 3);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[0] = extractByte(msg, 4);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[1] = extractByte(msg, 5);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[2] = extractByte(msg, 6);
          packetUBXMGAACK->data[packetUBXMGAACK->head].msgPayloadStart[3] = extractByte(msg, 7);
          // Increment the head
          packetUBXMGAACK->head++;
          if (packetUBXMGAACK->head == UBX_MGA_ACK_DATA0_RINGBUFFER_LEN)
            packetUBXMGAACK->head = 0;
        }
        else
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial->println(F("processUBXpacket: packetUBXMGAACK is full. ACK will be lost!"));
          }
#endif
        }
      }
    }
    else if (msg->id == UBX_MGA_DBD && msg->len <= UBX_MGA_DBD_LEN) // Message length may be less than UBX_MGA_DBD_LEN. UBX_MGA_DBD_LEN is the maximum it will be.
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXMGADBD != NULL)
      {
        // Calculate how many DBDs are already stored in the ring buffer
        uint8_t dbdBufferContains;
        if (packetUBXMGADBD->head >= packetUBXMGADBD->tail) // Check if wrap-around has occurred
        {
          // Wrap-around has not occurred so do a simple subtraction
          dbdBufferContains = packetUBXMGADBD->head - packetUBXMGADBD->tail;
        }
        else
        {
          // Wrap-around has occurred so do a simple subtraction but add in the buffer length (UBX_MGA_DBD_RINGBUFFER_LEN)
          dbdBufferContains = ((uint8_t)(((uint16_t)packetUBXMGADBD->head + (uint16_t)UBX_MGA_DBD_RINGBUFFER_LEN) - (uint16_t)packetUBXMGADBD->tail));
        }
        // Have we got space to store this DBD?
        if (dbdBufferContains < (UBX_MGA_DBD_RINGBUFFER_LEN - 1))
        {
          // Yes, we have, so store it
          // We need to save the entire message - header, payload and checksum
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryHeader1 = UBX_SYNCH_1;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryHeader2 = UBX_SYNCH_2;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryClass = UBX_CLASS_MGA;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryID = UBX_MGA_DBD;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryLenLSB = (uint8_t)(msg->len & 0xFF); // We need to store the length of the DBD entry. The entry itself does not contain a length...
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryLenMSB = (uint8_t)((msg->len >> 8) & 0xFF);
          for (uint16_t i = 0; i < msg->len; i++)
          {
            packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntry[i] = extractByte(msg, i);
          }
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryChecksumA = msg->checksumA;
          packetUBXMGADBD->data[packetUBXMGADBD->head].dbdEntryChecksumB = msg->checksumB;
          // Increment the head
          packetUBXMGADBD->head++;
          if (packetUBXMGADBD->head == UBX_MGA_DBD_RINGBUFFER_LEN)
            packetUBXMGADBD->head = 0;
        }
        else
        {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
          if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
          {
            _debugSerial->println(F("processUBXpacket: packetUBXMGADBD is full. DBD data will be lost!"));
          }
#endif
        }
      }
    }
    break;
  case UBX_CLASS_HNR:
    if (msg->id == UBX_HNR_PVT && msg->len == UBX_HNR_PVT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRPVT != NULL)
      {
        packetUBXHNRPVT->data.iTOW = extractLong(msg, 0);
        packetUBXHNRPVT->data.year = extractInt(msg, 4);
        packetUBXHNRPVT->data.month = extractByte(msg, 6);
        packetUBXHNRPVT->data.day = extractByte(msg, 7);
        packetUBXHNRPVT->data.hour = extractByte(msg, 8);
        packetUBXHNRPVT->data.min = extractByte(msg, 9);
        packetUBXHNRPVT->data.sec = extractByte(msg, 10);
        packetUBXHNRPVT->data.valid.all = extractByte(msg, 11);
        packetUBXHNRPVT->data.nano = extractSignedLong(msg, 12);
        packetUBXHNRPVT->data.gpsFix = extractByte(msg, 16);
        packetUBXHNRPVT->data.flags.all = extractByte(msg, 17);
        packetUBXHNRPVT->data.lon = extractSignedLong(msg, 20);
        packetUBXHNRPVT->data.lat = extractSignedLong(msg, 24);
        packetUBXHNRPVT->data.height = extractSignedLong(msg, 28);
        packetUBXHNRPVT->data.hMSL = extractSignedLong(msg, 32);
        packetUBXHNRPVT->data.gSpeed = extractSignedLong(msg, 36);
        packetUBXHNRPVT->data.speed = extractSignedLong(msg, 40);
        packetUBXHNRPVT->data.headMot = extractSignedLong(msg, 44);
        packetUBXHNRPVT->data.headVeh = extractSignedLong(msg, 48);
        packetUBXHNRPVT->data.hAcc = extractLong(msg, 52);
        packetUBXHNRPVT->data.vAcc = extractLong(msg, 56);
        packetUBXHNRPVT->data.sAcc = extractLong(msg, 60);
        packetUBXHNRPVT->data.headAcc = extractLong(msg, 64);

        // Mark all datums as fresh (not read before)
        packetUBXHNRPVT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRPVT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRPVT->callbackData->iTOW, &packetUBXHNRPVT->data.iTOW, sizeof(UBX_HNR_PVT_data_t));
          packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRPVT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_HNR_ATT && msg->len == UBX_HNR_ATT_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRATT != NULL)
      {
        packetUBXHNRATT->data.iTOW = extractLong(msg, 0);
        packetUBXHNRATT->data.version = extractByte(msg, 4);
        packetUBXHNRATT->data.roll = extractSignedLong(msg, 8);
        packetUBXHNRATT->data.pitch = extractSignedLong(msg, 12);
        packetUBXHNRATT->data.heading = extractSignedLong(msg, 16);
        packetUBXHNRATT->data.accRoll = extractLong(msg, 20);
        packetUBXHNRATT->data.accPitch = extractLong(msg, 24);
        packetUBXHNRATT->data.accHeading = extractLong(msg, 28);

        // Mark all datums as fresh (not read before)
        packetUBXHNRATT->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRATT->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRATT->callbackData->iTOW, &packetUBXHNRATT->data.iTOW, sizeof(UBX_HNR_ATT_data_t));
          packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRATT->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    else if (msg->id == UBX_HNR_INS && msg->len == UBX_HNR_INS_LEN)
    {
      // Parse various byte fields into storage - but only if we have memory allocated for it
      if (packetUBXHNRINS != NULL)
      {
        packetUBXHNRINS->data.bitfield0.all = extractLong(msg, 0);
        packetUBXHNRINS->data.iTOW = extractLong(msg, 8);
        packetUBXHNRINS->data.xAngRate = extractSignedLong(msg, 12);
        packetUBXHNRINS->data.yAngRate = extractSignedLong(msg, 16);
        packetUBXHNRINS->data.zAngRate = extractSignedLong(msg, 20);
        packetUBXHNRINS->data.xAccel = extractSignedLong(msg, 24);
        packetUBXHNRINS->data.yAccel = extractSignedLong(msg, 28);
        packetUBXHNRINS->data.zAccel = extractSignedLong(msg, 32);

        // Mark all datums as fresh (not read before)
        packetUBXHNRINS->moduleQueried.moduleQueried.all = 0xFFFFFFFF;

        // Check if we need to copy the data for the callback
        if ((packetUBXHNRINS->callbackData != NULL)                                     // If RAM has been allocated for the copy of the data
            && (packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid == false)) // AND the data is stale
        {
          memcpy(&packetUBXHNRINS->callbackData->bitfield0.all, &packetUBXHNRINS->data.bitfield0.all, sizeof(UBX_HNR_INS_data_t));
          packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid = true;
        }

        // Check if we need to copy the data into the file buffer
        if (packetUBXHNRINS->automaticFlags.flags.bits.addToFileBuffer)
        {
          storePacket(msg);
        }
      }
    }
    break;
  }
}

// Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
// This is called before we send a command message
void SFE_UBLOX_GNSS::calcChecksum(ubxPacket *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}

// Given a message and a byte, add to rolling "8-Bit Fletcher" checksum
// This is used when receiving messages from module
void SFE_UBLOX_GNSS::addToChecksum(uint8_t incoming)
{
  rollingChecksumA += incoming;
  rollingChecksumB += rollingChecksumA;
}

// Given a packet and payload, send everything including CRC bytes via I2C port
sfe_ublox_status_e SFE_UBLOX_GNSS::sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait, bool expectACKonly)
{
  sfe_ublox_status_e retVal = SFE_UBLOX_STATUS_SUCCESS;

  calcChecksum(outgoingUBX); // Sets checksum A and B bytes of the packet

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("\nSending: "));
    printPacket(outgoingUBX, true); // Always print payload
  }
#endif

  if (commType == COMM_TYPE_I2C)
  {
    retVal = sendI2cCommand(outgoingUBX, maxWait);
    if (retVal != SFE_UBLOX_STATUS_SUCCESS)
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("Send I2C Command failed"));
      }
#endif
      return retVal;
    }
  }
  else if (commType == COMM_TYPE_SERIAL)
  {
    sendSerialCommand(outgoingUBX);
  }
  else if (commType == COMM_TYPE_SPI)
  {
    sendSpiCommand(outgoingUBX);
  }

  if (maxWait > 0)
  {
    // Depending on what we just sent, either we need to look for an ACK or not
    if ((outgoingUBX->cls == UBX_CLASS_CFG) || (expectACKonly == true))
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("sendCommand: Waiting for ACK response"));
      }
#endif
      retVal = waitForACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
    else
    {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->println(F("sendCommand: Waiting for No ACK response"));
      }
#endif
      retVal = waitForNoACKResponse(outgoingUBX, outgoingUBX->cls, outgoingUBX->id, maxWait); // Wait for Ack response
    }
  }
  return retVal;
}

// Returns false if sensor fails to respond to I2C traffic
sfe_ublox_status_e SFE_UBLOX_GNSS::sendI2cCommand(ubxPacket *outgoingUBX, uint16_t maxWait)
{
  // From the integration guide:
  // "The receiver does not provide any write access except for writing UBX and NMEA messages to the
  //  receiver, such as configuration or aiding data. Therefore, the register set mentioned in section Read
  //  Access is not writeable. Following the start condition from the master, the 7-bit device address and
  //  the RW bit (which is a logic low for write access) are clocked onto the bus by the master transmitter.
  //  The receiver answers with an acknowledge (logic low) to indicate that it is responsible for the given
  //  address. Now, the master can write 2 to N bytes to the receiver, generating a stop condition after the
  //  last byte being written. The number of data bytes must be at least 2 to properly distinguish from
  //  the write access to set the address counter in random read accesses."
  // I take two things from this:
  // 1) We do not need to write 0xFF to point at register 0xFF. We're already pointing at it.
  // 2) We must always write at least 2 bytes, otherwise it looks like we are starting to do a read.
  // Point 2 is important. It means:
  // * In this function:
  //     if we do multiple writes (because we're trying to write more than i2cTransactionSize),
  //     we may need to write one byte less in the penultimate write to ensure we always have two bytes left for the final write.
  // * In pushRawData:
  //     if there is one byte to write, or one byte left to write, we need to do the same thing and may need to store a single
  //     byte until pushRawData is called again.
  // The next four lines can be commented. We do not need to point at the 0xFF data register
  //_i2cPort->beginTransmission((uint8_t)_gpsI2Caddress); //There is no register to write to, we just begin writing data bytes
  //_i2cPort->write(0xFF);
  // if (_i2cPort->endTransmission(false) != 0)         //Don't release bus
  //  return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); //Sensor did not ACK

  // The total number of bytes to be written is: payload len + 8
  // UBX_SYNCH_1
  // UBX_SYNCH_2
  // cls
  // id
  // len (MSB)
  // len (LSB)
  // < payload >
  // checksumA
  // checksumB

  // i2cTransactionSize will be at least 8. We don't need to check for smaller values than that.

  uint16_t bytesToSend = outgoingUBX->len + 8; // How many bytes need to be sent
  uint16_t bytesSent = 0;                      // How many bytes have been sent
  uint16_t bytesLeftToSend = bytesToSend;      // How many bytes remain to be sent
  uint16_t startSpot = 0;                      // Payload pointer

  while (bytesLeftToSend > 0)
  {
    uint16_t len = bytesLeftToSend; // How many bytes should we actually write?
    if (len > i2cTransactionSize)   // Limit len to i2cTransactionSize
      len = i2cTransactionSize;

    bytesLeftToSend -= len; // Calculate how many bytes will be left after we do this write

    // If bytesLeftToSend is zero, that's OK.
    // If bytesLeftToSend is >= 2, that's OK.
    // But if bytesLeftToSend is 1, we need to adjust len to make sure we write at least 2 bytes in the final write
    if (bytesLeftToSend == 1)
    {
      len -= 1;             // Decrement len by 1
      bytesLeftToSend += 1; // Increment bytesLeftToSend by 1
    }

    _i2cPort->beginTransmission((uint8_t)_gpsI2Caddress); // Start the transmission

    if (bytesSent == 0) // Is this the first write? If it is, write the header bytes
    {
      _i2cPort->write(UBX_SYNCH_1); //μ - oh ublox, you're funny. I will call you micro-blox from now on.
      _i2cPort->write(UBX_SYNCH_2); // b
      _i2cPort->write(outgoingUBX->cls);
      _i2cPort->write(outgoingUBX->id);
      _i2cPort->write(outgoingUBX->len & 0xFF); // LSB
      _i2cPort->write(outgoingUBX->len >> 8);   // MSB

      bytesSent += 6;

      uint16_t x = 0;
      // Write a portion of the payload to the bus.
      // Keep going until we reach the end of the payload (x == outgoingUBX->len)
      // or we've sent as many bytes as we can in this transmission (bytesSent == len).
      for (; (x < outgoingUBX->len) && (bytesSent < len); x++)
      {
        _i2cPort->write(outgoingUBX->payload[startSpot + x]);
        bytesSent++;
      }
      startSpot += x;

      // Can we write both checksum bytes?
      // We can send both bytes now if we have exactly 2 bytes left
      // to be sent in this transmission (bytesSent == (len - 2)).
      if (bytesSent == (len - 2))
      {
        // Write checksum
        _i2cPort->write(outgoingUBX->checksumA);
        _i2cPort->write(outgoingUBX->checksumB);
        bytesSent += 2;
      }
    }
    else // Keep writing payload bytes. Write the checksum at the right time.
    {
      uint16_t x = 0;
      // Write a portion of the payload to the bus.
      // Keep going until we've sent as many bytes as we can in this transmission (x == len)
      // or until we reach the end of the payload ((startSpot + x) == (outgoingUBX->len))
      for (; (x < len) && ((startSpot + x) < (outgoingUBX->len)); x++)
      {
        _i2cPort->write(outgoingUBX->payload[startSpot + x]);
        bytesSent++;
      }
      startSpot += x;

      // Can we write both checksum bytes?
      // We can send both bytes if we have exactly 2 bytes left to be sent (bytesSent == (bytesToSend - 2))
      // and if there is room for 2 bytes in this transmission
      if ((bytesSent == (bytesToSend - 2)) && (x == (len - 2)))
      {
        // Write checksum
        _i2cPort->write(outgoingUBX->checksumA);
        _i2cPort->write(outgoingUBX->checksumB);
        bytesSent += 2;
      }
    }

    if (bytesSent < bytesToSend) // Do we need to go round the loop again?
    {
      if (_i2cPort->endTransmission(_i2cStopRestart) != 0) // Don't release bus unless we have to
        return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE);        // Sensor did not ACK
    }
  }

  // All done transmitting bytes. Release bus.
  if (_i2cPort->endTransmission() != 0)
    return (SFE_UBLOX_STATUS_I2C_COMM_FAILURE); // Sensor did not ACK

  (void)maxWait; // Do something with maxWait just to avoid the pesky compiler warnings!

  return (SFE_UBLOX_STATUS_SUCCESS);
}

// Given a packet and payload, send everything including CRC bytesA via Serial port
void SFE_UBLOX_GNSS::sendSerialCommand(ubxPacket *outgoingUBX)
{
  // Write header bytes
  _serialPort->write(UBX_SYNCH_1); //μ - oh ublox, you're funny. I will call you micro-blox from now on.
  _serialPort->write(UBX_SYNCH_2); // b
  _serialPort->write(outgoingUBX->cls);
  _serialPort->write(outgoingUBX->id);
  _serialPort->write(outgoingUBX->len & 0xFF); // LSB
  _serialPort->write(outgoingUBX->len >> 8);   // MSB

  // Write payload.
  for (uint16_t i = 0; i < outgoingUBX->len; i++)
  {
    _serialPort->write(outgoingUBX->payload[i]);
  }

  // Write checksum
  _serialPort->write(outgoingUBX->checksumA);
  _serialPort->write(outgoingUBX->checksumB);
}

// Transfer a byte to SPI. Also capture any bytes received from the UBLOX device during sending and capture them in a small buffer so that
// they can be processed later with process
void SFE_UBLOX_GNSS::spiTransfer(uint8_t byteToTransfer)
{
  uint8_t returnedByte = _spiPort->transfer(byteToTransfer);
  if ((spiBufferIndex < getSpiTransactionSize()) && (returnedByte != 0xFF || currentSentence != SFE_UBLOX_SENTENCE_TYPE_NONE))
  {
    spiBuffer[spiBufferIndex] = returnedByte;
    spiBufferIndex++;
  }
}

// Send a command via SPI
void SFE_UBLOX_GNSS::sendSpiCommand(ubxPacket *outgoingUBX)
{
  if (spiBuffer == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->print(F("sendSpiCommand: no memory allocation for SPI Buffer!"));
    }
#endif
    return;
  }

  // Start at the beginning of the SPI buffer
  spiBufferIndex = 0;

  _spiPort->beginTransaction(SPISettings(_spiSpeed, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  // Write header bytes
  spiTransfer(UBX_SYNCH_1); //μ - oh ublox, you're funny. I will call you micro-blox from now on.
  spiTransfer(UBX_SYNCH_2); // b

  spiTransfer(outgoingUBX->cls);
  spiTransfer(outgoingUBX->id);
  spiTransfer(outgoingUBX->len & 0xFF); // LSB
  spiTransfer(outgoingUBX->len >> 8);

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug)
  {
    _debugSerial->print(F("sendSpiCommand: "));
    _debugSerial->print(UBX_SYNCH_1, HEX);
    _debugSerial->print(F(" "));
    _debugSerial->print(UBX_SYNCH_2, HEX);
    _debugSerial->print(F(" "));
    _debugSerial->print(outgoingUBX->cls, HEX);
    _debugSerial->print(F(" "));
    _debugSerial->print(outgoingUBX->id, HEX);
    _debugSerial->print(F(" "));
    _debugSerial->print(outgoingUBX->len & 0xFF, HEX);
    _debugSerial->print(F(" "));
    _debugSerial->print(outgoingUBX->len >> 8, HEX);
  }
#endif

  // Write payload.
  for (uint16_t i = 0; i < outgoingUBX->len; i++)
  {
    spiTransfer(outgoingUBX->payload[i]);
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug)
    {
      _debugSerial->print(F(" "));
      _debugSerial->print(outgoingUBX->payload[i], HEX);
    }
#endif
  }

  // Write checksum
  spiTransfer(outgoingUBX->checksumA);
  spiTransfer(outgoingUBX->checksumB);
  digitalWrite(_csPin, HIGH);
  _spiPort->endTransaction();

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug)
  {
    _debugSerial->print(F(" "));
    _debugSerial->print(outgoingUBX->checksumA, HEX);
    _debugSerial->print(F(" "));
    _debugSerial->println(outgoingUBX->checksumB, HEX);
  }
#endif
}

// Pretty prints the current ubxPacket
void SFE_UBLOX_GNSS::printPacket(ubxPacket *packet, bool alwaysPrintPayload)
{
  // Only print the payload is ignoreThisPayload is false otherwise
  // we could be printing gibberish from beyond the end of packetBuf
  // (These two lines get rid of a pesky compiler warning)
  bool printPayload = (ignoreThisPayload == false);
  printPayload |= (alwaysPrintPayload == true);

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("CLS:"));
    if (packet->cls == UBX_CLASS_NAV) // 1
      _debugSerial->print(F("NAV"));
    else if (packet->cls == UBX_CLASS_ACK) // 5
      _debugSerial->print(F("ACK"));
    else if (packet->cls == UBX_CLASS_CFG) // 6
      _debugSerial->print(F("CFG"));
    else if (packet->cls == UBX_CLASS_MON) // 0x0A
      _debugSerial->print(F("MON"));
    else
    {
      _debugSerial->print(F("0x"));
      _debugSerial->print(packet->cls, HEX);
    }

    _debugSerial->print(F(" ID:"));
    if (packet->cls == UBX_CLASS_NAV && packet->id == UBX_NAV_PVT)
      _debugSerial->print(F("PVT"));
    else if (packet->cls == UBX_CLASS_CFG && packet->id == UBX_CFG_RATE)
      _debugSerial->print(F("RATE"));
    else if (packet->cls == UBX_CLASS_CFG && packet->id == UBX_CFG_CFG)
      _debugSerial->print(F("SAVE"));
    else
    {
      _debugSerial->print(F("0x"));
      _debugSerial->print(packet->id, HEX);
    }

    _debugSerial->print(F(" Len: 0x"));
    _debugSerial->print(packet->len, HEX);

    if (printPayload)
    {
      _debugSerial->print(F(" Payload:"));

      for (uint16_t x = 0; x < packet->len; x++)
      {
        _debugSerial->print(F(" "));
        _debugSerial->print(packet->payload[x], HEX);
      }
    }
    else
    {
      _debugSerial->print(F(" Payload: IGNORED"));
    }
    _debugSerial->println();
  }
#else
  if (_printDebug == true)
  {
    _debugSerial->print(F("Len: 0x"));
    _debugSerial->print(packet->len, HEX);
  }
#endif
}

// When messages from the class CFG are sent to the receiver, the receiver will send an "acknowledge"(UBX - ACK - ACK) or a
//"not acknowledge"(UBX-ACK-NAK) message back to the sender, depending on whether or not the message was processed correctly.
// Some messages from other classes also use the same acknowledgement mechanism.

// When we poll or get a setting, we will receive _both_ a config packet and an ACK
// If the poll or get request is not valid, we will receive _only_ a NACK

// If we are trying to get or poll a setting, then packetCfg.len will be 0 or 1 when the packetCfg is _sent_.
// If we poll the setting for a particular port using UBX-CFG-PRT then .len will be 1 initially
// For all other gets or polls, .len will be 0 initially
//(It would be possible for .len to be 2 _if_ we were using UBX-CFG-MSG to poll the settings for a particular message - but we don't use that (currently))

// If the get or poll _fails_, i.e. is NACK'd, then packetCfg.len could still be 0 or 1 after the NACK is received
// But if the get or poll is ACK'd, then packetCfg.len will have been updated by the incoming data and will always be at least 2

// If we are going to set the value for a setting, then packetCfg.len will be at least 3 when the packetCfg is _sent_.
//(UBX-CFG-MSG appears to have the shortest set length of 3 bytes)

// We need to think carefully about how interleaved PVT packets affect things.
// It is entirely possible that our packetCfg and packetAck were received successfully
// but while we are still in the "if (checkUblox() == true)" loop a PVT packet is processed
// or _starts_ to arrive (remember that Serial data can arrive very slowly).

// Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got an ACK and a valid packetCfg (module is responding with register content)
// Returns SFE_UBLOX_STATUS_DATA_SENT if we got an ACK and no packetCfg (no valid packetCfg needed, module absorbs new register data)
// Returns SFE_UBLOX_STATUS_FAIL if something very bad happens (e.g. a double checksum failure)
// Returns SFE_UBLOX_STATUS_COMMAND_NACK if the packet was not-acknowledged (NACK)
// Returns SFE_UBLOX_STATUS_CRC_FAIL if we had a checksum failure
// Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
// Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an ACK and a valid packetCfg but that the packetCfg has been
//  or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e SFE_UBLOX_GNSS::waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() < (startTime + (unsigned long)maxTime))
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
    {
      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: valid data and valid ACK received after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and a correct ACK!
      }

      // We can be confident that the data packet (if we are going to get one) will always arrive
      // before the matching ACK. So if we sent a config packet which only produces an ACK
      // then outgoingUBX->classAndIDmatch will be NOT_DEFINED and the packetAck.classAndIDmatch will VALID.
      // We should not check outgoingUBX->valid, outgoingUBX->cls or outgoingUBX->id
      // as these may have been changed by an automatic packet.
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: no data and valid ACK after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_SENT); // We got an ACK but no data...
      }

      // If both the outgoingUBX->classAndIDmatch and packetAck.classAndIDmatch are VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by an automatic packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: data being OVERWRITTEN after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If packetAck.classAndIDmatch is VALID but both outgoingUBX->valid and outgoingUBX->classAndIDmatch
      // are NOT_VALID then we can be confident we have had a checksum failure on the data packet
      else if ((packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: CRC failed after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_CRC_FAIL); // Checksum fail
      }

      // If our packet was not-acknowledged (NACK) we do not receive a data packet - we only get the NACK.
      // So you would expect outgoingUBX->valid and outgoingUBX->classAndIDmatch to still be NOT_DEFINED
      // But if a full PVT packet arrives afterwards outgoingUBX->valid could be VALID (or just possibly NOT_VALID)
      // but outgoingUBX->cls and outgoingUBX->id would not match...
      // So I think this is telling us we need a special state for packetAck.classAndIDmatch to tell us
      // the packet was definitely NACK'd otherwise we are possibly just guessing...
      // Note: the addition of packetBuf changes the logic of this, but we'll leave the code as is for now.
      else if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_NOTACKNOWLEDGED)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: data was NOTACKNOWLEDGED (NACK) after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_COMMAND_NACK); // We received a NACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID but the packetAck.classAndIDmatch is NOT_VALID
      // then the ack probably had a checksum error. We will take a gamble and return DATA_RECEIVED.
      // If we were playing safe, we should return FAIL instead
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: VALID data and INVALID ACK received after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID and the packetAck.classAndIDmatch is NOT_VALID
      // then we return a FAIL. This must be a double checksum failure?
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForACKResponse: INVALID data and INVALID ACK received after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_FAIL); // We received invalid data and an invalid ACK!
      }

      // If the outgoingUBX->classAndIDmatch is VALID and the packetAck.classAndIDmatch is NOT_DEFINED
      // then the ACK has not yet been received and we should keep waiting for it
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForACKResponse: valid data after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->println(F(" msec. Waiting for ACK."));
        // }
      }

    } // checkUbloxInternal == true

    delay(1); // Allow an RTOS to get an elbow in (#11)
  }           // while (millis() < (startTime + (unsigned long)maxTime))

  // We have timed out...
  // If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
  // even though we did not get an ACK
  if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->print(F("waitForACKResponse: TIMEOUT with valid data after "));
      _debugSerial->print(millis() - startTime);
      _debugSerial->println(F(" msec. "));
    }
#endif
    return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data... But no ACK!
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("waitForACKResponse: TIMEOUT after "));
    _debugSerial->print(millis() - startTime);
    _debugSerial->println(F(" msec."));
  }
#endif

  return (SFE_UBLOX_STATUS_TIMEOUT);
}

// For non-CFG queries no ACK is sent so we use this function
// Returns SFE_UBLOX_STATUS_DATA_RECEIVED if we got a config packet full of response data that has CLS/ID match to our query packet
// Returns SFE_UBLOX_STATUS_CRC_FAIL if we got a corrupt config packet that has CLS/ID match to our query packet
// Returns SFE_UBLOX_STATUS_TIMEOUT if we timed out
// Returns SFE_UBLOX_STATUS_DATA_OVERWRITTEN if we got an a valid packetCfg but that the packetCfg has been
//  or is currently being overwritten (remember that Serial data can arrive very slowly)
sfe_ublox_status_e SFE_UBLOX_GNSS::waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
  outgoingUBX->valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a response to the packet we sent
  packetAck.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.valid = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  outgoingUBX->classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED; // This will go VALID (or NOT_VALID) when we receive a packet that matches the requested class and ID
  packetAck.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetBuf.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;
  packetAuto.classAndIDmatch = SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED;

  unsigned long startTime = millis();
  while (millis() - startTime < maxTime)
  {
    if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
    {

      // If outgoingUBX->classAndIDmatch is VALID
      // and outgoingUBX->valid is _still_ VALID and the class and ID _still_ match
      // then we can be confident that the data in outgoingUBX is valid
      if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForNoACKResponse: valid data with CLS/ID match after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data!
      }

      // If the outgoingUBX->classAndIDmatch is VALID
      // but the outgoingUBX->cls or ID no longer match then we can be confident that we had
      // valid data but it has been or is currently being overwritten by another packet (e.g. PVT).
      // If (e.g.) a PVT packet is _being_ received: outgoingUBX->valid will be NOT_DEFINED
      // If (e.g.) a PVT packet _has been_ received: outgoingUBX->valid will be VALID (or just possibly NOT_VALID)
      // So we cannot use outgoingUBX->valid as part of this check.
      // Note: the addition of packetBuf should make this check redundant!
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForNoACKResponse: data being OVERWRITTEN after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
      }

      // If outgoingUBX->classAndIDmatch is NOT_DEFINED
      // and outgoingUBX->valid is VALID then this must be (e.g.) a PVT packet
      else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID))
      {
        // if (_printDebug == true)
        // {
        //   _debugSerial->print(F("waitForNoACKResponse: valid but UNWANTED data after "));
        //   _debugSerial->print(millis() - startTime);
        //   _debugSerial->print(F(" msec. Class: "));
        //   _debugSerial->print(outgoingUBX->cls);
        //   _debugSerial->print(F(" ID: "));
        //   _debugSerial->print(outgoingUBX->id);
        // }
      }

      // If the outgoingUBX->classAndIDmatch is NOT_VALID then we return CRC failure
      else if (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if (_printDebug == true)
        {
          _debugSerial->print(F("waitForNoACKResponse: CLS/ID match but failed CRC after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" msec"));
        }
#endif
        return (SFE_UBLOX_STATUS_CRC_FAIL); // We received invalid data
      }
    }

    delay(1); // Allow an RTOS to get an elbow in (#11)
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("waitForNoACKResponse: TIMEOUT after "));
    _debugSerial->print(millis() - startTime);
    _debugSerial->println(F(" msec. No packet received."));
  }
#endif

  return (SFE_UBLOX_STATUS_TIMEOUT);
}

// Check if any callbacks are waiting to be processed
void SFE_UBLOX_GNSS::checkCallbacks(void)
{
  if (checkCallbacksReentrant == true) // Check for reentry (i.e. checkCallbacks has been called from inside a callback)
    return;

  checkCallbacksReentrant = true;

  if ((packetUBXNAVPOSECEF != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVPOSECEF->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVPOSECEF->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVPOSECEF->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV POSECEF"));
      packetUBXNAVPOSECEF->callbackPointer(*packetUBXNAVPOSECEF->callbackData); // Call the callback
    }
    if (packetUBXNAVPOSECEF->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV POSECEF"));
      packetUBXNAVPOSECEF->callbackPointerPtr(packetUBXNAVPOSECEF->callbackData); // Call the callback
    }
    packetUBXNAVPOSECEF->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVSTATUS != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVSTATUS->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVSTATUS->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV STATUS"));
      packetUBXNAVSTATUS->callbackPointer(*packetUBXNAVSTATUS->callbackData); // Call the callback
    }
    if (packetUBXNAVSTATUS->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV STATUS"));
      packetUBXNAVSTATUS->callbackPointerPtr(packetUBXNAVSTATUS->callbackData); // Call the callback
    }
    packetUBXNAVSTATUS->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVDOP != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVDOP->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVDOP->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV DOP"));
      packetUBXNAVDOP->callbackPointer(*packetUBXNAVDOP->callbackData); // Call the callback
    }
    if (packetUBXNAVDOP->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV DOP"));
      packetUBXNAVDOP->callbackPointerPtr(packetUBXNAVDOP->callbackData); // Call the callback
    }
    packetUBXNAVDOP->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVATT != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVATT->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVATT->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV ATT"));
      packetUBXNAVATT->callbackPointer(*packetUBXNAVATT->callbackData); // Call the callback
    }
    if (packetUBXNAVATT->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV ATT"));
      packetUBXNAVATT->callbackPointerPtr(packetUBXNAVATT->callbackData); // Call the callback
    }
    packetUBXNAVATT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVPVT != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVPVT->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVPVT->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV PVT"));
      packetUBXNAVPVT->callbackPointer(*packetUBXNAVPVT->callbackData); // Call the callback
    }
    if (packetUBXNAVPVT->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV PVT"));
      packetUBXNAVPVT->callbackPointerPtr(packetUBXNAVPVT->callbackData); // Call the callback
    }
    packetUBXNAVPVT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVODO != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVODO->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVODO->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV ODO"));
      packetUBXNAVODO->callbackPointer(*packetUBXNAVODO->callbackData); // Call the callback
    }
    if (packetUBXNAVODO->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV ODO"));
      packetUBXNAVODO->callbackPointerPtr(packetUBXNAVODO->callbackData); // Call the callback
    }
    packetUBXNAVODO->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVVELECEF != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVVELECEF->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVVELECEF->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV VELECEF"));
      packetUBXNAVVELECEF->callbackPointer(*packetUBXNAVVELECEF->callbackData); // Call the callback
    }
    if (packetUBXNAVVELECEF->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV VELECEF"));
      packetUBXNAVVELECEF->callbackPointerPtr(packetUBXNAVVELECEF->callbackData); // Call the callback
    }
    packetUBXNAVVELECEF->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVVELNED != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVVELNED->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVVELNED->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV VELNED"));
      packetUBXNAVVELNED->callbackPointer(*packetUBXNAVVELNED->callbackData); // Call the callback
    }
    if (packetUBXNAVVELNED->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV VELNED"));
      packetUBXNAVVELNED->callbackPointerPtr(packetUBXNAVVELNED->callbackData); // Call the callback
    }
    packetUBXNAVVELNED->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVHPPOSECEF != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVHPPOSECEF->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVHPPOSECEF->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV HPPOSECEF"));
      packetUBXNAVHPPOSECEF->callbackPointer(*packetUBXNAVHPPOSECEF->callbackData); // Call the callback
    }
    if (packetUBXNAVHPPOSECEF->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV HPPOSECEF"));
      packetUBXNAVHPPOSECEF->callbackPointerPtr(packetUBXNAVHPPOSECEF->callbackData); // Call the callback
    }
    packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVHPPOSLLH != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVHPPOSLLH->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVHPPOSLLH->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV HPPOSLLH"));
      packetUBXNAVHPPOSLLH->callbackPointer(*packetUBXNAVHPPOSLLH->callbackData); // Call the callback
    }
    if (packetUBXNAVHPPOSLLH->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV HPPOSLLH"));
      packetUBXNAVHPPOSLLH->callbackPointerPtr(packetUBXNAVHPPOSLLH->callbackData); // Call the callback
    }
    packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVPVAT != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVPVAT->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVPVAT->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV PVAT"));
      packetUBXNAVPVAT->callbackPointer(*packetUBXNAVPVAT->callbackData); // Call the callback
    }
    if (packetUBXNAVPVAT->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV PVAT"));
      packetUBXNAVPVAT->callbackPointerPtr(packetUBXNAVPVAT->callbackData); // Call the callback
    }
    packetUBXNAVPVAT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVTIMEUTC != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVTIMEUTC->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVTIMEUTC->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV TIMEUTC"));
      packetUBXNAVTIMEUTC->callbackPointerPtr(packetUBXNAVTIMEUTC->callbackData); // Call the callback
    }
    packetUBXNAVTIMEUTC->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVCLOCK != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVCLOCK->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVCLOCK->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV CLOCK"));
      packetUBXNAVCLOCK->callbackPointer(*packetUBXNAVCLOCK->callbackData); // Call the callback
    }
    if (packetUBXNAVCLOCK->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV CLOCK"));
      packetUBXNAVCLOCK->callbackPointerPtr(packetUBXNAVCLOCK->callbackData); // Call the callback
    }
    packetUBXNAVCLOCK->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVSVIN != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVSVIN->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVSVIN->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV SVIN"));
      packetUBXNAVSVIN->callbackPointerPtr(packetUBXNAVSVIN->callbackData); // Call the callback
    }
    packetUBXNAVSVIN->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVSAT != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVSAT->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVSAT->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV SAT"));
      packetUBXNAVSAT->callbackPointer(*packetUBXNAVSAT->callbackData); // Call the callback
    }
    if (packetUBXNAVSAT->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV SAT"));
      packetUBXNAVSAT->callbackPointerPtr(packetUBXNAVSAT->callbackData); // Call the callback
    }
    packetUBXNAVSAT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVRELPOSNED != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVRELPOSNED->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVRELPOSNED->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV RELPOSNED"));
      packetUBXNAVRELPOSNED->callbackPointer(*packetUBXNAVRELPOSNED->callbackData); // Call the callback
    }
    if (packetUBXNAVRELPOSNED->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV RELPOSNED"));
      packetUBXNAVRELPOSNED->callbackPointerPtr(packetUBXNAVRELPOSNED->callbackData); // Call the callback
    }
    packetUBXNAVRELPOSNED->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVAOPSTATUS != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVAOPSTATUS->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVAOPSTATUS->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for NAV AOPSTATUS"));
      packetUBXNAVAOPSTATUS->callbackPointer(*packetUBXNAVAOPSTATUS->callbackData); // Call the callback
    }
    if (packetUBXNAVAOPSTATUS->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV AOPSTATUS"));
      packetUBXNAVAOPSTATUS->callbackPointerPtr(packetUBXNAVAOPSTATUS->callbackData); // Call the callback
    }
    packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXNAVEOE != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXNAVEOE->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXNAVEOE->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for NAV EOE"));
      packetUBXNAVEOE->callbackPointerPtr(packetUBXNAVEOE->callbackData); // Call the callback
    }
    packetUBXNAVEOE->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXRXMPMP != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXRXMPMP->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXRXMPMP->callbackPointerPtr != NULL)                           // If the pointer to the callback has been defined
      && (packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    // if (_printDebug == true)
    //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for RXM PMP"));
    packetUBXRXMPMP->callbackPointerPtr(packetUBXRXMPMP->callbackData);   // Call the callback
    packetUBXRXMPMP->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXRXMPMPmessage != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXRXMPMPmessage->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXRXMPMPmessage->callbackPointerPtr != NULL)                           // If the pointer to the callback has been defined
      && (packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    // if (_printDebug == true)
    //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for RXM PMP message"));
    packetUBXRXMPMPmessage->callbackPointerPtr(packetUBXRXMPMPmessage->callbackData); // Call the callback
    packetUBXRXMPMPmessage->automaticFlags.flags.bits.callbackCopyValid = false;      // Mark the data as stale
  }

  if ((packetUBXRXMQZSSL6message != NULL) &&                   // If RAM has been allocated for message storage
      (packetUBXRXMQZSSL6message->callbackData != NULL) &&     // If RAM has been allocated for the copy of the data
      (packetUBXRXMQZSSL6message->callbackPointerPtr != NULL)) // If the pointer to the callback has been defined
  {
    for (int ch = 0; ch < UBX_RXM_QZSSL6_NUM_CHANNELS; ch++)
    {
      if (packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid & (1 << ch)) // If the copy of the data is valid
      {
        packetUBXRXMQZSSL6message->callbackPointerPtr(&packetUBXRXMQZSSL6message->callbackData[ch]); // Call the callback
        packetUBXRXMQZSSL6message->automaticFlags.flags.bits.callbackCopyValid &= ~(1 << ch);        // clear it
      }
    }
  }

  if ((packetUBXRXMCOR != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXRXMCOR->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXRXMCOR->callbackPointerPtr != NULL)                           // If the pointer to the callback has been defined
      && (packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    // if (_printDebug == true)
    //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for RXM COR"));
    packetUBXRXMCOR->callbackPointerPtr(packetUBXRXMCOR->callbackData);   // Call the callback
    packetUBXRXMCOR->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXRXMSFRBX != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXRXMSFRBX->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXRXMSFRBX->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for RXM SFRBX"));
      packetUBXRXMSFRBX->callbackPointer(*packetUBXRXMSFRBX->callbackData); // Call the callback
    }
    if (packetUBXRXMSFRBX->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for RXM SFRBX"));
      packetUBXRXMSFRBX->callbackPointerPtr(packetUBXRXMSFRBX->callbackData); // Call the callback
    }
    packetUBXRXMSFRBX->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXRXMRAWX != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXRXMRAWX->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXRXMRAWX->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for RXM RAWX"));
      packetUBXRXMRAWX->callbackPointer(*packetUBXRXMRAWX->callbackData); // Call the callback
    }
    if (packetUBXRXMRAWX->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for RXM RAWX"));
      packetUBXRXMRAWX->callbackPointerPtr(packetUBXRXMRAWX->callbackData); // Call the callback
    }
    packetUBXRXMRAWX->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXTIMTM2 != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXTIMTM2->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXTIMTM2->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for TIM TM2"));
      packetUBXTIMTM2->callbackPointer(*packetUBXTIMTM2->callbackData); // Call the callback
    }
    if (packetUBXTIMTM2->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for TIM TM2"));
      packetUBXTIMTM2->callbackPointerPtr(packetUBXTIMTM2->callbackData); // Call the callback
    }
    packetUBXTIMTM2->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXESFALG != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXESFALG->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXESFALG->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for ESF ALG"));
      packetUBXESFALG->callbackPointer(*packetUBXESFALG->callbackData); // Call the callback
    }
    if (packetUBXESFALG->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for ESF ALG"));
      packetUBXESFALG->callbackPointerPtr(packetUBXESFALG->callbackData); // Call the callback
    }
    packetUBXESFALG->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXESFINS != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXESFINS->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXESFINS->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for ESF INS"));
      packetUBXESFINS->callbackPointer(*packetUBXESFINS->callbackData); // Call the callback
    }
    if (packetUBXESFINS->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for ESF INS"));
      packetUBXESFINS->callbackPointerPtr(packetUBXESFINS->callbackData); // Call the callback
    }
    packetUBXESFINS->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXESFMEAS != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXESFMEAS->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXESFMEAS->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for ESF MEAS"));
      packetUBXESFMEAS->callbackPointer(*packetUBXESFMEAS->callbackData); // Call the callback
    }
    if (packetUBXESFMEAS->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for ESF MEAS"));
      packetUBXESFMEAS->callbackPointerPtr(packetUBXESFMEAS->callbackData); // Call the callback
    }
    packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXESFRAW != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXESFRAW->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXESFRAW->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for ESF RAW"));
      packetUBXESFRAW->callbackPointer(*packetUBXESFRAW->callbackData); // Call the callback
    }
    if (packetUBXESFRAW->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for ESF RAW"));
      packetUBXESFRAW->callbackPointerPtr(packetUBXESFRAW->callbackData); // Call the callback
    }
    packetUBXESFRAW->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXESFSTATUS != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXESFSTATUS->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXESFSTATUS->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for ESF STATUS"));
      packetUBXESFSTATUS->callbackPointer(*packetUBXESFSTATUS->callbackData); // Call the callback
    }
    if (packetUBXESFSTATUS->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for ESF STATUS"));
      packetUBXESFSTATUS->callbackPointerPtr(packetUBXESFSTATUS->callbackData); // Call the callback
    }
    packetUBXESFSTATUS->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXHNRATT != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXHNRATT->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXHNRATT->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for HNR ATT"));
      packetUBXHNRATT->callbackPointer(*packetUBXHNRATT->callbackData); // Call the callback
    }
    if (packetUBXHNRATT->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for HNR ATT"));
      packetUBXHNRATT->callbackPointerPtr(packetUBXHNRATT->callbackData); // Call the callback
    }
    packetUBXHNRATT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXHNRINS != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXHNRINS->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXHNRINS->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for HNR INS"));
      packetUBXHNRINS->callbackPointer(*packetUBXHNRINS->callbackData); // Call the callback
    }
    if (packetUBXHNRINS->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for HNR INS"));
      packetUBXHNRINS->callbackPointerPtr(packetUBXHNRINS->callbackData); // Call the callback
    }
    packetUBXHNRINS->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

  if ((packetUBXHNRPVT != NULL)                                                  // If RAM has been allocated for message storage
      && (packetUBXHNRPVT->callbackData != NULL)                                 // If RAM has been allocated for the copy of the data
      && (packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid == true)) // If the copy of the data is valid
  {
    if (packetUBXHNRPVT->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for HNR PVT"));
      packetUBXHNRPVT->callbackPointer(*packetUBXHNRPVT->callbackData); // Call the callback
    }
    if (packetUBXHNRPVT->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for HNR PVT"));
      packetUBXHNRPVT->callbackPointerPtr(packetUBXHNRPVT->callbackData); // Call the callback
    }
    packetUBXHNRPVT->automaticFlags.flags.bits.callbackCopyValid = false; // Mark the data as stale
  }

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  if ((storageNMEAGPGGA != NULL)                                               // If RAM has been allocated for message storage
      && (storageNMEAGPGGA->callbackCopy != NULL)                              // If RAM has been allocated for the copy of the data
      && (storageNMEAGPGGA->automaticFlags.flags.bits.callbackCopyValid == 1)) // If the copy of the data is valid
  {
    if (storageNMEAGPGGA->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for GPGGA"));
      storageNMEAGPGGA->callbackPointer(*storageNMEAGPGGA->callbackCopy); // Call the callback
    }
    if (storageNMEAGPGGA->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for GPGGA"));
      storageNMEAGPGGA->callbackPointerPtr(storageNMEAGPGGA->callbackCopy); // Call the callback
    }
    storageNMEAGPGGA->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
  }

  if ((storageNMEAGNGGA != NULL)                                               // If RAM has been allocated for message storage
      && (storageNMEAGNGGA->callbackCopy != NULL)                              // If RAM has been allocated for the copy of the data
      && (storageNMEAGNGGA->automaticFlags.flags.bits.callbackCopyValid == 1)) // If the copy of the data is valid
  {
    if (storageNMEAGNGGA->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for GNGGA"));
      storageNMEAGNGGA->callbackPointer(*storageNMEAGNGGA->callbackCopy); // Call the callback
    }
    if (storageNMEAGNGGA->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for GNGGA"));
      storageNMEAGNGGA->callbackPointerPtr(storageNMEAGNGGA->callbackCopy); // Call the callback
    }
    storageNMEAGNGGA->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
  }

  if ((storageNMEAGPVTG != NULL)                                               // If RAM has been allocated for message storage
      && (storageNMEAGPVTG->callbackCopy != NULL)                              // If RAM has been allocated for the copy of the data
      && (storageNMEAGPVTG->automaticFlags.flags.bits.callbackCopyValid == 1)) // If the copy of the data is valid
  {
    if (storageNMEAGPVTG->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for GPVTG"));
      storageNMEAGPVTG->callbackPointer(*storageNMEAGPVTG->callbackCopy); // Call the callback
    }
    if (storageNMEAGPVTG->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for GPVTG"));
      storageNMEAGPVTG->callbackPointerPtr(storageNMEAGPVTG->callbackCopy); // Call the callback
    }
    storageNMEAGPVTG->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
  }

  if ((storageNMEAGNVTG != NULL)                                               // If RAM has been allocated for message storage
      && (storageNMEAGNVTG->callbackCopy != NULL)                              // If RAM has been allocated for the copy of the data
      && (storageNMEAGNVTG->automaticFlags.flags.bits.callbackCopyValid == 1)) // If the copy of the data is valid
  {
    if (storageNMEAGNVTG->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for GNVTG"));
      storageNMEAGNVTG->callbackPointer(*storageNMEAGNVTG->callbackCopy); // Call the callback
    }
    if (storageNMEAGNVTG->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for GNVTG"));
      storageNMEAGNVTG->callbackPointerPtr(storageNMEAGNVTG->callbackCopy); // Call the callback
    }
    storageNMEAGNVTG->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
  }

  if ((storageNMEAGPRMC != NULL)                                               // If RAM has been allocated for message storage
      && (storageNMEAGPRMC->callbackCopy != NULL)                              // If RAM has been allocated for the copy of the data
      && (storageNMEAGPRMC->automaticFlags.flags.bits.callbackCopyValid == 1)) // If the copy of the data is valid
  {
    if (storageNMEAGPRMC->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for GPRMC"));
      storageNMEAGPRMC->callbackPointer(*storageNMEAGPRMC->callbackCopy); // Call the callback
    }
    if (storageNMEAGPRMC->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for GPRMC"));
      storageNMEAGPRMC->callbackPointerPtr(storageNMEAGPRMC->callbackCopy); // Call the callback
    }
    storageNMEAGPRMC->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
  }

  if ((storageNMEAGNRMC != NULL)                                               // If RAM has been allocated for message storage
      && (storageNMEAGNRMC->callbackCopy != NULL)                              // If RAM has been allocated for the copy of the data
      && (storageNMEAGNRMC->automaticFlags.flags.bits.callbackCopyValid == 1)) // If the copy of the data is valid
  {
    if (storageNMEAGNRMC->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for GNRMC"));
      storageNMEAGNRMC->callbackPointer(*storageNMEAGNRMC->callbackCopy); // Call the callback
    }
    if (storageNMEAGNRMC->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for GNRMC"));
      storageNMEAGNRMC->callbackPointerPtr(storageNMEAGNRMC->callbackCopy); // Call the callback
    }
    storageNMEAGNRMC->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
  }

  if ((storageNMEAGPZDA != NULL)                                               // If RAM has been allocated for message storage
      && (storageNMEAGPZDA->callbackCopy != NULL)                              // If RAM has been allocated for the copy of the data
      && (storageNMEAGPZDA->automaticFlags.flags.bits.callbackCopyValid == 1)) // If the copy of the data is valid
  {
    if (storageNMEAGPZDA->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for GPZDA"));
      storageNMEAGPZDA->callbackPointer(*storageNMEAGPZDA->callbackCopy); // Call the callback
    }
    if (storageNMEAGPZDA->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for GPZDA"));
      storageNMEAGPZDA->callbackPointerPtr(storageNMEAGPZDA->callbackCopy); // Call the callback
    }
    storageNMEAGPZDA->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
  }

  if ((storageNMEAGNZDA != NULL)                                               // If RAM has been allocated for message storage
      && (storageNMEAGNZDA->callbackCopy != NULL)                              // If RAM has been allocated for the copy of the data
      && (storageNMEAGNZDA->automaticFlags.flags.bits.callbackCopyValid == 1)) // If the copy of the data is valid
  {
    if (storageNMEAGNZDA->callbackPointer != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callback for GNZDA"));
      storageNMEAGNZDA->callbackPointer(*storageNMEAGNZDA->callbackCopy); // Call the callback
    }
    if (storageNMEAGNZDA->callbackPointerPtr != NULL) // If the pointer to the callback has been defined
    {
      // if (_printDebug == true)
      //   _debugSerial->println(F("checkCallbacks: calling callbackPtr for GNZDA"));
      storageNMEAGNZDA->callbackPointerPtr(storageNMEAGNZDA->callbackCopy); // Call the callback
    }
    storageNMEAGNZDA->automaticFlags.flags.bits.callbackCopyValid = 0; // Mark the data as stale
  }
#endif

  checkCallbacksReentrant = false;
}

// Push (e.g.) RTCM data directly to the module
// Returns true if all numDataBytes were pushed successfully
// Warning: this function does not check that the data is valid. It is the user's responsibility to ensure the data is valid before pushing.
// Default to using a restart between transmissions. But processors like ESP32 seem to need a stop (#30). Set stop to true to use a stop instead.
// On processors like the ESP32, you can use setI2CTransactionSize to increase the size of each transmission - to e.g. 128 bytes
bool SFE_UBLOX_GNSS::pushRawData(uint8_t *dataBytes, size_t numDataBytes, bool stop)
{
  // Return now if numDataBytes is zero
  if (numDataBytes == 0)
    return (false); // Indicate to the user that there was no data to push

  if (commType == COMM_TYPE_SERIAL)
  {
    // Serial: write all the bytes in one go
    size_t bytesWritten = _serialPort->write(dataBytes, numDataBytes);
    return (bytesWritten == numDataBytes);
  }
  else if (commType == COMM_TYPE_I2C)
  {
    // We can not write a single data byte to I2C as it would look like the address of a random read.
    // If numDataBytes is 1, we should probably just reject the data and return false.
    // But we'll be nice and store the byte until the next time pushRawData is called.
    if ((numDataBytes == 1) && (_pushSingleByte == false))
    {
      _pushThisSingleByte = *dataBytes;
      _pushSingleByte = true;
      return (false); // Indicate to the user that their data has not been pushed yet
    }

    // If stop is true then always use a stop
    // Else if _i2cStopRestart is true then always use a stop
    // Else use a restart where needed
    if (stop == true)
      stop = true; // Redundant - but makes it clear what is happening
    else if (_i2cStopRestart == true)
      stop = true;
    else
      stop = false; // Use a restart

    // I2C: split the data up into packets of i2cTransactionSize
    size_t bytesLeftToWrite = numDataBytes;
    size_t bytesWrittenTotal = 0;

    if (_pushSingleByte == true) // Increment bytesLeftToWrite if we have a single byte waiting to be pushed
      bytesLeftToWrite++;

    while (bytesLeftToWrite > 0)
    {
      size_t bytesToWrite; // Limit bytesToWrite to i2cTransactionSize
      if (bytesLeftToWrite > i2cTransactionSize)
        bytesToWrite = i2cTransactionSize;
      else
        bytesToWrite = bytesLeftToWrite;

      // If there would be one byte left to be written next time, send one byte less now
      if ((bytesLeftToWrite - bytesToWrite) == 1)
        bytesToWrite--;

      _i2cPort->beginTransmission(_gpsI2Caddress);

      size_t bytesWritten = 0;

      // If _pushSingleByte is true, push it now
      if (_pushSingleByte == true)
      {
        bytesWritten += _i2cPort->write(_pushThisSingleByte);         // Write the single byte
        bytesWritten += _i2cPort->write(dataBytes, bytesToWrite - 1); // Write the bytes - but send one byte less
        dataBytes += bytesToWrite - 1;                                // Point to fresh data
        _pushSingleByte = false;                                      // Clear the flag
      }
      else
      {
        bytesWritten += _i2cPort->write(dataBytes, bytesToWrite); // Write the bytes
        dataBytes += bytesToWrite;                                // Point to fresh data
      }

      bytesWrittenTotal += bytesWritten; // Update the totals
      bytesLeftToWrite -= bytesToWrite;

      if (bytesLeftToWrite > 0)
      {
        if (_i2cPort->endTransmission(stop) != 0) // Send a restart or stop command
          return (false);                         // Sensor did not ACK
      }
      else
      {
        if (_i2cPort->endTransmission() != 0) // We're done. Release bus. Always use a stop here
          return (false);                     // Sensor did not ACK
      }
    }

    return (bytesWrittenTotal == numDataBytes); // Return true if the correct number of bytes were written
  }
  else // SPI
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("pushRawData: SPI not currently supported"));
    }
#endif
    return (false);
  }
}

// Push MGA AssistNow data to the module.
// Check for UBX-MGA-ACK responses if required (if mgaAck is YES or ENQUIRE).
// Wait for maxWait millis after sending each packet (if mgaAck is NO).
// Return how many bytes were pushed successfully.
// If skipTime is true, any UBX-MGA-INI-TIME_UTC or UBX-MGA-INI-TIME_GNSS packets found in the data will be skipped,
// allowing the user to override with their own time data with setUTCTimeAssistance.
size_t SFE_UBLOX_GNSS::pushAssistNowData(const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(0, false, (const uint8_t *)dataBytes.c_str(), numDataBytes, mgaAck, maxWait));
}
size_t SFE_UBLOX_GNSS::pushAssistNowData(const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(0, false, dataBytes, numDataBytes, mgaAck, maxWait));
}
size_t SFE_UBLOX_GNSS::pushAssistNowData(bool skipTime, const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(0, skipTime, (const uint8_t *)dataBytes.c_str(), numDataBytes, mgaAck, maxWait));
}
size_t SFE_UBLOX_GNSS::pushAssistNowData(bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(0, skipTime, dataBytes, numDataBytes, mgaAck, maxWait));
}
size_t SFE_UBLOX_GNSS::pushAssistNowData(size_t offset, bool skipTime, const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(offset, skipTime, (const uint8_t *)dataBytes.c_str(), numDataBytes, mgaAck, maxWait));
}
size_t SFE_UBLOX_GNSS::pushAssistNowData(size_t offset, bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  return (pushAssistNowDataInternal(offset, skipTime, dataBytes, numDataBytes, mgaAck, maxWait));
}
size_t SFE_UBLOX_GNSS::pushAssistNowDataInternal(size_t offset, bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  size_t dataPtr = offset;     // Pointer into dataBytes
  size_t packetsProcessed = 0; // Keep count of how many packets have been processed
  size_t bytesPushed = 0;      // Keep count

  bool checkForAcks = (mgaAck == SFE_UBLOX_MGA_ASSIST_ACK_YES); // If mgaAck is YES, always check for Acks

  // If mgaAck is ENQUIRE, we need to check UBX-CFG-NAVX5 ackAiding to determine if UBX-MGA-ACK's are expected
  if (mgaAck == SFE_UBLOX_MGA_ASSIST_ACK_ENQUIRE)
  {
    uint8_t ackAiding = getAckAiding(maxWait); // Enquire if we should expect Acks
    if (ackAiding == 1)
      checkForAcks = true;

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->print(F("pushAssistNowData: mgaAck is ENQUIRE. getAckAiding returned "));
      _debugSerial->println(ackAiding);
    }
#endif
  }

  // If checkForAcks is true, then we need to set up storage for the UBX-MGA-ACK-DATA0 messages
  if (checkForAcks)
  {
    if (packetUBXMGAACK == NULL)
      initPacketUBXMGAACK();     // Check that RAM has been allocated for the MGA_ACK data
    if (packetUBXMGAACK == NULL) // Bail if the RAM allocation failed
      return (0);
  }

  while (dataPtr < (offset + numDataBytes)) // Keep going until we have processed all the bytes
  {
    // Start by checking the validity of the packet being pointed to
    bool dataIsOK = true;

    dataIsOK &= (*(dataBytes + dataPtr + 0) == UBX_SYNCH_1);   // Check for 0xB5
    dataIsOK &= (*(dataBytes + dataPtr + 1) == UBX_SYNCH_2);   // Check for 0x62
    dataIsOK &= (*(dataBytes + dataPtr + 2) == UBX_CLASS_MGA); // Check for class UBX-MGA

    size_t packetLength = ((size_t) * (dataBytes + dataPtr + 4)) | (((size_t) * (dataBytes + dataPtr + 5)) << 8); // Extract the length

    uint8_t checksumA = 0;
    uint8_t checksumB = 0;
    // Calculate the checksum bytes
    // Keep going until the end of the packet is reached (payloadPtr == (dataPtr + packetLength))
    // or we reach the end of the AssistNow data (payloadPtr == offset + numDataBytes)
    for (size_t payloadPtr = dataPtr + ((size_t)2); (payloadPtr < (dataPtr + packetLength + ((size_t)6))) && (payloadPtr < (offset + numDataBytes)); payloadPtr++)
    {
      checksumA += *(dataBytes + payloadPtr);
      checksumB += checksumA;
    }
    // Check the checksum bytes
    dataIsOK &= (checksumA == *(dataBytes + dataPtr + packetLength + ((size_t)6)));
    dataIsOK &= (checksumB == *(dataBytes + dataPtr + packetLength + ((size_t)7)));

    dataIsOK &= ((dataPtr + packetLength + ((size_t)8)) <= (offset + numDataBytes)); // Check we haven't overrun

    // If the data is valid, push it
    if (dataIsOK)
    {
      // Check if this is time assistance data which should be skipped
      if ((skipTime) && ((*(dataBytes + dataPtr + 3) == UBX_MGA_INI_TIME_UTC) || (*(dataBytes + dataPtr + 3) == UBX_MGA_INI_TIME_GNSS)))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial->print(F("pushAssistNowData: skipped INI_TIME ID 0x"));
          if (*(dataBytes + dataPtr + 3) < 0x10)
            _debugSerial->print(F("0"));
          _debugSerial->println(*(dataBytes + dataPtr + 3), HEX);
        }
#endif
      }
      else
      {
        bool pushResult = pushRawData((uint8_t *)(dataBytes + dataPtr), packetLength + ((size_t)8)); // Push the data

        if (pushResult)
          bytesPushed += packetLength + ((size_t)8); // Increment bytesPushed if the push was successful

        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial->print(F("pushAssistNowData: packet ID 0x"));
          if (*(dataBytes + dataPtr + 3) < 0x10)
            _debugSerial->print(F("0"));
          _debugSerial->print(*(dataBytes + dataPtr + 3), HEX);
          _debugSerial->print(F(" length "));
          _debugSerial->println(packetLength);
        }

        if (checkForAcks)
        {
          unsigned long startTime = millis();
          bool keepGoing = true;
          while (keepGoing && (millis() < (startTime + maxWait))) // Keep checking for the ACK until we time out
          {
            checkUblox();
            if (packetUBXMGAACK->head != packetUBXMGAACK->tail) // Does the MGA ACK ringbuffer contain any ACK's?
            {
              bool dataAckd = true;                                                                                        // Check if we've received the correct ACK
              dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgId == *(dataBytes + dataPtr + 3));              // Check if the message ID matches
              dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[0] == *(dataBytes + dataPtr + 6)); // Check if the first four data bytes match
              dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[1] == *(dataBytes + dataPtr + 7));
              dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[2] == *(dataBytes + dataPtr + 8));
              dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[3] == *(dataBytes + dataPtr + 9));

              if (dataAckd) // Is this the ACK we are looking for?
              {
                if ((packetUBXMGAACK->data[packetUBXMGAACK->tail].type == (uint8_t)1) && (packetUBXMGAACK->data[packetUBXMGAACK->tail].infoCode == (uint8_t)SFE_UBLOX_MGA_ACK_INFOCODE_ACCEPTED))
                {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
                  if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
                  {
                    _debugSerial->print(F("pushAssistNowData: packet was accepted after "));
                    _debugSerial->print(millis() - startTime);
                    _debugSerial->println(F(" ms"));
                  }
#endif
                  packetsProcessed++;
                }
                else
                {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
                  if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
                  {
                    _debugSerial->print(F("pushAssistNowData: packet was _not_ accepted. infoCode is "));
                    _debugSerial->println(packetUBXMGAACK->data[packetUBXMGAACK->tail].infoCode);
                  }
#endif
                }
                keepGoing = false;
              }
              // Increment the tail
              packetUBXMGAACK->tail++;
              if (packetUBXMGAACK->tail == UBX_MGA_ACK_DATA0_RINGBUFFER_LEN)
                packetUBXMGAACK->tail = 0;
            }
          }
          if (keepGoing) // If keepGoing is still true, we must have timed out
          {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
            if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
            {
              _debugSerial->println(F("pushAssistNowData: packet ack timed out!"));
            }
#endif
          }
        }
        else
        {
          // We are not checking for Acks, so let's assume the send was successful?
          packetsProcessed++;
          // We are not checking for Acks, so delay for maxWait millis unless we've reached the end of the data
          if ((dataPtr + packetLength + ((size_t)8)) < (offset + numDataBytes))
          {
            delay(maxWait);
          }
        }
      }

      dataPtr += packetLength + ((size_t)8); // Point to the next message
    }
    else
    {

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      // The data was invalid. Send a debug message and then try to find the next 0xB5
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial->print(F("pushAssistNowData: bad data - ignored! dataPtr is "));
        _debugSerial->println(dataPtr);
      }
#endif

      while ((dataPtr < (offset + numDataBytes)) && (*(dataBytes + ++dataPtr) != UBX_SYNCH_1))
      {
        ; // Increment dataPtr until we are pointing at the next 0xB5 - or we reach the end of the data
      }
    }
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
  {
    _debugSerial->print(F("pushAssistNowData: packetsProcessed: "));
    _debugSerial->println(packetsProcessed);
  }
#endif

  return (bytesPushed); // Return the number of valid bytes successfully pushed
}

// PRIVATE: Allocate RAM for packetUBXMGAACK and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXMGAACK()
{
  packetUBXMGAACK = new UBX_MGA_ACK_DATA0_t; // Allocate RAM for the main struct
  if (packetUBXMGAACK == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXMGAACK: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXMGAACK->head = 0; // Initialize the ring buffer pointers
  packetUBXMGAACK->tail = 0;
  return (true);
}

// Provide initial time assistance
bool SFE_UBLOX_GNSS::setUTCTimeAssistance(uint16_t year, uint8_t month, uint8_t day,
                                          uint8_t hour, uint8_t minute, uint8_t second, uint32_t nanos,
                                          uint16_t tAccS, uint32_t tAccNs, uint8_t source,
                                          sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  uint8_t iniTimeUTC[32];       // Create the UBX-MGA-INI-TIME_UTC message by hand
  memset(iniTimeUTC, 0x00, 32); // Set all unused / reserved bytes and the checksum to zero

  iniTimeUTC[0] = UBX_SYNCH_1;              // Sync char 1
  iniTimeUTC[1] = UBX_SYNCH_2;              // Sync char 2
  iniTimeUTC[2] = UBX_CLASS_MGA;            // Class
  iniTimeUTC[3] = UBX_MGA_INI_TIME_UTC;     // ID
  iniTimeUTC[4] = 24;                       // Length LSB
  iniTimeUTC[5] = 0x00;                     // Length MSB
  iniTimeUTC[6] = 0x10;                     // type
  iniTimeUTC[7] = 0x00;                     // version
  iniTimeUTC[8] = source;                   // ref (source)
  iniTimeUTC[9] = 0x80;                     // leapSecs. Set to 0x80 = unknown
  iniTimeUTC[10] = (uint8_t)(year & 0xFF);  // year LSB
  iniTimeUTC[11] = (uint8_t)(year >> 8);    // year MSB
  iniTimeUTC[12] = month;                   // month starting at 1
  iniTimeUTC[13] = day;                     // day starting at 1
  iniTimeUTC[14] = hour;                    // hour 0:23
  iniTimeUTC[15] = minute;                  // minute 0:59
  iniTimeUTC[16] = second;                  // seconds 0:59
  iniTimeUTC[18] = (uint8_t)(nanos & 0xFF); // nanoseconds LSB
  iniTimeUTC[19] = (uint8_t)((nanos >> 8) & 0xFF);
  iniTimeUTC[20] = (uint8_t)((nanos >> 16) & 0xFF);
  iniTimeUTC[21] = (uint8_t)(nanos >> 24);   // nanoseconds MSB
  iniTimeUTC[22] = (uint8_t)(tAccS & 0xFF);  // seconds part of the accuracy LSB
  iniTimeUTC[23] = (uint8_t)(tAccS >> 8);    // seconds part of the accuracy MSB
  iniTimeUTC[26] = (uint8_t)(tAccNs & 0xFF); // nanoseconds part of the accuracy LSB
  iniTimeUTC[27] = (uint8_t)((tAccNs >> 8) & 0xFF);
  iniTimeUTC[28] = (uint8_t)((tAccNs >> 16) & 0xFF);
  iniTimeUTC[29] = (uint8_t)(tAccNs >> 24); // nanoseconds part of the accuracy MSB

  for (uint8_t i = 2; i < 30; i++) // Calculate the checksum
  {
    iniTimeUTC[30] += iniTimeUTC[i];
    iniTimeUTC[31] += iniTimeUTC[30];
  }

  // Return true if the one packet was pushed successfully
  return (pushAssistNowDataInternal(0, false, iniTimeUTC, 32, mgaAck, maxWait) == 32);
}

// Provide initial position assistance
// The units for ecefX/Y/Z and posAcc (stddev) are cm.
bool SFE_UBLOX_GNSS::setPositionAssistanceXYZ(int32_t ecefX, int32_t ecefY, int32_t ecefZ, uint32_t posAcc, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  uint8_t iniPosXYZ[28];       // Create the UBX-MGA-INI-POS_XYZ message by hand
  memset(iniPosXYZ, 0x00, 28); // Set all unused / reserved bytes and the checksum to zero

  iniPosXYZ[0] = UBX_SYNCH_1;         // Sync char 1
  iniPosXYZ[1] = UBX_SYNCH_2;         // Sync char 2
  iniPosXYZ[2] = UBX_CLASS_MGA;       // Class
  iniPosXYZ[3] = UBX_MGA_INI_POS_XYZ; // ID
  iniPosXYZ[4] = 20;                  // Length LSB
  iniPosXYZ[5] = 0x00;                // Length MSB
  iniPosXYZ[6] = 0x00;                // type
  iniPosXYZ[7] = 0x00;                // version

  union // Use a union to convert from int32_t to uint32_t
  {
    int32_t signedLong;
    uint32_t unsignedLong;
  } signedUnsigned;

  signedUnsigned.signedLong = ecefX;
  iniPosXYZ[10] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosXYZ[11] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosXYZ[12] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosXYZ[13] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  signedUnsigned.signedLong = ecefY;
  iniPosXYZ[14] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosXYZ[15] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosXYZ[16] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosXYZ[17] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  signedUnsigned.signedLong = ecefZ;
  iniPosXYZ[18] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosXYZ[19] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosXYZ[20] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosXYZ[21] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  iniPosXYZ[22] = (uint8_t)(posAcc & 0xFF); // LSB
  iniPosXYZ[23] = (uint8_t)((posAcc >> 8) & 0xFF);
  iniPosXYZ[24] = (uint8_t)((posAcc >> 16) & 0xFF);
  iniPosXYZ[25] = (uint8_t)(posAcc >> 24); // MSB

  for (uint8_t i = 2; i < 26; i++) // Calculate the checksum
  {
    iniPosXYZ[26] += iniPosXYZ[i];
    iniPosXYZ[27] += iniPosXYZ[26];
  }

  // Return true if the one packet was pushed successfully
  return (pushAssistNowDataInternal(0, false, iniPosXYZ, 28, mgaAck, maxWait) == 28);
}

// The units for lat and lon are degrees * 1e-7 (WGS84)
// The units for alt (WGS84) and posAcc (stddev) are cm.
bool SFE_UBLOX_GNSS::setPositionAssistanceLLH(int32_t lat, int32_t lon, int32_t alt, uint32_t posAcc, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait)
{
  uint8_t iniPosLLH[28];       // Create the UBX-MGA-INI-POS_LLH message by hand
  memset(iniPosLLH, 0x00, 28); // Set all unused / reserved bytes and the checksum to zero

  iniPosLLH[0] = UBX_SYNCH_1;         // Sync char 1
  iniPosLLH[1] = UBX_SYNCH_2;         // Sync char 2
  iniPosLLH[2] = UBX_CLASS_MGA;       // Class
  iniPosLLH[3] = UBX_MGA_INI_POS_LLH; // ID
  iniPosLLH[4] = 20;                  // Length LSB
  iniPosLLH[5] = 0x00;                // Length MSB
  iniPosLLH[6] = 0x01;                // type
  iniPosLLH[7] = 0x00;                // version

  union // Use a union to convert from int32_t to uint32_t
  {
    int32_t signedLong;
    uint32_t unsignedLong;
  } signedUnsigned;

  signedUnsigned.signedLong = lat;
  iniPosLLH[10] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosLLH[11] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosLLH[12] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosLLH[13] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  signedUnsigned.signedLong = lon;
  iniPosLLH[14] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosLLH[15] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosLLH[16] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosLLH[17] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  signedUnsigned.signedLong = alt;
  iniPosLLH[18] = (uint8_t)(signedUnsigned.unsignedLong & 0xFF); // LSB
  iniPosLLH[19] = (uint8_t)((signedUnsigned.unsignedLong >> 8) & 0xFF);
  iniPosLLH[20] = (uint8_t)((signedUnsigned.unsignedLong >> 16) & 0xFF);
  iniPosLLH[21] = (uint8_t)(signedUnsigned.unsignedLong >> 24); // MSB

  iniPosLLH[22] = (uint8_t)(posAcc & 0xFF); // LSB
  iniPosLLH[23] = (uint8_t)((posAcc >> 8) & 0xFF);
  iniPosLLH[24] = (uint8_t)((posAcc >> 16) & 0xFF);
  iniPosLLH[25] = (uint8_t)(posAcc >> 24); // MSB

  for (uint8_t i = 2; i < 26; i++) // Calculate the checksum
  {
    iniPosLLH[26] += iniPosLLH[i];
    iniPosLLH[27] += iniPosLLH[26];
  }

  // Return true if the one packet was pushed successfully
  return (pushAssistNowDataInternal(0, false, iniPosLLH, 28, mgaAck, maxWait) == 28);
}

// Find the start of the AssistNow Offline (UBX_MGA_ANO) data for the chosen day
// The daysIntoFture parameter makes it easy to get the data for (e.g.) tomorrow based on today's date
// Returns numDataBytes if unsuccessful
// TO DO: enhance this so it will find the nearest data for the chosen day - instead of an exact match
size_t SFE_UBLOX_GNSS::findMGAANOForDate(const String &dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture)
{
  return (findMGAANOForDateInternal((const uint8_t *)dataBytes.c_str(), numDataBytes, year, month, day, daysIntoFuture));
}
size_t SFE_UBLOX_GNSS::findMGAANOForDate(const uint8_t *dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture)
{
  return (findMGAANOForDateInternal(dataBytes, numDataBytes, year, month, day, daysIntoFuture));
}
size_t SFE_UBLOX_GNSS::findMGAANOForDateInternal(const uint8_t *dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture)
{
  size_t dataPtr = 0;     // Pointer into dataBytes
  bool dateFound = false; // Flag to indicate when the date has been found

  // Calculate matchDay, matchMonth and matchYear
  uint8_t matchDay = day;
  uint8_t matchMonth = month;
  uint8_t matchYear = (uint8_t)(year - 2000);

  // Add on daysIntoFuture
  uint8_t daysIntoFutureCopy = daysIntoFuture;
  while (daysIntoFutureCopy > 0)
  {
    matchDay++;
    daysIntoFutureCopy--;
    switch (matchMonth)
    {
    case 1:
    case 3:
    case 5:
    case 7:
    case 8:
    case 10:
    case 12:
      if (matchDay == 32)
      {
        matchDay = 1;
        matchMonth++;
        if (matchMonth == 13)
        {
          matchMonth = 1;
          matchYear++;
        }
      }
      break;
    case 4:
    case 6:
    case 9:
    case 11:
      if (matchDay == 31)
      {
        matchDay = 1;
        matchMonth++;
      }
      break;
    default: // February
      if (((matchYear % 4) == 0) && (matchDay == 30))
      {
        matchDay = 1;
        matchMonth++;
      }
      else if (((matchYear % 4) > 0) && (matchDay == 29))
      {
        matchDay = 1;
        matchMonth++;
      }
      break;
    }
  }

  while ((!dateFound) && (dataPtr < numDataBytes)) // Keep going until we have found the date or processed all the bytes
  {
    // Start by checking the validity of the packet being pointed to
    bool dataIsOK = true;

    dataIsOK &= (*(dataBytes + dataPtr + 0) == UBX_SYNCH_1);   // Check for 0xB5
    dataIsOK &= (*(dataBytes + dataPtr + 1) == UBX_SYNCH_2);   // Check for 0x62
    dataIsOK &= (*(dataBytes + dataPtr + 2) == UBX_CLASS_MGA); // Check for class UBX-MGA

    size_t packetLength = ((size_t) * (dataBytes + dataPtr + 4)) | (((size_t) * (dataBytes + dataPtr + 5)) << 8); // Extract the length

    uint8_t checksumA = 0;
    uint8_t checksumB = 0;
    // Calculate the checksum bytes
    // Keep going until the end of the packet is reached (payloadPtr == (dataPtr + packetLength))
    // or we reach the end of the AssistNow data (payloadPtr == numDataBytes)
    for (size_t payloadPtr = dataPtr + ((size_t)2); (payloadPtr < (dataPtr + packetLength + ((size_t)6))) && (payloadPtr < numDataBytes); payloadPtr++)
    {
      checksumA += *(dataBytes + payloadPtr);
      checksumB += checksumA;
    }
    // Check the checksum bytes
    dataIsOK &= (checksumA == *(dataBytes + dataPtr + packetLength + ((size_t)6)));
    dataIsOK &= (checksumB == *(dataBytes + dataPtr + packetLength + ((size_t)7)));

    dataIsOK &= ((dataPtr + packetLength + ((size_t)8)) <= numDataBytes); // Check we haven't overrun

    // If the data is valid, check for a date match
    if (dataIsOK)
    {
      if ((*(dataBytes + dataPtr + 3) == UBX_MGA_ANO) && (*(dataBytes + dataPtr + 10) == matchYear) && (*(dataBytes + dataPtr + 11) == matchMonth) && (*(dataBytes + dataPtr + 12) == matchDay))
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial->print(F("findMGAANOForDate: found date match at location "));
          _debugSerial->println(dataPtr);
        }
#endif
        dateFound = true;
      }
      else
      {
        // The data is valid, but these are not the droids we are looking for...
        dataPtr += packetLength + ((size_t)8); // Point to the next message
      }
    }
    else
    {

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      // The data was invalid. Send a debug message and then try to find the next 0xB5
      if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      {
        _debugSerial->print(F("findMGAANOForDate: bad data - ignored! dataPtr is "));
        _debugSerial->println(dataPtr);
      }
#endif

      while ((dataPtr < numDataBytes) && (*(dataBytes + ++dataPtr) != UBX_SYNCH_1))
      {
        ; // Increment dataPtr until we are pointing at the next 0xB5 - or we reach the end of the data
      }
    }
  }

  return (dataPtr);
}

// Read the whole navigation data base. The receiver will send all available data from its internal database.
// Data is written to dataBytes. Set maxNumDataBytes to the (maximum) size of dataBytes.
// If the database exceeds maxNumDataBytes, the excess bytes will be lost.
// The function returns the number of database bytes written to dataBytes.
// The return value will be equal to maxNumDataBytes if excess data was received.
// The function will timeout after maxWait milliseconds - in case the final UBX-MGA-ACK was missed.
size_t SFE_UBLOX_GNSS::readNavigationDatabase(uint8_t *dataBytes, size_t maxNumDataBytes, uint16_t maxWait)
{
  // Allocate RAM to store the MGA ACK message
  if (packetUBXMGAACK == NULL)
    initPacketUBXMGAACK();     // Check that RAM has been allocated for the MGA_ACK data
  if (packetUBXMGAACK == NULL) // Bail if the RAM allocation failed
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("readNavigationDatabase: packetUBXMGAACK RAM allocation failed!"));
    }
#endif
    return ((size_t)0);
  }
  if (packetUBXMGAACK->head != packetUBXMGAACK->tail) // Does the MGA ACK ringbuffer contain any data?
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("readNavigationDatabase: packetUBXMGAACK contains unprocessed data. Clearing it."));
    }
#endif
    packetUBXMGAACK->tail = packetUBXMGAACK->head; // Clear the buffer by setting the tail equal to the head
  }

  // Allocate RAM to store the MGA DBD messages
  if (packetUBXMGADBD == NULL)
    initPacketUBXMGADBD();     // Check that RAM has been allocated for the MGA_DBD data
  if (packetUBXMGADBD == NULL) // Bail if the RAM allocation failed
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("readNavigationDatabase: packetUBXMGADBD RAM allocation failed!"));
    }
#endif
    return ((size_t)0);
  }
  if (packetUBXMGADBD->head != packetUBXMGADBD->tail) // Does the MGA DBD ringbuffer contain any data?
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("readNavigationDatabase: packetUBXMGADBD contains unprocessed data. Clearing it."));
    }
#endif
    packetUBXMGADBD->tail = packetUBXMGADBD->head; // Clear the buffer by setting the tail equal to the head
  }

  // Record what ackAiding is currently set to so we can restore it
  uint8_t currentAckAiding = getAckAiding();
  if (currentAckAiding == 255)
    currentAckAiding = 0; // If the get failed, disable the ACKs when returning
  // Enable ackAiding
  setAckAiding(1);

  // Record what i2cPollingWait is currently set to so we can restore it
  uint8_t currentI2cPollingWait = i2cPollingWait;
  // Set the I2C polling wait to 1ms
  i2cPollingWait = 1;

  // Construct the poll message:
  uint8_t pollNaviDatabase[8];       // Create the UBX-MGA-DBD message by hand
  memset(pollNaviDatabase, 0x00, 8); // Set all unused / reserved bytes and the checksum to zero

  pollNaviDatabase[0] = UBX_SYNCH_1;   // Sync char 1
  pollNaviDatabase[1] = UBX_SYNCH_2;   // Sync char 2
  pollNaviDatabase[2] = UBX_CLASS_MGA; // Class
  pollNaviDatabase[3] = UBX_MGA_DBD;   // ID
  pollNaviDatabase[4] = 0x00;          // Length LSB
  pollNaviDatabase[5] = 0x00;          // Length MSB

  for (uint8_t i = 2; i < 6; i++) // Calculate the checksum
  {
    pollNaviDatabase[6] += pollNaviDatabase[i];
    pollNaviDatabase[7] += pollNaviDatabase[6];
  }

  // Push the poll message to the module.
  // Do not Wait for an ACK - the DBD data will start arriving immediately.
  size_t pushResult = pushAssistNowDataInternal(0, false, pollNaviDatabase, (size_t)8, SFE_UBLOX_MGA_ASSIST_ACK_NO, 0);

  // Check pushResult == 8
  if (pushResult != 8)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("readNavigationDatabase: pushAssistNowDataInternal failed!"));
    }
#endif
    i2cPollingWait = currentI2cPollingWait; // Restore i2cPollingWait
    setAckAiding(currentAckAiding);         // Restore Ack Aiding
    return ((size_t)0);
  }

  // Now keep checking for the arrival of UBX-MGA-DBD packets and write them to dataBytes
  bool keepGoing = true;
  unsigned long startTime = millis();
  uint32_t databaseEntriesRX = 0; // Keep track of how many database entries are received
  size_t numBytesReceived = 0;    // Keep track of how many bytes are received

  while (keepGoing && (millis() < (startTime + maxWait)))
  {
    checkUblox();

    while (packetUBXMGADBD->head != packetUBXMGADBD->tail) // Does the MGA DBD ringbuffer contain any data?
    {
      // The data will be valid - process will have already checked it. So we can simply copy the data into dataBuffer.
      // We do not need to check if there is room to store the entire database entry. pushAssistNowData will check the data before pushing it.
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryHeader1;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryHeader2;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryClass;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryID;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryLenLSB;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryLenMSB;
      size_t msgLen = (((size_t)packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryLenMSB) * 256) + ((size_t)packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryLenLSB);
      for (size_t i = 0; i < msgLen; i++)
      {
        if (numBytesReceived < maxNumDataBytes)
          *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntry[i];
      }
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryChecksumA;
      if (numBytesReceived < maxNumDataBytes)
        *(dataBytes + (numBytesReceived++)) = packetUBXMGADBD->data[packetUBXMGADBD->tail].dbdEntryChecksumB;

      // Increment the tail
      packetUBXMGADBD->tail++;
      if (packetUBXMGADBD->tail == UBX_MGA_DBD_RINGBUFFER_LEN)
        packetUBXMGADBD->tail = 0;

      databaseEntriesRX++; // Increment the number of entries received
    }

    // The final MGA-ACK is sent at the end of the DBD packets. So, we need to check the ACK buffer _after_ the DBD buffer.
    while (packetUBXMGAACK->head != packetUBXMGAACK->tail) // Does the MGA ACK ringbuffer contain any data?
    {
      // Check if we've received the correct ACK
      bool idMatch = (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgId == UBX_MGA_DBD); // Check if the message ID matches

      bool dataAckd = true;
      dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[0] == (uint8_t)(databaseEntriesRX & 0xFF)); // Check if the ACK contents match databaseEntriesRX
      dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[1] == (uint8_t)((databaseEntriesRX >> 8) & 0xFF));
      dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[2] == (uint8_t)((databaseEntriesRX >> 16) & 0xFF));
      dataAckd &= (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[3] == (uint8_t)((databaseEntriesRX >> 24) & 0xFF));

      if (idMatch && dataAckd) // Is the ACK valid?
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial->print(F("readNavigationDatabase: ACK received. databaseEntriesRX is "));
          _debugSerial->print(databaseEntriesRX);
          _debugSerial->print(F(". numBytesReceived is "));
          _debugSerial->print(numBytesReceived);
          _debugSerial->print(F(". DBD read complete after "));
          _debugSerial->print(millis() - startTime);
          _debugSerial->println(F(" ms"));
        }
#endif
        keepGoing = false;
      }
      else if (idMatch)
      {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
        if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
        {
          _debugSerial->print(F("readNavigationDatabase: unexpected ACK received. databaseEntriesRX is 0x"));
          _debugSerial->print(databaseEntriesRX, HEX);
          _debugSerial->print(F(". msgPayloadStart is 0x"));
          for (uint8_t i = 4; i > 0; i--)
          {
            if (packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[i - 1] < 0x10)
              _debugSerial->print(F("0"));
            _debugSerial->print(packetUBXMGAACK->data[packetUBXMGAACK->tail].msgPayloadStart[i - 1], HEX);
          }
          _debugSerial->println();
        }
#endif
      }

      // Increment the tail
      packetUBXMGAACK->tail++;
      if (packetUBXMGAACK->tail == UBX_MGA_ACK_DATA0_RINGBUFFER_LEN)
        packetUBXMGAACK->tail = 0;
    }
  }

  if (keepGoing) // If keepGoing is still true, we must have timed out
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("readNavigationDatabase: DBD RX timed out!"));
    }
#endif
  }

  i2cPollingWait = currentI2cPollingWait; // Restore i2cPollingWait
  setAckAiding(currentAckAiding);         // Restore Ack Aiding

  return (numBytesReceived);
}

// PRIVATE: Allocate RAM for packetUBXMGADBD and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXMGADBD()
{
  packetUBXMGADBD = new UBX_MGA_DBD_t; // Allocate RAM for the main struct
  if (packetUBXMGADBD == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXMGADBD: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXMGADBD->head = 0; // Initialize the ring buffer pointers
  packetUBXMGADBD->tail = 0;
  return (true);
}

// Support for data logging

// Set the file buffer size. This must be called _before_ .begin
void SFE_UBLOX_GNSS::setFileBufferSize(uint16_t bufferSize)
{
  fileBufferSize = bufferSize;
}

// Return the file buffer size
uint16_t SFE_UBLOX_GNSS::getFileBufferSize(void)
{
  return (fileBufferSize);
}

// Extract numBytes of data from the file buffer. Copy it to destination.
// It is the user's responsibility to ensure destination is large enough.
// Returns the number of bytes extracted - which may be less than numBytes.
uint16_t SFE_UBLOX_GNSS::extractFileBufferData(uint8_t *destination, uint16_t numBytes)
{
  // Check how many bytes are available in the buffer
  uint16_t bytesAvailable = fileBufferSpaceUsed();
  if (numBytes > bytesAvailable) // Limit numBytes if required
    numBytes = bytesAvailable;

  // Start copying at fileBufferTail. Wrap-around if required.
  uint16_t bytesBeforeWrapAround = fileBufferSize - fileBufferTail; // How much space is available 'above' Tail?
  if (bytesBeforeWrapAround > numBytes)                             // Will we need to wrap-around?
  {
    bytesBeforeWrapAround = numBytes; // We need to wrap-around
  }
  memcpy(destination, &ubxFileBuffer[fileBufferTail], bytesBeforeWrapAround); // Copy the data out of the buffer

  // Is there any data leftover which we need to copy from the 'bottom' of the buffer?
  uint16_t bytesLeftToCopy = numBytes - bytesBeforeWrapAround; // Calculate if there are any bytes left to copy
  if (bytesLeftToCopy > 0)                                     // If there are bytes left to copy
  {
    memcpy(&destination[bytesBeforeWrapAround], &ubxFileBuffer[0], bytesLeftToCopy); // Copy the remaining data out of the buffer
    fileBufferTail = bytesLeftToCopy;                                                // Update Tail. The next byte to be read will be read from here.
  }
  else
  {
    fileBufferTail += numBytes; // Only update Tail. The next byte to be read will be read from here.
  }

  return (numBytes); // Return the number of bytes extracted
}

// Returns the number of bytes available in file buffer which are waiting to be read
uint16_t SFE_UBLOX_GNSS::fileBufferAvailable(void)
{
  return (fileBufferSpaceUsed());
}

// Returns the maximum number of bytes which the file buffer contained.
// Handy for checking the buffer is large enough to handle all the incoming data.
uint16_t SFE_UBLOX_GNSS::getMaxFileBufferAvail(void)
{
  return (fileBufferMaxAvail);
}

// Clear the file buffer - discard all contents
void SFE_UBLOX_GNSS::clearFileBuffer(void)
{
  if (fileBufferSize == 0) // Bail if the user has not called setFileBufferSize (probably redundant)
    return;
  fileBufferTail = fileBufferHead;
}

// Reset fileBufferMaxAvail
void SFE_UBLOX_GNSS::clearMaxFileBufferAvail(void)
{
  fileBufferMaxAvail = 0;
}

// PRIVATE: Create the file buffer. Called by .begin
bool SFE_UBLOX_GNSS::createFileBuffer(void)
{
  if (fileBufferSize == 0) // Bail if the user has not called setFileBufferSize
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("createFileBuffer: Warning. fileBufferSize is zero. Data logging is not possible."));
    }
#endif
    return (false);
  }

  if (ubxFileBuffer != NULL) // Bail if RAM has already been allocated for the file buffer
  {                          // This will happen if you call .begin more than once - without calling .end first
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("createFileBuffer: Warning. File buffer already exists. Skipping..."));
    }
#endif
    return (false);
  }

  ubxFileBuffer = new uint8_t[fileBufferSize]; // Allocate RAM for the buffer

  if (ubxFileBuffer == NULL) // Check if the new (alloc) was successful
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("createFileBuffer: RAM alloc failed!"));
    }
    return (false);
  }

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("createFileBuffer: fileBufferSize is: "));
    _debugSerial->println(fileBufferSize);
  }
#endif

  fileBufferHead = 0; // Initialize head and tail
  fileBufferTail = 0;

  return (true);
}

// PRIVATE: Check how much space is available in the buffer
uint16_t SFE_UBLOX_GNSS::fileBufferSpaceAvailable(void)
{
  return (fileBufferSize - fileBufferSpaceUsed());
}

// PRIVATE: Check how much space is used in the buffer
uint16_t SFE_UBLOX_GNSS::fileBufferSpaceUsed(void)
{
  if (fileBufferHead >= fileBufferTail) // Check if wrap-around has occurred
  {
    // Wrap-around has not occurred so do a simple subtraction
    return (fileBufferHead - fileBufferTail);
  }
  else
  {
    // Wrap-around has occurred so do a simple subtraction but add in the fileBufferSize
    return ((uint16_t)(((uint32_t)fileBufferHead + (uint32_t)fileBufferSize) - (uint32_t)fileBufferTail));
  }
}

// PRIVATE: Add a UBX packet to the file buffer
bool SFE_UBLOX_GNSS::storePacket(ubxPacket *msg)
{
  // First, check that the file buffer has been created
  if ((ubxFileBuffer == NULL) || (fileBufferSize == 0))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("storePacket: file buffer not available!"));
    }
#endif
    return (false);
  }

  // Now, check if there is enough space in the buffer for all of the data
  uint16_t totalLength = msg->len + 8; // Total length. Include sync chars, class, id, length and checksum bytes
  if (totalLength > fileBufferSpaceAvailable())
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("storePacket: insufficient space available! Data will be lost!"));
    }
#endif
    return (false);
  }

  // Store the two sync chars
  uint8_t sync_chars[] = {UBX_SYNCH_1, UBX_SYNCH_2};
  writeToFileBuffer(sync_chars, 2);

  // Store the Class & ID
  writeToFileBuffer(&msg->cls, 1);
  writeToFileBuffer(&msg->id, 1);

  // Store the length. Ensure length is little-endian
  uint8_t msg_length[2];
  msg_length[0] = msg->len & 0xFF;
  msg_length[1] = msg->len >> 8;
  writeToFileBuffer(msg_length, 2);

  // Store the payload
  writeToFileBuffer(msg->payload, msg->len);

  // Store the checksum
  writeToFileBuffer(&msg->checksumA, 1);
  writeToFileBuffer(&msg->checksumB, 1);

  return (true);
}

// PRIVATE: Add theBytes to the file buffer
bool SFE_UBLOX_GNSS::storeFileBytes(uint8_t *theBytes, uint16_t numBytes)
{
  // First, check that the file buffer has been created
  if ((ubxFileBuffer == NULL) || (fileBufferSize == 0))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("storeFileBytes: file buffer not available!"));
    }
#endif
    return (false);
  }

  // Now, check if there is enough space in the buffer for all of the data
  if (numBytes > fileBufferSpaceAvailable())
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("storeFileBytes: insufficient space available! Data will be lost!"));
    }
#endif
    return (false);
  }

  // There is room for all the data in the buffer so copy the data into the buffer
  writeToFileBuffer(theBytes, numBytes);

  return (true);
}

// PRIVATE: Write theBytes to the file buffer
void SFE_UBLOX_GNSS::writeToFileBuffer(uint8_t *theBytes, uint16_t numBytes)
{
  // Start writing at fileBufferHead. Wrap-around if required.
  uint16_t bytesBeforeWrapAround = fileBufferSize - fileBufferHead; // How much space is available 'above' Head?
  if (bytesBeforeWrapAround > numBytes)                             // Is there enough room for all the data?
  {
    bytesBeforeWrapAround = numBytes; // There is enough room for all the data
  }
  memcpy(&ubxFileBuffer[fileBufferHead], theBytes, bytesBeforeWrapAround); // Copy the data into the buffer

  // Is there any data leftover which we need to copy to the 'bottom' of the buffer?
  uint16_t bytesLeftToCopy = numBytes - bytesBeforeWrapAround; // Calculate if there are any bytes left to copy
  if (bytesLeftToCopy > 0)                                     // If there are bytes left to copy
  {
    memcpy(&ubxFileBuffer[0], &theBytes[bytesBeforeWrapAround], bytesLeftToCopy); // Copy the remaining data into the buffer
    fileBufferHead = bytesLeftToCopy;                                             // Update Head. The next byte written will be written here.
  }
  else
  {
    fileBufferHead += numBytes; // Only update Head. The next byte written will be written here.
  }

  // Update fileBufferMaxAvail if required
  uint16_t bytesInBuffer = fileBufferSpaceUsed();
  if (bytesInBuffer > fileBufferMaxAvail)
    fileBufferMaxAvail = bytesInBuffer;
}

//=-=-=-=-=-=-=-= Specific commands =-=-=-=-=-=-=-==-=-=-=-=-=-=-=
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Loads the payloadCfg array with the current protocol bits located the UBX-CFG-PRT register for a given port
bool SFE_UBLOX_GNSS::getPortSettings(uint8_t portID, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 1;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = portID;

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_RECEIVED); // We are expecting data and an ACK
}

// Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
// Port 0=I2c, 1=UART1, 2=UART2, 3=USB, 4=SPI
// Bit:0 = UBX, :1=NMEA, :5=RTCM3
bool SFE_UBLOX_GNSS::setPortOutput(uint8_t portID, uint8_t outStreamSettings, uint16_t maxWait)
{
  // Get the current config values for this port ID
  if (getPortSettings(portID, maxWait) == false)
    return (false); // Something went wrong. Bail.

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  // payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[14] = outStreamSettings; // OutProtocolMask LSB - Set outStream bits

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Configure a given port to input UBX, NMEA, RTCM3 or a combination thereof
// Port 0=I2c, 1=UART1, 2=UART2, 3=USB, 4=SPI
// Bit:0 = UBX, :1=NMEA, :5=RTCM3
bool SFE_UBLOX_GNSS::setPortInput(uint8_t portID, uint8_t inStreamSettings, uint16_t maxWait)
{
  // Get the current config values for this port ID
  // This will load the payloadCfg array with current port settings
  if (getPortSettings(portID, maxWait) == false)
    return (false); // Something went wrong. Bail.

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  // payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[12] = inStreamSettings; // InProtocolMask LSB - Set inStream bits

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Changes the I2C address that the u-blox module responds to
// 0x42 is the default but can be changed with this command
bool SFE_UBLOX_GNSS::setI2CAddress(uint8_t deviceAddress, uint16_t maxWait)
{
  // Get the current config values for the I2C port
  // This will load the payloadCfg array with current port settings
  if (getPortSettings(COM_PORT_I2C, maxWait) == false)
    return (false); // Something went wrong. Bail.

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  // payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[4] = deviceAddress << 1; // DDC mode LSB

  if (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT) // We are only expecting an ACK
  {
    // Success! Now change our internal global.
    _gpsI2Caddress = deviceAddress; // Store the I2C address from user
    return (true);
  }
  return (false);
}

// Changes the serial baud rate of the u-blox module, can't return success/fail 'cause ACK from modem
// is lost due to baud rate change
void SFE_UBLOX_GNSS::setSerialRate(uint32_t baudrate, uint8_t uartPort, uint16_t maxWait)
{
  // Get the current config values for the UART port
  // This will load the payloadCfg array with current port settings
  if (getPortSettings(uartPort, maxWait) == false)
    return; // Something went wrong. Bail.

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("Current baud rate: "));
    _debugSerial->println(((uint32_t)payloadCfg[10] << 16) | ((uint32_t)payloadCfg[9] << 8) | payloadCfg[8]);
  }
#endif

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  // payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[8] = baudrate;
  payloadCfg[9] = baudrate >> 8;
  payloadCfg[10] = baudrate >> 16;
  payloadCfg[11] = baudrate >> 24;

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("New baud rate:"));
    _debugSerial->println(((uint32_t)payloadCfg[10] << 16) | ((uint32_t)payloadCfg[9] << 8) | payloadCfg[8]);
  }
#endif

  sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("setSerialRate: sendCommand returned: "));
    _debugSerial->println(statusString(retVal));
  }
#else
  (void)retVal; // Get rid of a pesky compiler warning!
#endif
}

// Configure a port to output UBX, NMEA, RTCM3 or a combination thereof
bool SFE_UBLOX_GNSS::setI2COutput(uint8_t comSettings, uint16_t maxWait)
{
  return (setPortOutput(COM_PORT_I2C, comSettings, maxWait));
}
bool SFE_UBLOX_GNSS::setUART1Output(uint8_t comSettings, uint16_t maxWait)
{
  return (setPortOutput(COM_PORT_UART1, comSettings, maxWait));
}
bool SFE_UBLOX_GNSS::setUART2Output(uint8_t comSettings, uint16_t maxWait)
{
  return (setPortOutput(COM_PORT_UART2, comSettings, maxWait));
}
bool SFE_UBLOX_GNSS::setUSBOutput(uint8_t comSettings, uint16_t maxWait)
{
  return (setPortOutput(COM_PORT_USB, comSettings, maxWait));
}
bool SFE_UBLOX_GNSS::setSPIOutput(uint8_t comSettings, uint16_t maxWait)
{
  return (setPortOutput(COM_PORT_SPI, comSettings, maxWait));
}

// Want to see the NMEA messages on the Serial port? Here's how
void SFE_UBLOX_GNSS::setNMEAOutputPort(Stream &nmeaOutputPort)
{
  _nmeaOutputPort = &nmeaOutputPort; // Store the port from user
}

void SFE_UBLOX_GNSS::setOutputPort(Stream &outputPort)
{
  _outputPort = &outputPort; // Store the port from user
}

// Reset to defaults

void SFE_UBLOX_GNSS::factoryReset()
{
  // Copy default settings to permanent
  // Note: this does not load the permanent configuration into the current configuration. Calling factoryDefault() will do that.
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_CFG;
  packetCfg.len = 13;
  packetCfg.startingSpot = 0;
  for (uint8_t i = 0; i < 4; i++)
  {
    payloadCfg[0 + i] = 0xff; // clear mask: copy default config to permanent config
    payloadCfg[4 + i] = 0x00; // save mask: don't save current to permanent
    payloadCfg[8 + i] = 0x00; // load mask: don't copy permanent config to current
  }
  payloadCfg[12] = 0xff;      // all forms of permanent memory
  sendCommand(&packetCfg, 0); // don't expect ACK
  hardReset();                // cause factory default config to actually be loaded and used cleanly
}

void SFE_UBLOX_GNSS::hardReset()
{
  // Issue hard reset
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RST;
  packetCfg.len = 4;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = 0xff;       // cold start
  payloadCfg[1] = 0xff;       // cold start
  payloadCfg[2] = 0;          // 0=HW reset
  payloadCfg[3] = 0;          // reserved
  sendCommand(&packetCfg, 0); // don't expect ACK
}

void SFE_UBLOX_GNSS::softwareResetGNSSOnly()
{
  // Issue controlled software reset (GNSS only)
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RST;
  packetCfg.len = 4;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = 0;          // hot start
  payloadCfg[1] = 0;          // hot start
  payloadCfg[2] = 0x02;       // 0x02 = Controlled software reset (GNSS only)
  payloadCfg[3] = 0;          // reserved
  sendCommand(&packetCfg, 0); // don't expect ACK
}

// Reset module to factory defaults
// This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
bool SFE_UBLOX_GNSS::factoryDefault(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_CFG;
  packetCfg.len = 12;
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  packetCfg.payload[0] = 0xFF; // Set any bit in the clearMask field to clear saved config
  packetCfg.payload[1] = 0xFF;
  packetCfg.payload[8] = 0xFF; // Set any bit in the loadMask field to discard current config and rebuild from lower non-volatile memory layers
  packetCfg.payload[9] = 0xFF;

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Save configuration to BBR / Flash

// Save current configuration to flash and BBR (battery backed RAM)
// This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
bool SFE_UBLOX_GNSS::saveConfiguration(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_CFG;
  packetCfg.len = 12;
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  packetCfg.payload[4] = 0xFF; // Set any bit in the saveMask field to save current config to Flash and BBR
  packetCfg.payload[5] = 0xFF;

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Save the selected configuration sub-sections to flash and BBR (battery backed RAM)
// This still works but it is the old way of configuring ublox modules. See getVal and setVal for the new methods
bool SFE_UBLOX_GNSS::saveConfigSelective(uint32_t configMask, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_CFG;
  packetCfg.len = 12;
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  packetCfg.payload[4] = configMask & 0xFF; // Set the appropriate bits in the saveMask field to save current config to Flash and BBR
  packetCfg.payload[5] = (configMask >> 8) & 0xFF;
  packetCfg.payload[6] = (configMask >> 16) & 0xFF;
  packetCfg.payload[7] = (configMask >> 24) & 0xFF;

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Configure a given message type for a given port (UART1, I2C, SPI, etc)
bool SFE_UBLOX_GNSS::configureMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t sendRate, uint16_t maxWait)
{
  // Poll for the current settings for a given message
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 2;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = msgClass;
  payloadCfg[1] = msgID;

  // This will load the payloadCfg array with current settings of the given register
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);                                                       // If command send fails then bail

  // Now send it back with new mods
  packetCfg.len = 8;

  // payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[2 + portID] = sendRate; // Send rate is relative to the event a message is registered on. For example, if the rate of a navigation message is set to 2, the message is sent every 2nd navigation solution.

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Enable a given message type, default of 1 per update rate (usually 1 per second)
bool SFE_UBLOX_GNSS::enableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t rate, uint16_t maxWait)
{
  return (configureMessage(msgClass, msgID, portID, rate, maxWait));
}
// Disable a given message type on a given port
bool SFE_UBLOX_GNSS::disableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint16_t maxWait)
{
  return (configureMessage(msgClass, msgID, portID, 0, maxWait));
}

bool SFE_UBLOX_GNSS::enableNMEAMessage(uint8_t msgID, uint8_t portID, uint8_t rate, uint16_t maxWait)
{
  return (configureMessage(UBX_CLASS_NMEA, msgID, portID, rate, maxWait));
}
bool SFE_UBLOX_GNSS::disableNMEAMessage(uint8_t msgID, uint8_t portID, uint16_t maxWait)
{
  return (enableNMEAMessage(msgID, portID, 0, maxWait));
}

// Given a message number turns on a message ID for output over a given portID (UART, I2C, SPI, USB, etc)
// To disable a message, set secondsBetween messages to 0
// Note: This function will return false if the message is already enabled
// For base station RTK output we need to enable various sentences

// NEO-M8P has four:
// 1005 = 0xF5 0x05 - Stationary RTK reference ARP
// 1077 = 0xF5 0x4D - GPS MSM7
// 1087 = 0xF5 0x57 - GLONASS MSM7
// 1230 = 0xF5 0xE6 - GLONASS code-phase biases, set to once every 10 seconds

// ZED-F9P has six:
// 1005, 1074, 1084, 1094, 1124, 1230

// Much of this configuration is not documented and instead discerned from u-center binary console
bool SFE_UBLOX_GNSS::enableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint8_t sendRate, uint16_t maxWait)
{
  return (configureMessage(UBX_RTCM_MSB, messageNumber, portID, sendRate, maxWait));
}

// Disable a given message on a given port by setting secondsBetweenMessages to zero
bool SFE_UBLOX_GNSS::disableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait)
{
  return (enableRTCMmessage(messageNumber, portID, 0, maxWait));
}

// Functions used for RTK and base station setup

// Get the current TimeMode3 settings - these contain survey in statuses
bool SFE_UBLOX_GNSS::getSurveyMode(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_TMODE3;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_RECEIVED); // We are expecting data and an ACK
}

// Get the current TimeMode3 settings - these contain survey in statuses
bool SFE_UBLOX_GNSS::getSurveyMode(UBX_CFG_TMODE3_data_t *data, uint16_t maxWait)
{
  if (data == NULL) // Check if the user forgot to include the data pointer
    return (false); // Bail

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_TMODE3;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  // Extract the data
  data->version = extractByte(&packetCfg, 0);
  data->flags.all = extractInt(&packetCfg, 2);
  data->ecefXOrLat = extractSignedLong(&packetCfg, 4);
  data->ecefYOrLon = extractSignedLong(&packetCfg, 8);
  data->ecefZOrAlt = extractSignedLong(&packetCfg, 12);
  data->ecefXOrLatHP = extractSignedChar(&packetCfg, 16);
  data->ecefYOrLonHP = extractSignedChar(&packetCfg, 17);
  data->ecefZOrAltHP = extractSignedChar(&packetCfg, 18);
  data->fixedPosAcc = extractLong(&packetCfg, 20);
  data->svinMinDur = extractLong(&packetCfg, 24);
  data->svinAccLimit = extractLong(&packetCfg, 28);

  return (true);
}

// Control Survey-In for NEO-M8P
bool SFE_UBLOX_GNSS::setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint16_t maxWait)
{
  return (setSurveyModeFull(mode, (uint32_t)observationTime, requiredAccuracy, maxWait));
}
bool SFE_UBLOX_GNSS::setSurveyModeFull(uint8_t mode, uint32_t observationTime, float requiredAccuracy, uint16_t maxWait)
{
  if (getSurveyMode(maxWait) == false) // Ask module for the current TimeMode3 settings. Loads into payloadCfg.
    return (false);

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_TMODE3;
  packetCfg.len = 40;
  packetCfg.startingSpot = 0;

  // payloadCfg should be loaded with poll response. Now modify only the bits we care about
  payloadCfg[2] = mode; // Set mode. Survey-In and Disabled are most common. Use ECEF (not LAT/LON/ALT).

  // svinMinDur is U4 (uint32_t) in seconds
  payloadCfg[24] = observationTime & 0xFF; // svinMinDur in seconds
  payloadCfg[25] = (observationTime >> 8) & 0xFF;
  payloadCfg[26] = (observationTime >> 16) & 0xFF;
  payloadCfg[27] = (observationTime >> 24) & 0xFF;

  // svinAccLimit is U4 (uint32_t) in 0.1mm.
  uint32_t svinAccLimit = (uint32_t)(requiredAccuracy * 10000.0); // Convert m to 0.1mm
  payloadCfg[28] = svinAccLimit & 0xFF;                           // svinAccLimit in 0.1mm increments
  payloadCfg[29] = (svinAccLimit >> 8) & 0xFF;
  payloadCfg[30] = (svinAccLimit >> 16) & 0xFF;
  payloadCfg[31] = (svinAccLimit >> 24) & 0xFF;

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Begin Survey-In for NEO-M8P
bool SFE_UBLOX_GNSS::enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint16_t maxWait)
{
  return (setSurveyModeFull(SVIN_MODE_ENABLE, (uint32_t)observationTime, requiredAccuracy, maxWait));
}
bool SFE_UBLOX_GNSS::enableSurveyModeFull(uint32_t observationTime, float requiredAccuracy, uint16_t maxWait)
{
  return (setSurveyModeFull(SVIN_MODE_ENABLE, observationTime, requiredAccuracy, maxWait));
}

// Stop Survey-In for NEO-M8P
bool SFE_UBLOX_GNSS::disableSurveyMode(uint16_t maxWait)
{
  return (setSurveyMode(SVIN_MODE_DISABLE, 0, 0, maxWait));
}

// Set the ECEF or Lat/Long coordinates of a receiver
// This imediately puts the receiver in TIME mode (fixed) and will begin outputting RTCM sentences if enabled
// This is helpful once an antenna's position has been established. See this tutorial: https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station#gather-raw-gnss-data
//  For ECEF the units are: cm, 0.1mm, cm, 0.1mm, cm, 0.1mm
//  For Lat/Lon/Alt the units are: degrees^-7, degrees^-9, degrees^-7, degrees^-9, cm, 0.1mm
bool SFE_UBLOX_GNSS::setStaticPosition(int32_t ecefXOrLat, int8_t ecefXOrLatHP, int32_t ecefYOrLon, int8_t ecefYOrLonHP, int32_t ecefZOrAlt, int8_t ecefZOrAltHP, bool latLong, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_TMODE3;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the current TimeMode3 settings. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (false);

  packetCfg.len = 40;

  // customCfg should be loaded with poll response. Now modify only the bits we care about
  payloadCfg[2] = 2; // Set mode to fixed. Use ECEF (not LAT/LON/ALT).

  if (latLong == true)
    payloadCfg[3] = (uint8_t)(1 << 0); // Set mode to fixed. Use LAT/LON/ALT.

  // Set ECEF X or Lat
  payloadCfg[4] = (ecefXOrLat >> 8 * 0) & 0xFF; // LSB
  payloadCfg[5] = (ecefXOrLat >> 8 * 1) & 0xFF;
  payloadCfg[6] = (ecefXOrLat >> 8 * 2) & 0xFF;
  payloadCfg[7] = (ecefXOrLat >> 8 * 3) & 0xFF; // MSB

  // Set ECEF Y or Long
  payloadCfg[8] = (ecefYOrLon >> 8 * 0) & 0xFF; // LSB
  payloadCfg[9] = (ecefYOrLon >> 8 * 1) & 0xFF;
  payloadCfg[10] = (ecefYOrLon >> 8 * 2) & 0xFF;
  payloadCfg[11] = (ecefYOrLon >> 8 * 3) & 0xFF; // MSB

  // Set ECEF Z or Altitude
  payloadCfg[12] = (ecefZOrAlt >> 8 * 0) & 0xFF; // LSB
  payloadCfg[13] = (ecefZOrAlt >> 8 * 1) & 0xFF;
  payloadCfg[14] = (ecefZOrAlt >> 8 * 2) & 0xFF;
  payloadCfg[15] = (ecefZOrAlt >> 8 * 3) & 0xFF; // MSB

  // Set high precision parts
  payloadCfg[16] = ecefXOrLatHP;
  payloadCfg[17] = ecefYOrLonHP;
  payloadCfg[18] = ecefZOrAltHP;

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

bool SFE_UBLOX_GNSS::setStaticPosition(int32_t ecefXOrLat, int32_t ecefYOrLon, int32_t ecefZOrAlt, bool latlong, uint16_t maxWait)
{
  return (setStaticPosition(ecefXOrLat, 0, ecefYOrLon, 0, ecefZOrAlt, 0, latlong, maxWait));
}

// Set the DGNSS differential mode
bool SFE_UBLOX_GNSS::setDGNSSConfiguration(sfe_ublox_dgnss_mode_e dgnssMode, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_DGNSS;
  packetCfg.len = 4;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = (uint8_t)dgnssMode;
  payloadCfg[1] = 0; // reserved0
  payloadCfg[2] = 0;
  payloadCfg[3] = 0;

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Module Protocol Version

// Get the current protocol version of the u-blox module we're communicating with
// This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
uint8_t SFE_UBLOX_GNSS::getProtocolVersionHigh(uint16_t maxWait)
{
  if (moduleSWVersion == NULL)
    initModuleSWVersion();     // Check that RAM has been allocated for the SW version
  if (moduleSWVersion == NULL) // Bail if the RAM allocation failed
    return (false);

  if (moduleSWVersion->moduleQueried == false)
    getProtocolVersion(maxWait);
  return (moduleSWVersion->versionHigh);
}

// Get the current protocol version of the u-blox module we're communicating with
// This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
uint8_t SFE_UBLOX_GNSS::getProtocolVersionLow(uint16_t maxWait)
{
  if (moduleSWVersion == NULL)
    initModuleSWVersion();     // Check that RAM has been allocated for the SW version
  if (moduleSWVersion == NULL) // Bail if the RAM allocation failed
    return (false);

  if (moduleSWVersion->moduleQueried == false)
    getProtocolVersion(maxWait);
  return (moduleSWVersion->versionLow);
}

// Get the current protocol version of the u-blox module we're communicating with
// This is helpful when deciding if we should call the high-precision Lat/Long (HPPOSLLH) or the regular (POSLLH)
bool SFE_UBLOX_GNSS::getProtocolVersion(uint16_t maxWait)
{
  if (moduleSWVersion == NULL)
    initModuleSWVersion();     // Check that RAM has been allocated for the SW version
  if (moduleSWVersion == NULL) // Bail if the RAM allocation failed
    return (false);

  // Send packet with only CLS and ID, length of zero. This will cause the module to respond with the contents of that CLS/ID.
  packetCfg.cls = UBX_CLASS_MON;
  packetCfg.id = UBX_MON_VER;

  packetCfg.len = 0;
  packetCfg.startingSpot = 40; // Start at first "extended software information" string

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are only expecting data (no ACK)
    return (false);                                                       // If command send fails then bail

  // Payload should now contain ~220 characters (depends on module type)

  // if (_printDebug == true)
  // {
  //   _debugSerial->print(F("MON VER Payload:"));
  //   for (int location = 0; location < packetCfg.len; location++)
  //   {
  //     if (location % 30 == 0)
  //       _debugSerial->println();
  //     _debugSerial->write(payloadCfg[location]);
  //   }
  //   _debugSerial->println();
  // }

  // We will step through the payload looking at each extension field of 30 bytes
  for (uint8_t extensionNumber = 0; extensionNumber < 10; extensionNumber++)
  {
    // Now we need to find "PROTVER=18.00" in the incoming byte stream
    if ((payloadCfg[(30 * extensionNumber) + 0] == 'P') && (payloadCfg[(30 * extensionNumber) + 6] == 'R'))
    {
      moduleSWVersion->versionHigh = (payloadCfg[(30 * extensionNumber) + 8] - '0') * 10 + (payloadCfg[(30 * extensionNumber) + 9] - '0');  // Convert '18' to 18
      moduleSWVersion->versionLow = (payloadCfg[(30 * extensionNumber) + 11] - '0') * 10 + (payloadCfg[(30 * extensionNumber) + 12] - '0'); // Convert '00' to 00
      moduleSWVersion->moduleQueried = true;                                                                                                // Mark this data as new

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
      if (_printDebug == true)
      {
        _debugSerial->print(F("Protocol version: "));
        _debugSerial->print(moduleSWVersion->versionHigh);
        _debugSerial->print(F("."));
        _debugSerial->println(moduleSWVersion->versionLow);
      }
#endif
      return (true); // Success!
    }
  }

  return (false); // We failed
}

// PRIVATE: Allocate RAM for moduleSWVersion and initialize it
bool SFE_UBLOX_GNSS::initModuleSWVersion()
{
  moduleSWVersion = new moduleSWVersion_t; // Allocate RAM for the main struct
  if (moduleSWVersion == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initModuleSWVersion: RAM alloc failed!"));
#endif
    return (false);
  }
  moduleSWVersion->versionHigh = 0;
  moduleSWVersion->versionLow = 0;
  moduleSWVersion->moduleQueried = false;
  return (true);
}

// Geofences

// Add a new geofence using UBX-CFG-GEOFENCE
bool SFE_UBLOX_GNSS::addGeofence(int32_t latitude, int32_t longitude, uint32_t radius, byte confidence, byte pinPolarity, byte pin, uint16_t maxWait)
{
  if (currentGeofenceParams == NULL)
    initGeofenceParams();            // Check if RAM has been allocated for currentGeofenceParams
  if (currentGeofenceParams == NULL) // Abort if the RAM allocation failed
    return (false);

  if (currentGeofenceParams->numFences >= 4)
    return (false); // Quit if we already have four geofences defined

  // Store the new geofence parameters
  currentGeofenceParams->lats[currentGeofenceParams->numFences] = latitude;
  currentGeofenceParams->longs[currentGeofenceParams->numFences] = longitude;
  currentGeofenceParams->rads[currentGeofenceParams->numFences] = radius;
  currentGeofenceParams->numFences = currentGeofenceParams->numFences + 1; // Increment the number of fences

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_GEOFENCE;
  packetCfg.len = (currentGeofenceParams->numFences * 12) + 8;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = 0;                                // Message version = 0x00
  payloadCfg[1] = currentGeofenceParams->numFences; // numFences
  payloadCfg[2] = confidence;                       // confLvl = Confidence level 0-4 (none, 68%, 95%, 99.7%, 99.99%)
  payloadCfg[3] = 0;                                // reserved1
  if (pin > 0)
  {
    payloadCfg[4] = 1; // enable PIO combined fence state
  }
  else
  {
    payloadCfg[4] = 0; // disable PIO combined fence state
  }
  payloadCfg[5] = pinPolarity; // PIO pin polarity (0 = low means inside, 1 = low means outside (or unknown))
  payloadCfg[6] = pin;         // PIO pin
  payloadCfg[7] = 0;           // reserved2
  payloadCfg[8] = currentGeofenceParams->lats[0] & 0xFF;
  payloadCfg[9] = currentGeofenceParams->lats[0] >> 8;
  payloadCfg[10] = currentGeofenceParams->lats[0] >> 16;
  payloadCfg[11] = currentGeofenceParams->lats[0] >> 24;
  payloadCfg[12] = currentGeofenceParams->longs[0] & 0xFF;
  payloadCfg[13] = currentGeofenceParams->longs[0] >> 8;
  payloadCfg[14] = currentGeofenceParams->longs[0] >> 16;
  payloadCfg[15] = currentGeofenceParams->longs[0] >> 24;
  payloadCfg[16] = currentGeofenceParams->rads[0] & 0xFF;
  payloadCfg[17] = currentGeofenceParams->rads[0] >> 8;
  payloadCfg[18] = currentGeofenceParams->rads[0] >> 16;
  payloadCfg[19] = currentGeofenceParams->rads[0] >> 24;
  if (currentGeofenceParams->numFences >= 2)
  {
    payloadCfg[20] = currentGeofenceParams->lats[1] & 0xFF;
    payloadCfg[21] = currentGeofenceParams->lats[1] >> 8;
    payloadCfg[22] = currentGeofenceParams->lats[1] >> 16;
    payloadCfg[23] = currentGeofenceParams->lats[1] >> 24;
    payloadCfg[24] = currentGeofenceParams->longs[1] & 0xFF;
    payloadCfg[25] = currentGeofenceParams->longs[1] >> 8;
    payloadCfg[26] = currentGeofenceParams->longs[1] >> 16;
    payloadCfg[27] = currentGeofenceParams->longs[1] >> 24;
    payloadCfg[28] = currentGeofenceParams->rads[1] & 0xFF;
    payloadCfg[29] = currentGeofenceParams->rads[1] >> 8;
    payloadCfg[30] = currentGeofenceParams->rads[1] >> 16;
    payloadCfg[31] = currentGeofenceParams->rads[1] >> 24;
  }
  if (currentGeofenceParams->numFences >= 3)
  {
    payloadCfg[32] = currentGeofenceParams->lats[2] & 0xFF;
    payloadCfg[33] = currentGeofenceParams->lats[2] >> 8;
    payloadCfg[34] = currentGeofenceParams->lats[2] >> 16;
    payloadCfg[35] = currentGeofenceParams->lats[2] >> 24;
    payloadCfg[36] = currentGeofenceParams->longs[2] & 0xFF;
    payloadCfg[37] = currentGeofenceParams->longs[2] >> 8;
    payloadCfg[38] = currentGeofenceParams->longs[2] >> 16;
    payloadCfg[39] = currentGeofenceParams->longs[2] >> 24;
    payloadCfg[40] = currentGeofenceParams->rads[2] & 0xFF;
    payloadCfg[41] = currentGeofenceParams->rads[2] >> 8;
    payloadCfg[42] = currentGeofenceParams->rads[2] >> 16;
    payloadCfg[43] = currentGeofenceParams->rads[2] >> 24;
  }
  if (currentGeofenceParams->numFences >= 4)
  {
    payloadCfg[44] = currentGeofenceParams->lats[3] & 0xFF;
    payloadCfg[45] = currentGeofenceParams->lats[3] >> 8;
    payloadCfg[46] = currentGeofenceParams->lats[3] >> 16;
    payloadCfg[47] = currentGeofenceParams->lats[3] >> 24;
    payloadCfg[48] = currentGeofenceParams->longs[3] & 0xFF;
    payloadCfg[49] = currentGeofenceParams->longs[3] >> 8;
    payloadCfg[50] = currentGeofenceParams->longs[3] >> 16;
    payloadCfg[51] = currentGeofenceParams->longs[3] >> 24;
    payloadCfg[52] = currentGeofenceParams->rads[3] & 0xFF;
    payloadCfg[53] = currentGeofenceParams->rads[3] >> 8;
    payloadCfg[54] = currentGeofenceParams->rads[3] >> 16;
    payloadCfg[55] = currentGeofenceParams->rads[3] >> 24;
  }
  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Clear all geofences using UBX-CFG-GEOFENCE
bool SFE_UBLOX_GNSS::clearGeofences(uint16_t maxWait)
{
  if (currentGeofenceParams == NULL)
    initGeofenceParams();            // Check if RAM has been allocated for currentGeofenceParams
  if (currentGeofenceParams == NULL) // Abort if the RAM allocation failed
    return (false);

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_GEOFENCE;
  packetCfg.len = 8;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = 0; // Message version = 0x00
  payloadCfg[1] = 0; // numFences
  payloadCfg[2] = 0; // confLvl
  payloadCfg[3] = 0; // reserved1
  payloadCfg[4] = 0; // disable PIO combined fence state
  payloadCfg[5] = 0; // PIO pin polarity (0 = low means inside, 1 = low means outside (or unknown))
  payloadCfg[6] = 0; // PIO pin
  payloadCfg[7] = 0; // reserved2

  currentGeofenceParams->numFences = 0; // Zero the number of geofences currently in use

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Clear the antenna control settings using UBX-CFG-ANT
// This function is hopefully redundant but may be needed to release
// any PIO pins pre-allocated for antenna functions
bool SFE_UBLOX_GNSS::clearAntPIO(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_ANT;
  packetCfg.len = 4;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = 0x10; // Antenna flag mask: set the recovery bit
  payloadCfg[1] = 0;
  payloadCfg[2] = 0xFF; // Antenna pin configuration: set pinSwitch and pinSCD to 31
  payloadCfg[3] = 0xFF; // Antenna pin configuration: set pinOCD to 31, set reconfig bit

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Returns the combined geofence state using UBX-NAV-GEOFENCE
bool SFE_UBLOX_GNSS::getGeofenceState(geofenceState &currentGeofenceState, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_NAV;
  packetCfg.id = UBX_NAV_GEOFENCE;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the geofence status. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  currentGeofenceState.status = payloadCfg[5];    // Extract the status
  currentGeofenceState.numFences = payloadCfg[6]; // Extract the number of geofences
  currentGeofenceState.combState = payloadCfg[7]; // Extract the combined state of all geofences
  if (currentGeofenceState.numFences > 0)
    currentGeofenceState.states[0] = payloadCfg[8]; // Extract geofence 1 state
  if (currentGeofenceState.numFences > 1)
    currentGeofenceState.states[1] = payloadCfg[10]; // Extract geofence 2 state
  if (currentGeofenceState.numFences > 2)
    currentGeofenceState.states[2] = payloadCfg[12]; // Extract geofence 3 state
  if (currentGeofenceState.numFences > 3)
    currentGeofenceState.states[3] = payloadCfg[14]; // Extract geofence 4 state

  return (true);
}

// PRIVATE: Allocate RAM for currentGeofenceParams and initialize it
bool SFE_UBLOX_GNSS::initGeofenceParams()
{
  currentGeofenceParams = new geofenceParams_t; // Allocate RAM for the main struct
  if (currentGeofenceParams == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initGeofenceParams: RAM alloc failed!"));
#endif
    return (false);
  }
  currentGeofenceParams->numFences = 0;
  return (true);
}

// Power Save Mode
// Enables/Disables Low Power Mode using UBX-CFG-RXM
bool SFE_UBLOX_GNSS::powerSaveMode(bool power_save, uint16_t maxWait)
{
  // Let's begin by checking the Protocol Version as UBX_CFG_RXM is not supported on the ZED (protocol >= 27)
  uint8_t protVer = getProtocolVersionHigh(maxWait);
  /*
  if (_printDebug == true)
  {
    _debugSerial->print(F("Protocol version is "));
    _debugSerial->println(protVer);
  }
  */
  if (protVer >= 27)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("powerSaveMode (UBX-CFG-RXM) is not supported by this protocol version"));
    }
    return (false);
  }

  // Now let's change the power setting using UBX-CFG-RXM
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RXM;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the current power management settings. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  if (power_save)
  {
    payloadCfg[1] = 1; // Power Save Mode
  }
  else
  {
    payloadCfg[1] = 0; // Continuous Mode
  }

  packetCfg.len = 2;
  packetCfg.startingSpot = 0;

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Get Power Save Mode
// Returns the current Low Power Mode using UBX-CFG-RXM
// Returns 255 if the sendCommand fails
uint8_t SFE_UBLOX_GNSS::getPowerSaveMode(uint16_t maxWait)
{
  // Let's begin by checking the Protocol Version as UBX_CFG_RXM is not supported on the ZED (protocol >= 27)
  uint8_t protVer = getProtocolVersionHigh(maxWait);
  /*
  if (_printDebug == true)
  {
    _debugSerial->print(F("Protocol version is "));
    _debugSerial->println(protVer);
  }
  */
  if (protVer >= 27)
  {
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
    {
      _debugSerial->println(F("powerSaveMode (UBX-CFG-RXM) is not supported by this protocol version"));
    }
    return (255);
  }

  // Now let's read the power setting using UBX-CFG-RXM
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RXM;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the current power management settings. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (255);

  return (payloadCfg[1]); // Return the low power mode
}

// Powers off the GPS device for a given duration to reduce power consumption.
// NOTE: Querying the device before the duration is complete, for example by "getLatitude()" will wake it up!
// Returns true if command has not been not acknowledged.
// Returns false if command has not been acknowledged or maxWait = 0.
bool SFE_UBLOX_GNSS::powerOff(uint32_t durationInMs, uint16_t maxWait)
{
  // use durationInMs = 0 for infinite duration
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("Powering off for "));
    _debugSerial->print(durationInMs);
    _debugSerial->println(" ms");
  }
#endif

  // Power off device using UBX-RXM-PMREQ
  packetCfg.cls = UBX_CLASS_RXM; // 0x02
  packetCfg.id = UBX_RXM_PMREQ;  // 0x41
  packetCfg.len = 8;
  packetCfg.startingSpot = 0;

  // duration
  // big endian to little endian, switch byte order
  payloadCfg[0] = (durationInMs >> (8 * 0)) & 0xff;
  payloadCfg[1] = (durationInMs >> (8 * 1)) & 0xff;
  payloadCfg[2] = (durationInMs >> (8 * 2)) & 0xff;
  payloadCfg[3] = (durationInMs >> (8 * 3)) & 0xff;

  payloadCfg[4] = 0x02; // Flags : set the backup bit
  payloadCfg[5] = 0x00; // Flags
  payloadCfg[6] = 0x00; // Flags
  payloadCfg[7] = 0x00; // Flags

  if (maxWait != 0)
  {
    // check for "not acknowledged" command
    return (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_COMMAND_NACK);
  }
  else
  {
    sendCommand(&packetCfg, maxWait);
    return false; // can't tell if command not acknowledged if maxWait = 0
  }
}

// Powers off the GPS device for a given duration to reduce power consumption.
// While powered off it can be woken up by creating a falling or rising voltage edge on the specified pin.
// NOTE: The GPS seems to be sensitve to signals on the pins while powered off. Works best when Microcontroller is in deepsleep.
// NOTE: Querying the device before the duration is complete, for example by "getLatitude()" will wake it up!
// Returns true if command has not been not acknowledged.
// Returns false if command has not been acknowledged or maxWait = 0.
bool SFE_UBLOX_GNSS::powerOffWithInterrupt(uint32_t durationInMs, uint32_t wakeupSources, bool forceWhileUsb, uint16_t maxWait)
{
  // use durationInMs = 0 for infinite duration
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("Powering off for "));
    _debugSerial->print(durationInMs);
    _debugSerial->println(" ms");
  }
#endif

  // Power off device using UBX-RXM-PMREQ
  packetCfg.cls = UBX_CLASS_RXM; // 0x02
  packetCfg.id = UBX_RXM_PMREQ;  // 0x41
  packetCfg.len = 16;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = 0x00; // message version

  // bytes 1-3 are reserved - and must be set to zero
  payloadCfg[1] = 0x00;
  payloadCfg[2] = 0x00;
  payloadCfg[3] = 0x00;

  // duration
  // big endian to little endian, switch byte order
  payloadCfg[4] = (durationInMs >> (8 * 0)) & 0xff;
  payloadCfg[5] = (durationInMs >> (8 * 1)) & 0xff;
  payloadCfg[6] = (durationInMs >> (8 * 2)) & 0xff;
  payloadCfg[7] = (durationInMs >> (8 * 3)) & 0xff;

  // flags

  // disables USB interface when powering off, defaults to true
  if (forceWhileUsb)
  {
    payloadCfg[8] = 0x06; // force | backup
  }
  else
  {
    payloadCfg[8] = 0x02; // backup only (leave the force bit clear - module will stay on if USB is connected)
  }

  payloadCfg[9] = 0x00;
  payloadCfg[10] = 0x00;
  payloadCfg[11] = 0x00;

  // wakeUpSources

  // wakeupPin mapping, defaults to VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0

  // Possible values are:
  // VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX
  // VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0
  // VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT1
  // VAL_RXM_PMREQ_WAKEUPSOURCE_SPICS

  payloadCfg[12] = (wakeupSources >> (8 * 0)) & 0xff;
  payloadCfg[13] = (wakeupSources >> (8 * 1)) & 0xff;
  payloadCfg[14] = (wakeupSources >> (8 * 2)) & 0xff;
  payloadCfg[15] = (wakeupSources >> (8 * 3)) & 0xff;

  if (maxWait != 0)
  {
    // check for "not acknowledged" command
    return (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_COMMAND_NACK);
  }
  else
  {
    sendCommand(&packetCfg, maxWait);
    return false; // can't tell if command not acknowledged if maxWait = 0
  }
}

// Dynamic Platform Model

// Change the dynamic platform model using UBX-CFG-NAV5
// Possible values are:
// PORTABLE,STATIONARY,PEDESTRIAN,AUTOMOTIVE,SEA,
// AIRBORNE1g,AIRBORNE2g,AIRBORNE4g,WRIST,BIKE
// WRIST is not supported in protocol versions less than 18
// BIKE is supported in protocol versions 19.2
bool SFE_UBLOX_GNSS::setDynamicModel(dynModel newDynamicModel, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_NAV5;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the current navigation model settings. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  payloadCfg[0] = 0x01;            // mask: set only the dyn bit (0)
  payloadCfg[1] = 0x00;            // mask
  payloadCfg[2] = newDynamicModel; // dynModel

  packetCfg.len = 36;
  packetCfg.startingSpot = 0;

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Get the dynamic platform model using UBX-CFG-NAV5
// Returns DYN_MODEL_UNKNOWN (255) if the sendCommand fails
uint8_t SFE_UBLOX_GNSS::getDynamicModel(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_NAV5;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the current navigation model settings. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (DYN_MODEL_UNKNOWN);

  return (payloadCfg[2]); // Return the dynamic model
}

// Reset the odometer
bool SFE_UBLOX_GNSS::resetOdometer(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_NAV;
  packetCfg.id = UBX_NAV_RESETODO;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // This is a special case as we are only expecting an ACK but this is not a CFG message
  return (sendCommand(&packetCfg, maxWait, true) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Enable/Disable individual GNSS systems using UBX-CFG-GNSS
bool SFE_UBLOX_GNSS::enableGNSS(bool enable, sfe_ublox_gnss_ids_e id, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_GNSS;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  uint8_t numConfigBlocks = payloadCfg[3]; // Extract the numConfigBlocks

  for (uint8_t block = 0; block < numConfigBlocks; block++) // Check each configuration block
  {
    if (payloadCfg[(block * 8) + 4] == (uint8_t)id) // Check the gnssId for this block. Do we have a match?
    {
      // We have a match so set/clear the enable bit in flags
      if (enable)
        payloadCfg[(block * 8) + 4 + 4] |= 0x01; // Set the enable bit in flags (Little Endian)
      else
        payloadCfg[(block * 8) + 4 + 4] &= 0xFE; // Clear the enable bit in flags (Little Endian)
    }
  }

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Check if an individual GNSS system is enabled
bool SFE_UBLOX_GNSS::isGNSSenabled(sfe_ublox_gnss_ids_e id, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_GNSS;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  bool retVal = false;

  uint8_t numConfigBlocks = payloadCfg[3]; // Extract the numConfigBlocks

  for (uint8_t block = 0; block < numConfigBlocks; block++) // Check each configuration block
  {
    if (payloadCfg[(block * 8) + 4] == (uint8_t)id) // Check the gnssId for this block. Do we have a match?
    {
      // We have a match so check the enable bit in flags
      if ((payloadCfg[(block * 8) + 4 + 4] & 0x01) > 0) // Check the enable bit in flags (Little Endian)
        retVal = true;
    }
  }

  return (retVal);
}

// Reset ESF automatic IMU-mount alignment
bool SFE_UBLOX_GNSS::resetIMUalignment(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_ESF;
  packetCfg.id = UBX_ESF_RESETALG;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // This is a special case as we are only expecting an ACK but this is not a CFG message
  return (sendCommand(&packetCfg, maxWait, true) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// UBX-CFG-ESFALG is not documented. This was found using u-center.
// Returns the state of the UBX-CFG-ESFALG 'Automatic IMU-mount Alignment' flag
bool SFE_UBLOX_GNSS::getESFAutoAlignment(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_ESFALG;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("getESFAutoAlignment failed"));
    }
#endif

    return (false); // If command send fails then bail
  }

  return (payloadCfg[1] & 0b1); // Return Bit 0
}

// Set the state of the UBX-CFG-ESFALG 'Automatic IMU-mount Alignment' flag
bool SFE_UBLOX_GNSS::setESFAutoAlignment(bool enable, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_ESFALG;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("getESFAutoAlignment failed"));
    }
#endif

    return (false); // If command send fails then bail
  }

  // payloadCfg is now filled

  if (enable)
    payloadCfg[1] |= 0b1;
  else
    payloadCfg[1] &= ~(0b1);

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_SENT) // This time we are only expecting an ACK
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
    {
      _debugSerial->println(F("setESFAutoAlignment failed"));
    }
#endif
    return (false);
  }

  return (true);
}

// Get the time pulse parameters using UBX_CFG_TP5
bool SFE_UBLOX_GNSS::getTimePulseParameters(UBX_CFG_TP5_data_t *data, uint16_t maxWait)
{
  if (data == NULL) // Check if the user forgot to include the data pointer
    return (false); // Bail

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_TP5;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  // Extract the data
  data->tpIdx = extractByte(&packetCfg, 0);
  data->version = extractByte(&packetCfg, 1);
  data->antCableDelay = extractSignedInt(&packetCfg, 4);
  data->rfGroupDelay = extractSignedInt(&packetCfg, 6);
  data->freqPeriod = extractLong(&packetCfg, 8);
  data->freqPeriodLock = extractLong(&packetCfg, 12);
  data->pulseLenRatio = extractLong(&packetCfg, 16);
  data->pulseLenRatioLock = extractLong(&packetCfg, 20);
  data->userConfigDelay = extractSignedLong(&packetCfg, 24);
  data->flags.all = extractLong(&packetCfg, 28);

  return (true);
}

// Set the time pulse parameters using UBX_CFG_TP5
bool SFE_UBLOX_GNSS::setTimePulseParameters(UBX_CFG_TP5_data_t *data, uint16_t maxWait)
{
  if (data == NULL) // Check if the user forgot to include the data pointer
    return (false); // Bail

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_TP5;
  packetCfg.len = UBX_CFG_TP5_LEN;
  packetCfg.startingSpot = 0;

  // Insert the data
  payloadCfg[0] = data->tpIdx;
  payloadCfg[1] = data->version;
  payloadCfg[4] = data->antCableDelay & 0xFF; // Little Endian
  payloadCfg[5] = data->antCableDelay >> 8;
  payloadCfg[6] = data->rfGroupDelay & 0xFF; // Little Endian
  payloadCfg[7] = data->rfGroupDelay >> 8;
  payloadCfg[8] = data->freqPeriod & 0xFF; // Little Endian
  payloadCfg[9] = (data->freqPeriod >> 8) & 0xFF;
  payloadCfg[10] = (data->freqPeriod >> 16) & 0xFF;
  payloadCfg[11] = (data->freqPeriod >> 24) & 0xFF;
  payloadCfg[12] = data->freqPeriodLock & 0xFF; // Little Endian
  payloadCfg[13] = (data->freqPeriodLock >> 8) & 0xFF;
  payloadCfg[14] = (data->freqPeriodLock >> 16) & 0xFF;
  payloadCfg[15] = (data->freqPeriodLock >> 24) & 0xFF;
  payloadCfg[16] = data->pulseLenRatio & 0xFF; // Little Endian
  payloadCfg[17] = (data->pulseLenRatio >> 8) & 0xFF;
  payloadCfg[18] = (data->pulseLenRatio >> 16) & 0xFF;
  payloadCfg[19] = (data->pulseLenRatio >> 24) & 0xFF;
  payloadCfg[20] = data->pulseLenRatioLock & 0xFF; // Little Endian
  payloadCfg[21] = (data->pulseLenRatioLock >> 8) & 0xFF;
  payloadCfg[22] = (data->pulseLenRatioLock >> 16) & 0xFF;
  payloadCfg[23] = (data->pulseLenRatioLock >> 24) & 0xFF;
  payloadCfg[24] = data->userConfigDelay & 0xFF; // Little Endian
  payloadCfg[25] = (data->userConfigDelay >> 8) & 0xFF;
  payloadCfg[26] = (data->userConfigDelay >> 16) & 0xFF;
  payloadCfg[27] = (data->userConfigDelay >> 24) & 0xFF;
  payloadCfg[28] = data->flags.all & 0xFF; // Little Endian
  payloadCfg[29] = (data->flags.all >> 8) & 0xFF;
  payloadCfg[30] = (data->flags.all >> 16) & 0xFF;
  payloadCfg[31] = (data->flags.all >> 24) & 0xFF;

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Get the jamming/interference monitor configuration using UBX_CFG_ITFM
bool SFE_UBLOX_GNSS::getJammingConfiguration(UBX_CFG_ITFM_data_t *data, uint16_t maxWait)
{
  if (data == NULL) // Check if the user forgot to include the data pointer
    return (false); // Bail

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_ITFM;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  // Extract the data
  data->config.all = extractLong(&packetCfg, 0);
  data->config2.all = extractLong(&packetCfg, 4);

  return (true);
}

// Set the jamming/interference monitor configuration using UBX_CFG_ITFM
bool SFE_UBLOX_GNSS::setJammingConfiguration(UBX_CFG_ITFM_data_t *data, uint16_t maxWait)
{
  if (data == NULL) // Check if the user forgot to include the data pointer
    return (false); // Bail

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_ITFM;
  packetCfg.len = UBX_CFG_ITFM_LEN;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = data->config.all & 0xFF; // Little Endian
  payloadCfg[1] = (data->config.all >> 8) & 0xFF;
  payloadCfg[2] = (data->config.all >> 16) & 0xFF;
  payloadCfg[3] = (data->config.all >> 24) & 0xFF;
  payloadCfg[4] = data->config2.all & 0xFF; // Little Endian
  payloadCfg[5] = (data->config2.all >> 8) & 0xFF;
  payloadCfg[6] = (data->config2.all >> 16) & 0xFF;
  payloadCfg[7] = (data->config2.all >> 24) & 0xFF;

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Get the RF information using UBX_MON_RF
bool SFE_UBLOX_GNSS::getRFinformation(UBX_MON_RF_data_t *data, uint16_t maxWait)
{
  if (data == NULL) // Check if the user forgot to include the data pointer
    return (false); // Bail

  packetCfg.cls = UBX_CLASS_MON;
  packetCfg.id = UBX_MON_RF;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  // Extract the data
  data->header.version = extractByte(&packetCfg, 0);
  data->header.nBlocks = extractByte(&packetCfg, 1);

  // Extract the RF information blocks
  for (uint8_t block = 0; (block < data->header.nBlocks) && (block < UBX_MON_RF_MAX_BLOCKS); block++)
  {
    data->blocks[block].blockId = extractByte(&packetCfg, 4 + (block * 24));
    data->blocks[block].flags.all = extractByte(&packetCfg, 5 + (block * 24));
    data->blocks[block].antStatus = extractByte(&packetCfg, 6 + (block * 24));
    data->blocks[block].antPower = extractByte(&packetCfg, 7 + (block * 24));
    data->blocks[block].postStatus = extractLong(&packetCfg, 8 + (block * 24));
    data->blocks[block].noisePerMS = extractInt(&packetCfg, 16 + (block * 24));
    data->blocks[block].agcCnt = extractInt(&packetCfg, 18 + (block * 24));
    data->blocks[block].jamInd = extractByte(&packetCfg, 20 + (block * 24));
    data->blocks[block].ofsI = extractSignedChar(&packetCfg, 21 + (block * 24));
    data->blocks[block].magI = extractByte(&packetCfg, 22 + (block * 24));
    data->blocks[block].ofsQ = extractSignedChar(&packetCfg, 23 + (block * 24));
    data->blocks[block].magQ = extractByte(&packetCfg, 24 + (block * 24));
  }

  return (true);
}

// Get the hardware status (including jamming) using UBX_MON_HW
bool SFE_UBLOX_GNSS::getHWstatus(UBX_MON_HW_data_t *data, uint16_t maxWait)
{
  if (data == NULL) // Check if the user forgot to include the data pointer
    return (false); // Bail

  packetCfg.cls = UBX_CLASS_MON;
  packetCfg.id = UBX_MON_HW;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  // Extract the data
  data->pinSel = extractLong(&packetCfg, 0);
  data->pinBank = extractLong(&packetCfg, 4);
  data->pinDir = extractLong(&packetCfg, 8);
  data->pinVal = extractLong(&packetCfg, 12);
  data->noisePerMS = extractInt(&packetCfg, 16);
  data->agcCnt = extractInt(&packetCfg, 18);
  data->aStatus = extractByte(&packetCfg, 20);
  data->aPower = extractByte(&packetCfg, 21);
  data->flags.all = extractByte(&packetCfg, 22);
  data->usedMask = extractLong(&packetCfg, 24);
  for (uint8_t pin = 0; pin < 17; pin++)
  {
    data->VP[pin] = extractByte(&packetCfg, 28 + pin);
  }
  data->jamInd = extractByte(&packetCfg, 45);
  data->pinIrq = extractLong(&packetCfg, 48);
  data->pullH = extractLong(&packetCfg, 52);
  data->pullL = extractLong(&packetCfg, 56);

  return (true);
}

// Get the extended hardware status using UBX_MON_HW2
bool SFE_UBLOX_GNSS::getHW2status(UBX_MON_HW2_data_t *data, uint16_t maxWait)
{
  if (data == NULL) // Check if the user forgot to include the data pointer
    return (false); // Bail

  packetCfg.cls = UBX_CLASS_MON;
  packetCfg.id = UBX_MON_HW2;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  // Extract the data
  data->ofsI = extractSignedChar(&packetCfg, 0);
  data->magI = extractByte(&packetCfg, 1);
  data->ofsQ = extractSignedChar(&packetCfg, 2);
  data->magQ = extractByte(&packetCfg, 3);
  data->cfgSource = extractByte(&packetCfg, 4);
  data->lowLevCfg = extractLong(&packetCfg, 8); // Low-level configuration (obsolete for protocol versions greater than 15.00)
  data->postStatus = extractLong(&packetCfg, 20);

  return (true);
}

// UBX-CFG-NAVX5 - get/set the ackAiding byte. If ackAiding is 1, UBX-MGA-ACK messages will be sent by the module to acknowledge the MGA data
uint8_t SFE_UBLOX_GNSS::getAckAiding(uint16_t maxWait) // Get the ackAiding byte - returns 255 if the sendCommand fails
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_NAVX5;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (255);

  // Extract the ackAiding byte
  // There are three versions of UBX-CFG-NAVX5 but ackAiding is always in byte 17
  return (extractByte(&packetCfg, 17));
}
bool SFE_UBLOX_GNSS::setAckAiding(uint8_t ackAiding, uint16_t maxWait) // Set the ackAiding byte
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_NAVX5;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  // Set the ackAiding byte
  // There are three versions of UBX-CFG-NAVX5 but ackAiding is always in byte 17
  payloadCfg[17] = ackAiding;

  // There are three versions of UBX-CFG-NAVX5 but the ackAid flag is always in bit 10 of mask1
  payloadCfg[2] = 0x00; // Clear the LS byte of mask1
  payloadCfg[3] = 0x04; // Set _only_ the ackAid flag = bit 10 of mask1 = bit 2 of the MS byte
  payloadCfg[4] = 0x00; // Clear the LS byte of mask2, just in case
  payloadCfg[5] = 0x00; // Clear the LS byte of mask2, just in case

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// AssistNow Autonomous support
// UBX-CFG-NAVX5 - get the AssistNow Autonomous configuration (aopCfg) - returns 255 if the sendCommand fails
uint8_t SFE_UBLOX_GNSS::getAopCfg(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_NAVX5;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (255);

  // Extract the aopCfg byte
  // There are three versions of UBX-CFG-NAVX5 but aopCfg is always in byte 27
  return (extractByte(&packetCfg, 27));
}
// Set the aopCfg byte and the aopOrdMaxErr word
bool SFE_UBLOX_GNSS::setAopCfg(uint8_t aopCfg, uint16_t aopOrbMaxErr, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_NAVX5;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  // Set the aopCfg byte
  // There are three versions of UBX-CFG-NAVX5 but aopCfg is always in byte 27 and aopOrbMaxErr is always in bytes 30 & 31
  payloadCfg[27] = aopCfg;
  payloadCfg[30] = (uint8_t)(aopOrbMaxErr & 0xFF); // aopOrbMaxErr LSB
  payloadCfg[31] = (uint8_t)(aopOrbMaxErr >> 8);   // aopOrbMaxErr MSB

  // There are three versions of UBX-CFG-NAVX5 but the aop flag is always in bit 14 of mask1
  payloadCfg[2] = 0x00; // Clear the LS byte of mask1
  payloadCfg[3] = 0x40; // Set _only_ the aop flag = bit 14 of mask1 = bit 6 of the MS byte
  payloadCfg[4] = 0x00; // Clear the LS byte of mask2, just in case
  payloadCfg[5] = 0x00; // Clear the LS byte of mask2, just in case

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// SPARTN dynamic keys
//"When the receiver boots, the host should send 'current' and 'next' keys in one message." - Use setDynamicSPARTNKeys for this.
//"Every time the 'current' key is expired, 'next' takes its place."
//"Therefore the host should then retrieve the new 'next' key and send only that." - Use setDynamicSPARTNKey for this.
// The key can be provided in binary (uint8_t) format or in ASCII Hex (char) format, but in both cases keyLengthBytes _must_ represent the binary key length in bytes.
bool SFE_UBLOX_GNSS::setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const char *key)
{
  uint8_t *binaryKey = new uint8_t[keyLengthBytes]; // Allocate memory to store the binaryKey

  if (binaryKey == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
      _debugSerial->println(F("setDynamicSPARTNKey: binaryKey RAM allocation failed!"));
#endif
    return (false);
  }

  bool ok = true;

  // Convert the ASCII Hex const char to binary uint8_t
  for (uint16_t i = 0; i < ((uint16_t)keyLengthBytes * 2); i += 2)
  {
    if ((key[i] >= '0') && (key[i] <= '9'))
    {
      binaryKey[i >> 1] = (key[i] - '0') << 4;
    }
    else if ((key[i] >= 'a') && (key[i] <= 'f'))
    {
      binaryKey[i >> 1] = (key[i] + 10 - 'a') << 4;
    }
    else if ((key[i] >= 'A') && (key[i] <= 'F'))
    {
      binaryKey[i >> 1] = (key[i] + 10 - 'A') << 4;
    }
    else
    {
      ok = false;
    }

    if ((key[i + 1] >= '0') && (key[i + 1] <= '9'))
    {
      binaryKey[i >> 1] |= key[i + 1] - '0';
    }
    else if ((key[i + 1] >= 'a') && (key[i + 1] <= 'f'))
    {
      binaryKey[i >> 1] |= key[i + 1] + 10 - 'a';
    }
    else if ((key[i + 1] >= 'A') && (key[i + 1] <= 'F'))
    {
      binaryKey[i >> 1] |= key[i + 1] + 10 - 'A';
    }
    else
    {
      ok = false;
    }
  }

  if (ok)
    ok = setDynamicSPARTNKey(keyLengthBytes, validFromWno, validFromTow, (const uint8_t *)binaryKey);

  delete[] binaryKey; // Free the memory allocated for binaryKey

  return (ok);
}

bool SFE_UBLOX_GNSS::setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const uint8_t *key)
{
  // Check if there is room for the key in packetCfg. Resize the buffer if not.
  size_t payloadLength = (size_t)keyLengthBytes + 12;
  if (packetCfgPayloadSize < payloadLength)
  {
    if (!setPacketCfgPayloadSize(payloadLength)) // Check if the resize was successful
    {
      return (false);
    }
  }

  // Copy the key etc. into packetCfg
  packetCfg.cls = UBX_CLASS_RXM;
  packetCfg.id = UBX_RXM_SPARTNKEY;
  packetCfg.len = payloadLength;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = 0x01; // version
  payloadCfg[1] = 0x01; // numKeys
  payloadCfg[2] = 0x00; // reserved0
  payloadCfg[3] = 0x00; // reserved0
  payloadCfg[4] = 0x00; // reserved1
  payloadCfg[5] = keyLengthBytes;
  payloadCfg[6] = validFromWno & 0xFF; // validFromWno little-endian
  payloadCfg[7] = validFromWno >> 8;
  payloadCfg[8] = validFromTow & 0xFF; // validFromTow little-endian
  payloadCfg[9] = (validFromTow >> 8) & 0xFF;
  payloadCfg[10] = (validFromTow >> 16) & 0xFF;
  payloadCfg[11] = (validFromTow >> 24) & 0xFF;

  memcpy(&payloadCfg[12], key, keyLengthBytes);

  return (sendCommand(&packetCfg, 0) == SFE_UBLOX_STATUS_SUCCESS); // UBX-RXM-SPARTNKEY is silent. It does not ACK (or NACK)
}

bool SFE_UBLOX_GNSS::setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const char *key1,
                                          uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const char *key2)
{
  uint8_t *binaryKey1 = new uint8_t[keyLengthBytes1]; // Allocate memory to store binaryKey1

  if (binaryKey1 == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
      _debugSerial->println(F("setDynamicSPARTNKeys: binaryKey1 RAM allocation failed!"));
#endif
    return (false);
  }

  uint8_t *binaryKey2 = new uint8_t[keyLengthBytes2]; // Allocate memory to store binaryKey2

  if (binaryKey2 == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if (_printDebug == true)
      _debugSerial->println(F("setDynamicSPARTNKeys: binaryKey2 RAM allocation failed!"));
#endif
    delete[] binaryKey1;
    return (false);
  }

  bool ok = true;

  // Convert the ASCII Hex const char to binary uint8_t
  for (uint16_t i = 0; i < ((uint16_t)keyLengthBytes1 * 2); i += 2)
  {
    if ((key1[i] >= '0') && (key1[i] <= '9'))
    {
      binaryKey1[i >> 1] = (key1[i] - '0') << 4;
    }
    else if ((key1[i] >= 'a') && (key1[i] <= 'f'))
    {
      binaryKey1[i >> 1] = (key1[i] + 10 - 'a') << 4;
    }
    else if ((key1[i] >= 'A') && (key1[i] <= 'F'))
    {
      binaryKey1[i >> 1] = (key1[i] + 10 - 'A') << 4;
    }
    else
    {
      ok = false;
    }

    if ((key1[i + 1] >= '0') && (key1[i + 1] <= '9'))
    {
      binaryKey1[i >> 1] |= key1[i + 1] - '0';
    }
    else if ((key1[i + 1] >= 'a') && (key1[i + 1] <= 'f'))
    {
      binaryKey1[i >> 1] |= key1[i + 1] + 10 - 'a';
    }
    else if ((key1[i + 1] >= 'A') && (key1[i + 1] <= 'F'))
    {
      binaryKey1[i >> 1] |= key1[i + 1] + 10 - 'A';
    }
    else
    {
      ok = false;
    }
  }

  // Convert the ASCII Hex const char to binary uint8_t
  for (uint16_t i = 0; i < ((uint16_t)keyLengthBytes2 * 2); i += 2)
  {
    if ((key2[i] >= '0') && (key2[i] <= '9'))
    {
      binaryKey2[i >> 1] = (key2[i] - '0') << 4;
    }
    else if ((key2[i] >= 'a') && (key2[i] <= 'f'))
    {
      binaryKey2[i >> 1] = (key2[i] + 10 - 'a') << 4;
    }
    else if ((key2[i] >= 'A') && (key2[i] <= 'F'))
    {
      binaryKey2[i >> 1] = (key2[i] + 10 - 'A') << 4;
    }
    else
    {
      ok = false;
    }

    if ((key2[i + 1] >= '0') && (key2[i + 1] <= '9'))
    {
      binaryKey2[i >> 1] |= key2[i + 1] - '0';
    }
    else if ((key2[i + 1] >= 'a') && (key2[i + 1] <= 'f'))
    {
      binaryKey2[i >> 1] |= key2[i + 1] + 10 - 'a';
    }
    else if ((key2[i + 1] >= 'A') && (key2[i + 1] <= 'F'))
    {
      binaryKey2[i >> 1] |= key2[i + 1] + 10 - 'A';
    }
    else
    {
      ok = false;
    }
  }

  if (ok)
    ok = setDynamicSPARTNKeys(keyLengthBytes1, validFromWno1, validFromTow1, (const uint8_t *)binaryKey1,
                              keyLengthBytes2, validFromWno2, validFromTow2, (const uint8_t *)binaryKey2);

  delete[] binaryKey1; // Free the memory allocated for binaryKey1
  delete[] binaryKey2; // Free the memory allocated for binaryKey2

  return (ok);
}

bool SFE_UBLOX_GNSS::setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const uint8_t *key1,
                                          uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const uint8_t *key2)
{
  // Check if there is room for the key in packetCfg. Resize the buffer if not.
  size_t payloadLength = (size_t)keyLengthBytes1 + (size_t)keyLengthBytes2 + 20;
  if (packetCfgPayloadSize < payloadLength)
  {
    if (!setPacketCfgPayloadSize(payloadLength)) // Check if the resize was successful
    {
      return (false);
    }
  }

  // Copy the key etc. into packetCfg
  packetCfg.cls = UBX_CLASS_RXM;
  packetCfg.id = UBX_RXM_SPARTNKEY;
  packetCfg.len = payloadLength;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = 0x01; // version
  payloadCfg[1] = 0x02; // numKeys
  payloadCfg[2] = 0x00; // reserved0
  payloadCfg[3] = 0x00; // reserved0
  payloadCfg[4] = 0x00; // reserved1
  payloadCfg[5] = keyLengthBytes1;
  payloadCfg[6] = validFromWno1 & 0xFF; // validFromWno little-endian
  payloadCfg[7] = validFromWno1 >> 8;
  payloadCfg[8] = validFromTow1 & 0xFF; // validFromTow little-endian
  payloadCfg[9] = (validFromTow1 >> 8) & 0xFF;
  payloadCfg[10] = (validFromTow1 >> 16) & 0xFF;
  payloadCfg[11] = (validFromTow1 >> 24) & 0xFF;
  payloadCfg[12] = 0x00; // reserved1
  payloadCfg[13] = keyLengthBytes2;
  payloadCfg[14] = validFromWno2 & 0xFF; // validFromWno little-endian
  payloadCfg[15] = validFromWno2 >> 8;
  payloadCfg[16] = validFromTow2 & 0xFF; // validFromTow little-endian
  payloadCfg[17] = (validFromTow2 >> 8) & 0xFF;
  payloadCfg[18] = (validFromTow2 >> 16) & 0xFF;
  payloadCfg[19] = (validFromTow2 >> 24) & 0xFF;

  memcpy(&payloadCfg[20], key1, keyLengthBytes1);
  memcpy(&payloadCfg[20 + keyLengthBytes1], key2, keyLengthBytes2);

  return (sendCommand(&packetCfg, 0) == SFE_UBLOX_STATUS_SUCCESS); // UBX-RXM-SPARTNKEY is silent. It does not ACK (or NACK)
}

// CONFIGURATION INTERFACE (protocol v27 and above)

// Form 32-bit key from group/id/size
uint32_t SFE_UBLOX_GNSS::createKey(uint16_t group, uint16_t id, uint8_t size)
{
  uint32_t key = 0;
  key |= (uint32_t)id;
  key |= (uint32_t)group << 16;
  key |= (uint32_t)size << 28;
  return (key);
}

// Given a key, load the payload with data that can then be extracted to 8, 16, or 32 bits
// This function takes a full 32-bit key
// Default layer is RAM
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
sfe_ublox_status_e SFE_UBLOX_GNSS::getVal(uint32_t key, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALGET;
  packetCfg.len = 4 + 4 * 1; // While multiple keys are allowed, we will send only one key at a time
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  // VALGET uses different memory layer definitions to VALSET
  // because it can only return the value for one layer.
  // So we need to fiddle the layer here.
  // And just to complicate things further, the ZED-F9P only responds
  // correctly to layer 0 (RAM) and layer 7 (Default)!
  uint8_t getLayer = 7;                         // 7 is the "Default Layer"
  if ((layer & VAL_LAYER_RAM) == VAL_LAYER_RAM) // Did the user request the RAM layer?
  {
    getLayer = 0; // Layer 0 is RAM
  }

  payloadCfg[0] = 0;        // Message Version - set to 0
  payloadCfg[1] = getLayer; // Layer

  // Load key into outgoing payload
  payloadCfg[4] = key >> 8 * 0; // Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("key: 0x"));
    _debugSerial->print(key, HEX);
    _debugSerial->println();
  }
#endif

  // Send VALGET command with this key

  sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
  if (_printDebug == true)
  {
    _debugSerial->print(F("getVal: sendCommand returned: "));
    _debugSerial->println(statusString(retVal));
  }
#endif

  // Verify the response is the correct length as compared to what the user called (did the module respond with 8-bits but the user called getVal32?)
  // Response is 8 bytes plus cfg data
  // if(packet->len > 8+1)

  // The response is now sitting in payload, ready for extraction
  return (retVal);
}

// Given a key, return its value
// This function takes a full 32-bit key
// Default layer is RAM
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::getVal8(uint32_t key, uint8_t layer, uint16_t maxWait)
{
  if (getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (0);

  return (extractByte(&packetCfg, 8));
}
uint16_t SFE_UBLOX_GNSS::getVal16(uint32_t key, uint8_t layer, uint16_t maxWait)
{
  if (getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (0);

  return (extractInt(&packetCfg, 8));
}
uint32_t SFE_UBLOX_GNSS::getVal32(uint32_t key, uint8_t layer, uint16_t maxWait)
{
  if (getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (0);

  return (extractLong(&packetCfg, 8));
}
uint64_t SFE_UBLOX_GNSS::getVal64(uint32_t key, uint8_t layer, uint16_t maxWait)
{
  if (getVal(key, layer, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (0);

  return (extractLongLong(&packetCfg, 8));
}

// Given a group, ID and size, return the value of this config spot
// The 32-bit key is put together from group/ID/size. See other getVal to send key directly.
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::getVal8(uint16_t group, uint16_t id, uint8_t size, uint8_t layer, uint16_t maxWait)
{
  uint32_t key = createKey(group, id, size);
  return getVal8(key, layer, maxWait);
}
uint16_t SFE_UBLOX_GNSS::getVal16(uint16_t group, uint16_t id, uint8_t size, uint8_t layer, uint16_t maxWait)
{
  uint32_t key = createKey(group, id, size);
  return getVal16(key, layer, maxWait);
}
uint32_t SFE_UBLOX_GNSS::getVal32(uint16_t group, uint16_t id, uint8_t size, uint8_t layer, uint16_t maxWait)
{
  uint32_t key = createKey(group, id, size);
  return getVal32(key, layer, maxWait);
}
uint64_t SFE_UBLOX_GNSS::getVal64(uint16_t group, uint16_t id, uint8_t size, uint8_t layer, uint16_t maxWait)
{
  uint32_t key = createKey(group, id, size);
  return getVal64(key, layer, maxWait);
}

// Given a key, set a 16-bit value
// This function takes a full 32-bit key
// Default layer is all: RAM+BBR+Flash
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::setVal(uint32_t key, uint16_t value, uint8_t layer, uint16_t maxWait)
{
  return setVal16(key, value, layer, maxWait);
}

// Given a key, set a 16-bit value
// This function takes a full 32-bit key
// Default layer is all: RAM+BBR+Flash
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::setVal16(uint32_t key, uint16_t value, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + 2; // 4 byte header, 4 byte key ID, 2 bytes of value
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // Load key into outgoing payload
  payloadCfg[4] = key >> 8 * 0; // Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

  // Load user's value
  payloadCfg[8] = value >> 8 * 0; // Value LSB
  payloadCfg[9] = value >> 8 * 1;

  // Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Given a key, set an 8-bit value
// This function takes a full 32-bit key
// Default layer is all: RAM+BBR+Flash
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::setVal8(uint32_t key, uint8_t value, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + 1; // 4 byte header, 4 byte key ID, 1 byte value
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // Load key into outgoing payload
  payloadCfg[4] = key >> 8 * 0; // Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

  // Load user's value
  payloadCfg[8] = value; // Value

  // Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Given a key, set a 32-bit value
// This function takes a full 32-bit key
// Default layer is all: RAM+BBR+Flash
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::setVal32(uint32_t key, uint32_t value, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + 4; // 4 byte header, 4 byte key ID, 4 bytes of value
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // Load key into outgoing payload
  payloadCfg[4] = key >> 8 * 0; // Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

  // Load user's value
  payloadCfg[8] = value >> 8 * 0; // Value LSB
  payloadCfg[9] = value >> 8 * 1;
  payloadCfg[10] = value >> 8 * 2;
  payloadCfg[11] = value >> 8 * 3;

  // Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Given a key, set a 64-bit value
// This function takes a full 32-bit key
// Default layer is all: RAM+BBR+Flash
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::setVal64(uint32_t key, uint64_t value, uint8_t layer, uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + 8; // 4 byte header, 4 byte key ID, 8 bytes of value
  packetCfg.startingSpot = 0;

  // Clear packet payload
  memset(payloadCfg, 0, packetCfg.len);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // Load key into outgoing payload
  payloadCfg[4] = key >> 8 * 0; // Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

  // Load user's value
  payloadCfg[8] = value >> 8 * 0; // Value LSB
  payloadCfg[9] = value >> 8 * 1;
  payloadCfg[10] = value >> 8 * 2;
  payloadCfg[11] = value >> 8 * 3;
  payloadCfg[12] = value >> 8 * 4;
  payloadCfg[13] = value >> 8 * 5;
  payloadCfg[14] = value >> 8 * 6;
  payloadCfg[15] = value >> 8 * 7;

  // Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Start defining a new UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 64-bit value
// Default layer is RAM+BBR+Flash
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::newCfgValset64(uint32_t key, uint64_t value, uint8_t layer)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + 8; // 4 byte header, 4 byte key ID, 8 bytes of value
  packetCfg.startingSpot = 0;

  _numCfgKeyIDs = 1;

  // Clear all of packet payload
  memset(payloadCfg, 0, packetCfgPayloadSize);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // Load key into outgoing payload
  payloadCfg[4] = key >> 8 * 0; // Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

  // Load user's value
  payloadCfg[8] = value >> 8 * 0; // Value LSB
  payloadCfg[9] = value >> 8 * 1;
  payloadCfg[10] = value >> 8 * 2;
  payloadCfg[11] = value >> 8 * 3;
  payloadCfg[12] = value >> 8 * 4;
  payloadCfg[13] = value >> 8 * 5;
  payloadCfg[14] = value >> 8 * 6;
  payloadCfg[15] = value >> 8 * 7;

  // All done
  return (true);
}

// Start defining a new UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 32-bit value
// Default layer is RAM+BBR+Flash
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::newCfgValset32(uint32_t key, uint32_t value, uint8_t layer)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + 4; // 4 byte header, 4 byte key ID, 4 bytes of value
  packetCfg.startingSpot = 0;

  _numCfgKeyIDs = 1;

  // Clear all of packet payload
  memset(payloadCfg, 0, packetCfgPayloadSize);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // Load key into outgoing payload
  payloadCfg[4] = key >> 8 * 0; // Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

  // Load user's value
  payloadCfg[8] = value >> 8 * 0; // Value LSB
  payloadCfg[9] = value >> 8 * 1;
  payloadCfg[10] = value >> 8 * 2;
  payloadCfg[11] = value >> 8 * 3;

  // All done
  return (true);
}

// Start defining a new UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 16-bit value
// Default layer is RAM+BBR+Flash
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::newCfgValset16(uint32_t key, uint16_t value, uint8_t layer)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + 2; // 4 byte header, 4 byte key ID, 2 bytes of value
  packetCfg.startingSpot = 0;

  _numCfgKeyIDs = 1;

  // Clear all of packet payload
  memset(payloadCfg, 0, packetCfgPayloadSize);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // Load key into outgoing payload
  payloadCfg[4] = key >> 8 * 0; // Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

  // Load user's value
  payloadCfg[8] = value >> 8 * 0; // Value LSB
  payloadCfg[9] = value >> 8 * 1;

  // All done
  return (true);
}

// Start defining a new UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 8-bit value
// Default layer is RAM+BBR+Flash
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::newCfgValset8(uint32_t key, uint8_t value, uint8_t layer)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4 + 4 + 1; // 4 byte header, 4 byte key ID, 1 byte value
  packetCfg.startingSpot = 0;

  _numCfgKeyIDs = 1;

// Clear all of packet payload
  memset(payloadCfg, 0, packetCfgPayloadSize);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // Load key into outgoing payload
  payloadCfg[4] = key >> 8 * 0; // Key LSB
  payloadCfg[5] = key >> 8 * 1;
  payloadCfg[6] = key >> 8 * 2;
  payloadCfg[7] = key >> 8 * 3;

  // Load user's value
  payloadCfg[8] = value; // Value

  // All done
  return (true);
}

// Start defining a new (empty) UBX-CFG-VALSET ubxPacket
// Configuration of modern u-blox modules is now done via getVal/setVal/delVal, ie protocol v27 and above found on ZED-F9P
uint8_t SFE_UBLOX_GNSS::newCfgValset(uint8_t layer)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_VALSET;
  packetCfg.len = 4; // 4 byte header
  packetCfg.startingSpot = 0;

  _numCfgKeyIDs = 0;

  // Clear all of packet payload
  memset(payloadCfg, 0, packetCfgPayloadSize);

  payloadCfg[0] = 0;     // Message Version - set to 0
  payloadCfg[1] = layer; // By default we ask for the BBR layer

  // All done
  return (true);
}

// Add another keyID and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 64-bit value
uint8_t SFE_UBLOX_GNSS::addCfgValset64(uint32_t key, uint64_t value)
{
  if ((_autoSendAtSpaceRemaining > 0) && (packetCfg.len >= (packetCfgPayloadSize - _autoSendAtSpaceRemaining)))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset64: autosend"));
#endif
    sendCommand(&packetCfg);
    packetCfg.len = 4; // 4 byte header
    packetCfg.startingSpot = 0;
    _numCfgKeyIDs = 0;
    memset(&payloadCfg[4], 0, packetCfgPayloadSize - 4);
  }

  if (packetCfg.len >= (packetCfgPayloadSize - 12))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset64: packetCfgPayloadSize reached!"));
#endif
    return false;
  }

  if (_numCfgKeyIDs == CFG_VALSET_MAX_KEYS)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset64: key limit reached!"));
#endif
    return false;
  }

  // Load key into outgoing payload
  payloadCfg[packetCfg.len + 0] = key >> 8 * 0; // Key LSB
  payloadCfg[packetCfg.len + 1] = key >> 8 * 1;
  payloadCfg[packetCfg.len + 2] = key >> 8 * 2;
  payloadCfg[packetCfg.len + 3] = key >> 8 * 3;

  // Load user's value
  payloadCfg[packetCfg.len + 4] = value >> 8 * 0; // Value LSB
  payloadCfg[packetCfg.len + 5] = value >> 8 * 1;
  payloadCfg[packetCfg.len + 6] = value >> 8 * 2;
  payloadCfg[packetCfg.len + 7] = value >> 8 * 3;
  payloadCfg[packetCfg.len + 8] = value >> 8 * 4;
  payloadCfg[packetCfg.len + 9] = value >> 8 * 5;
  payloadCfg[packetCfg.len + 10] = value >> 8 * 6;
  payloadCfg[packetCfg.len + 11] = value >> 8 * 7;

  // Update packet length: 4 byte key ID, 8 bytes of value
  packetCfg.len = packetCfg.len + 4 + 8;

  _numCfgKeyIDs++;

  // All done
  return (true);
}

// Add another keyID and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 32-bit value
uint8_t SFE_UBLOX_GNSS::addCfgValset32(uint32_t key, uint32_t value)
{
  if ((_autoSendAtSpaceRemaining > 0) && (packetCfg.len >= (packetCfgPayloadSize - _autoSendAtSpaceRemaining)))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset32: autosend"));
#endif
    sendCommand(&packetCfg);
    packetCfg.len = 4; // 4 byte header
    packetCfg.startingSpot = 0;
    _numCfgKeyIDs = 0;
    memset(&payloadCfg[4], 0, packetCfgPayloadSize - 4);
  }

  if (packetCfg.len >= (packetCfgPayloadSize - 8))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset32: packetCfgPayloadSize reached!"));
#endif
    return false;
  }

  if (_numCfgKeyIDs == CFG_VALSET_MAX_KEYS)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset32: key limit reached!"));
#endif
    return false;
  }

  // Load key into outgoing payload
  payloadCfg[packetCfg.len + 0] = key >> 8 * 0; // Key LSB
  payloadCfg[packetCfg.len + 1] = key >> 8 * 1;
  payloadCfg[packetCfg.len + 2] = key >> 8 * 2;
  payloadCfg[packetCfg.len + 3] = key >> 8 * 3;

  // Load user's value
  payloadCfg[packetCfg.len + 4] = value >> 8 * 0; // Value LSB
  payloadCfg[packetCfg.len + 5] = value >> 8 * 1;
  payloadCfg[packetCfg.len + 6] = value >> 8 * 2;
  payloadCfg[packetCfg.len + 7] = value >> 8 * 3;

  // Update packet length: 4 byte key ID, 4 bytes of value
  packetCfg.len = packetCfg.len + 4 + 4;

  _numCfgKeyIDs++;

  // All done
  return (true);
}

// Add another keyID and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 16-bit value
uint8_t SFE_UBLOX_GNSS::addCfgValset16(uint32_t key, uint16_t value)
{
  if ((_autoSendAtSpaceRemaining > 0) && (packetCfg.len >= (packetCfgPayloadSize - _autoSendAtSpaceRemaining)))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset16: autosend"));
#endif
    sendCommand(&packetCfg);
    packetCfg.len = 4; // 4 byte header
    packetCfg.startingSpot = 0;
    _numCfgKeyIDs = 0;
    memset(&payloadCfg[4], 0, packetCfgPayloadSize - 4);
  }

  if (packetCfg.len >= (packetCfgPayloadSize - 6))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset16: packetCfgPayloadSize reached!"));
#endif
    return false;
  }

  if (_numCfgKeyIDs == CFG_VALSET_MAX_KEYS)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset16: key limit reached!"));
#endif
    return false;
  }

  // Load key into outgoing payload
  payloadCfg[packetCfg.len + 0] = key >> 8 * 0; // Key LSB
  payloadCfg[packetCfg.len + 1] = key >> 8 * 1;
  payloadCfg[packetCfg.len + 2] = key >> 8 * 2;
  payloadCfg[packetCfg.len + 3] = key >> 8 * 3;

  // Load user's value
  payloadCfg[packetCfg.len + 4] = value >> 8 * 0; // Value LSB
  payloadCfg[packetCfg.len + 5] = value >> 8 * 1;

  // Update packet length: 4 byte key ID, 2 bytes of value
  packetCfg.len = packetCfg.len + 4 + 2;

  _numCfgKeyIDs++;

  // All done
  return (true);
}

// Add another keyID and value to an existing UBX-CFG-VALSET ubxPacket
// This function takes a full 32-bit key and 8-bit value
uint8_t SFE_UBLOX_GNSS::addCfgValset8(uint32_t key, uint8_t value)
{
  if ((_autoSendAtSpaceRemaining > 0) && (packetCfg.len >= (packetCfgPayloadSize - _autoSendAtSpaceRemaining)))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset8: autosend"));
#endif
    sendCommand(&packetCfg);
    packetCfg.len = 4; // 4 byte header
    packetCfg.startingSpot = 0;
    _numCfgKeyIDs = 0;
    memset(&payloadCfg[4], 0, packetCfgPayloadSize - 4);
  }

  if (packetCfg.len >= (packetCfgPayloadSize - 5))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset8: packetCfgPayloadSize reached!"));
#endif
    return false;
  }

  if (_numCfgKeyIDs == CFG_VALSET_MAX_KEYS)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("addCfgValset8: key limit reached!"));
#endif
    return false;
  }

  // Load key into outgoing payload
  payloadCfg[packetCfg.len + 0] = key >> 8 * 0; // Key LSB
  payloadCfg[packetCfg.len + 1] = key >> 8 * 1;
  payloadCfg[packetCfg.len + 2] = key >> 8 * 2;
  payloadCfg[packetCfg.len + 3] = key >> 8 * 3;

  // Load user's value
  payloadCfg[packetCfg.len + 4] = value; // Value

  // Update packet length: 4 byte key ID, 1 byte value
  packetCfg.len = packetCfg.len + 4 + 1;

  _numCfgKeyIDs++;

  // All done
  return (true);
}

// Add a final keyID and value to an existing UBX-CFG-VALSET ubxPacket and send it
// This function takes a full 32-bit key and 64-bit value
uint8_t SFE_UBLOX_GNSS::sendCfgValset64(uint32_t key, uint64_t value, uint16_t maxWait)
{
  if ((_autoSendAtSpaceRemaining > 0) && (packetCfg.len >= (packetCfgPayloadSize - _autoSendAtSpaceRemaining)))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset64: autosend"));
#endif
    sendCommand(&packetCfg);
    packetCfg.len = 4; // 4 byte header
    packetCfg.startingSpot = 0;
    _numCfgKeyIDs = 0;
    memset(&payloadCfg[4], 0, packetCfgPayloadSize - 4);
  }

  if (packetCfg.len >= (packetCfgPayloadSize - 12))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset64: packetCfgPayloadSize reached!"));
#endif
  }
  else if (_numCfgKeyIDs == CFG_VALSET_MAX_KEYS)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset64: key limit reached!"));
#endif
  }
  else
    // Load keyID and value into outgoing payload
    addCfgValset64(key, value);

  _numCfgKeyIDs = 0;

  // Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Add a final keyID and value to an existing UBX-CFG-VALSET ubxPacket and send it
// This function takes a full 32-bit key and 32-bit value
uint8_t SFE_UBLOX_GNSS::sendCfgValset32(uint32_t key, uint32_t value, uint16_t maxWait)
{
  if ((_autoSendAtSpaceRemaining > 0) && (packetCfg.len >= (packetCfgPayloadSize - _autoSendAtSpaceRemaining)))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset32: autosend"));
#endif
    sendCommand(&packetCfg);
    packetCfg.len = 4; // 4 byte header
    packetCfg.startingSpot = 0;
    _numCfgKeyIDs = 0;
    memset(&payloadCfg[4], 0, packetCfgPayloadSize - 4);
  }

  if (packetCfg.len >= (packetCfgPayloadSize - 8))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset32: packetCfgPayloadSize reached!"));
#endif
  }
  else if (_numCfgKeyIDs == CFG_VALSET_MAX_KEYS)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset32: key limit reached!"));
#endif
  }
  else
    // Load keyID and value into outgoing payload
    addCfgValset32(key, value);

  _numCfgKeyIDs = 0;

  // Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Add a final keyID and value to an existing UBX-CFG-VALSET ubxPacket and send it
// This function takes a full 32-bit key and 16-bit value
uint8_t SFE_UBLOX_GNSS::sendCfgValset16(uint32_t key, uint16_t value, uint16_t maxWait)
{
  if ((_autoSendAtSpaceRemaining > 0) && (packetCfg.len >= (packetCfgPayloadSize - _autoSendAtSpaceRemaining)))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset16: autosend"));
#endif
    sendCommand(&packetCfg);
    packetCfg.len = 4; // 4 byte header
    packetCfg.startingSpot = 0;
    _numCfgKeyIDs = 0;
    memset(&payloadCfg[4], 0, packetCfgPayloadSize - 4);
  }

  if (packetCfg.len >= (packetCfgPayloadSize - 6))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset16: packetCfgPayloadSize reached!"));
#endif
  }
  else if (_numCfgKeyIDs == CFG_VALSET_MAX_KEYS)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset16: key limit reached!"));
#endif
  }
  else
    // Load keyID and value into outgoing payload
    addCfgValset16(key, value);

  _numCfgKeyIDs = 0;

  // Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Add a final keyID and value to an existing UBX-CFG-VALSET ubxPacket and send it
// This function takes a full 32-bit key and 8-bit value
uint8_t SFE_UBLOX_GNSS::sendCfgValset8(uint32_t key, uint8_t value, uint16_t maxWait)
{
  if ((_autoSendAtSpaceRemaining > 0) && (packetCfg.len >= (packetCfgPayloadSize - _autoSendAtSpaceRemaining)))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset8: autosend"));
#endif
    sendCommand(&packetCfg);
    packetCfg.len = 4; // 4 byte header
    packetCfg.startingSpot = 0;
    _numCfgKeyIDs = 0;
    memset(&payloadCfg[4], 0, packetCfgPayloadSize - 4);
  }

  if (packetCfg.len >= (packetCfgPayloadSize - 5))
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset8: packetCfgPayloadSize reached!"));
#endif
  }
  else if (_numCfgKeyIDs == CFG_VALSET_MAX_KEYS)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("sendCfgValset8: key limit reached!"));
#endif
  }
  else
    // Load keyID and value into outgoing payload
    addCfgValset8(key, value);

  _numCfgKeyIDs = 0;

  // Send VALSET command with this key and value
  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Send the UBX-CFG-VALSET ubxPacket
uint8_t SFE_UBLOX_GNSS::sendCfgValset(uint16_t maxWait)
{
  if (_numCfgKeyIDs == 0)
    return true; // Nothing to send...

  // Send VALSET command with this key and value
  bool success = sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT; // We are only expecting an ACK

  if (success)
    _numCfgKeyIDs = 0;

  return success;
}

// Return the number of keys in the CfgValset
uint8_t SFE_UBLOX_GNSS::getCfgValsetLen()
{
  return _numCfgKeyIDs;
}

// Return the number of free bytes remaining in packetCfgPayload
size_t SFE_UBLOX_GNSS::getCfgValsetSpaceRemaining()
{
  return getPacketCfgSpaceRemaining();
}

//=-=-=-=-=-=-=-= "Automatic" Messages =-=-=-=-=-=-=-==-=-=-=-=-=-=-=
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

// ***** NAV POSECEF automatic support

bool SFE_UBLOX_GNSS::getNAVPOSECEF(uint16_t maxWait)
{
  if (packetUBXNAVPOSECEF == NULL)
    initPacketUBXNAVPOSECEF();     // Check that RAM has been allocated for the POSECEF data
  if (packetUBXNAVPOSECEF == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPOSECEF->automaticFlags.flags.bits.automatic && packetUBXNAVPOSECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_POSECEF);
    return packetUBXNAVPOSECEF->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVPOSECEF->automaticFlags.flags.bits.automatic && !packetUBXNAVPOSECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_POSECEF;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPOSECEF
// works.
bool SFE_UBLOX_GNSS::setAutoNAVPOSECEF(bool enable, uint16_t maxWait)
{
  return setAutoNAVPOSECEFrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPOSECEF
// works.
bool SFE_UBLOX_GNSS::setAutoNAVPOSECEF(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVPOSECEFrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPOSECEF
// works.
bool SFE_UBLOX_GNSS::setAutoNAVPOSECEFrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVPOSECEF == NULL)
    initPacketUBXNAVPOSECEF();     // Check that RAM has been allocated for the data
  if (packetUBXNAVPOSECEF == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_POSECEF;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVPOSECEF->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVPOSECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVPOSECEF->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVPOSECEFcallback(void (*callbackPointer)(UBX_NAV_POSECEF_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVPOSECEF(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVPOSECEF->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVPOSECEF->callbackData = new UBX_NAV_POSECEF_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVPOSECEF->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVPOSECEFcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVPOSECEF->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoNAVPOSECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_POSECEF_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVPOSECEF(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVPOSECEF->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVPOSECEF->callbackData = new UBX_NAV_POSECEF_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVPOSECEF->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVPOSECEFcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVPOSECEF->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and POSECEF is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVPOSECEF(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVPOSECEF == NULL)
    initPacketUBXNAVPOSECEF();     // Check that RAM has been allocated for the data
  if (packetUBXNAVPOSECEF == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVPOSECEF->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVPOSECEF->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVPOSECEF->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVPOSECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVPOSECEF and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVPOSECEF()
{
  packetUBXNAVPOSECEF = new UBX_NAV_POSECEF_t; // Allocate RAM for the main struct
  if (packetUBXNAVPOSECEF == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVPOSECEF: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVPOSECEF->automaticFlags.flags.all = 0;
  packetUBXNAVPOSECEF->callbackPointer = NULL;
  packetUBXNAVPOSECEF->callbackPointerPtr = NULL;
  packetUBXNAVPOSECEF->callbackData = NULL;
  packetUBXNAVPOSECEF->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale. This is handy to get data alignment after CRC failure
// or if there are no helper functions and the user wants to request fresh data
void SFE_UBLOX_GNSS::flushNAVPOSECEF()
{
  if (packetUBXNAVPOSECEF == NULL)
    return;                                                 // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPOSECEF->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVPOSECEF(bool enabled)
{
  if (packetUBXNAVPOSECEF == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPOSECEF->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV STATUS automatic support

bool SFE_UBLOX_GNSS::getNAVSTATUS(uint16_t maxWait)
{
  if (packetUBXNAVSTATUS == NULL)
    initPacketUBXNAVSTATUS();     // Check that RAM has been allocated for the STATUS data
  if (packetUBXNAVSTATUS == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVSTATUS->automaticFlags.flags.bits.automatic && packetUBXNAVSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_STATUS);
    return packetUBXNAVSTATUS->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVSTATUS->automaticFlags.flags.bits.automatic && !packetUBXNAVSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_STATUS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getNAVSTATUS
// works.
bool SFE_UBLOX_GNSS::setAutoNAVSTATUS(bool enable, uint16_t maxWait)
{
  return setAutoNAVSTATUSrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getNAVSTATUS
// works.
bool SFE_UBLOX_GNSS::setAutoNAVSTATUS(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVSTATUSrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getNAVSTATUS
// works.
bool SFE_UBLOX_GNSS::setAutoNAVSTATUSrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVSTATUS == NULL)
    initPacketUBXNAVSTATUS();     // Check that RAM has been allocated for the data
  if (packetUBXNAVSTATUS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_STATUS;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVSTATUS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVSTATUS->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVSTATUScallback(void (*callbackPointer)(UBX_NAV_STATUS_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVSTATUS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVSTATUS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVSTATUS->callbackData = new UBX_NAV_STATUS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVSTATUS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVSTATUScallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVSTATUS->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoNAVSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_NAV_STATUS_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVSTATUS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVSTATUS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVSTATUS->callbackData = new UBX_NAV_STATUS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVSTATUS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVSTATUScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVSTATUS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and STATUS is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVSTATUS(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVSTATUS == NULL)
    initPacketUBXNAVSTATUS();     // Check that RAM has been allocated for the data
  if (packetUBXNAVSTATUS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVSTATUS->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVSTATUS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVSTATUS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVSTATUS and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVSTATUS()
{
  packetUBXNAVSTATUS = new UBX_NAV_STATUS_t; // Allocate RAM for the main struct
  if (packetUBXNAVSTATUS == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVSTATUS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVSTATUS->automaticFlags.flags.all = 0;
  packetUBXNAVSTATUS->callbackPointer = NULL;
  packetUBXNAVSTATUS->callbackPointerPtr = NULL;
  packetUBXNAVSTATUS->callbackData = NULL;
  packetUBXNAVSTATUS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale. This is handy to get data alignment after CRC failure
// or if there are no helper functions and the user wants to request fresh data
void SFE_UBLOX_GNSS::flushNAVSTATUS()
{
  if (packetUBXNAVSTATUS == NULL)
    return;                                                // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSTATUS->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVSTATUS(bool enabled)
{
  if (packetUBXNAVSTATUS == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSTATUS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** DOP automatic support

bool SFE_UBLOX_GNSS::getDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == NULL)
    initPacketUBXNAVDOP();     // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVDOP->automaticFlags.flags.bits.automatic && packetUBXNAVDOP->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getDOP: Autoreporting"));
    //  }
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_DOP);
    return packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVDOP->automaticFlags.flags.bits.automatic && !packetUBXNAVDOP->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getDOP: Exit immediately"));
    //  }
    return (false);
  }
  else
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getDOP: Polling"));
    // }

    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_DOP;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getDOP: data in packetCfg was OVERWRITTEN by another message (but that's OK)"));
      // }
      return (true);
    }

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("getDOP retVal: "));
    //   _debugSerial->println(statusString(retVal));
    // }
    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getDOP
// works.
bool SFE_UBLOX_GNSS::setAutoDOP(bool enable, uint16_t maxWait)
{
  return setAutoDOPrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getDOP
// works.
bool SFE_UBLOX_GNSS::setAutoDOP(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoDOPrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getDOP
// works.
bool SFE_UBLOX_GNSS::setAutoDOPrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVDOP == NULL)
    initPacketUBXNAVDOP();     // Check that RAM has been allocated for the data
  if (packetUBXNAVDOP == NULL) // Only attempt this if RAM allocation was successful
    return false;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_DOP;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVDOP->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVDOP->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoDOPcallback(void (*callbackPointer)(UBX_NAV_DOP_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoDOP(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVDOP->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVDOP->callbackData = new UBX_NAV_DOP_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVDOP->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoDOPcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVDOP->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoDOPcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_DOP_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoDOP(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVDOP->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVDOP->callbackData = new UBX_NAV_DOP_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVDOP->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoDOPcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVDOP->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and DOP is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoDOP(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVDOP == NULL)
    initPacketUBXNAVDOP();     // Check that RAM has been allocated for the data
  if (packetUBXNAVDOP == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVDOP->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVDOP->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVDOP->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVDOP->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVDOP and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVDOP()
{
  packetUBXNAVDOP = new UBX_NAV_DOP_t; // Allocate RAM for the main struct
  if (packetUBXNAVDOP == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVDOP: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVDOP->automaticFlags.flags.all = 0;
  packetUBXNAVDOP->callbackPointer = NULL;
  packetUBXNAVDOP->callbackPointerPtr = NULL;
  packetUBXNAVDOP->callbackData = NULL;
  packetUBXNAVDOP->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the DOP data as read/stale. This is handy to get data alignment after CRC failure
void SFE_UBLOX_GNSS::flushDOP()
{
  if (packetUBXNAVDOP == NULL)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVDOP->moduleQueried.moduleQueried.all = 0; // Mark all DOPs as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVDOP(bool enabled)
{
  if (packetUBXNAVDOP == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVDOP->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** EOE automatic support

bool SFE_UBLOX_GNSS::getNAVEOE(uint16_t maxWait)
{
  if (packetUBXNAVEOE == NULL)
    initPacketUBXNAVEOE();     // Check that RAM has been allocated for the EOE data
  if (packetUBXNAVEOE == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVEOE->automaticFlags.flags.bits.automatic && packetUBXNAVEOE->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getEOE: Autoreporting"));
    //  }
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_EOE);
    return packetUBXNAVEOE->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVEOE->automaticFlags.flags.bits.automatic && !packetUBXNAVEOE->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getEOE: Exit immediately"));
    //  }
    return (false);
  }
  else
  {
    // Note to self: NAV-EOE is "Periodic" (only). Not sure if it can be polled?

    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_EOE;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getEOE
// works.
bool SFE_UBLOX_GNSS::setAutoNAVEOE(bool enable, uint16_t maxWait)
{
  return setAutoNAVEOErate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getEOE
// works.
bool SFE_UBLOX_GNSS::setAutoNAVEOE(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVEOErate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getEOE
// works.
bool SFE_UBLOX_GNSS::setAutoNAVEOErate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVEOE == NULL)
    initPacketUBXNAVEOE();     // Check that RAM has been allocated for the data
  if (packetUBXNAVEOE == NULL) // Only attempt this if RAM allocation was successful
    return false;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_EOE;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVEOE->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVEOE->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVEOE->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVEOEcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_EOE_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVEOE(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVEOE->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVEOE->callbackData = new UBX_NAV_EOE_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVEOE->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVEOEcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVEOE->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and EOE is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVEOE(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVEOE == NULL)
    initPacketUBXNAVEOE();     // Check that RAM has been allocated for the data
  if (packetUBXNAVEOE == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVEOE->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVEOE->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVEOE->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVEOE->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVEOE and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVEOE()
{
  packetUBXNAVEOE = new UBX_NAV_EOE_t; // Allocate RAM for the main struct
  if (packetUBXNAVEOE == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVEOE: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVEOE->automaticFlags.flags.all = 0;
  packetUBXNAVEOE->callbackPointerPtr = NULL;
  packetUBXNAVEOE->callbackData = NULL;
  packetUBXNAVEOE->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the EOE data as read/stale. This is handy to get data alignment after CRC failure
void SFE_UBLOX_GNSS::flushNAVEOE()
{
  if (packetUBXNAVEOE == NULL)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVEOE->moduleQueried.moduleQueried.all = 0; // Mark all EOEs as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVEOE(bool enabled)
{
  if (packetUBXNAVEOE == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVEOE->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** VEH ATT automatic support

bool SFE_UBLOX_GNSS::getVehAtt(uint16_t maxWait)
{
  return (getNAVATT(maxWait));
}

bool SFE_UBLOX_GNSS::getNAVATT(uint16_t maxWait)
{
  if (packetUBXNAVATT == NULL)
    initPacketUBXNAVATT();     // Check that RAM has been allocated for the NAV ATT data
  if (packetUBXNAVATT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXNAVATT->automaticFlags.flags.bits.automatic && packetUBXNAVATT->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_ATT);
    return packetUBXNAVATT->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVATT->automaticFlags.flags.bits.automatic && !packetUBXNAVATT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting HNR PVT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_ATT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic NAV ATT message generation by the GNSS. This changes the way getVehAtt
// works.
bool SFE_UBLOX_GNSS::setAutoNAVATT(bool enable, uint16_t maxWait)
{
  return setAutoNAVATTrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic NAV ATT message generation by the GNSS. This changes the way getVehAtt
// works.
bool SFE_UBLOX_GNSS::setAutoNAVATT(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVATTrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic NAV ATT attitude message generation by the GNSS. This changes the way getVehAtt
// works.
bool SFE_UBLOX_GNSS::setAutoNAVATTrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVATT == NULL)
    initPacketUBXNAVATT();     // Check that RAM has been allocated for the data
  if (packetUBXNAVATT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_ATT;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVATT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVATT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVATTcallback(void (*callbackPointer)(UBX_NAV_ATT_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVATT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVATT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVATT->callbackData = new UBX_NAV_ATT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVATT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVATTcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVATT->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoNAVATTcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_ATT_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVATT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVATT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVATT->callbackData = new UBX_NAV_ATT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVATT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVATTcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVATT->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and NAV ATT attitude is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVATT(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVATT == NULL)
    initPacketUBXNAVATT();     // Check that RAM has been allocated for the NAV ATT data
  if (packetUBXNAVATT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVATT->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVATT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVATT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVATT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVATT and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVATT()
{
  packetUBXNAVATT = new UBX_NAV_ATT_t; // Allocate RAM for the main struct
  if (packetUBXNAVATT == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVATT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVATT->automaticFlags.flags.all = 0;
  packetUBXNAVATT->callbackPointer = NULL;
  packetUBXNAVATT->callbackPointerPtr = NULL;
  packetUBXNAVATT->callbackData = NULL;
  packetUBXNAVATT->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the ATT data as read/stale. This is handy to get data alignment after CRC failure
void SFE_UBLOX_GNSS::flushNAVATT()
{
  if (packetUBXNAVATT == NULL)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVATT->moduleQueried.moduleQueried.all = 0; // Mark all ATT data as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVATT(bool enabled)
{
  if (packetUBXNAVATT == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVATT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** PVT automatic support

// Get the latest Position/Velocity/Time solution and fill all global variables
bool SFE_UBLOX_GNSS::getPVT(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->automaticFlags.flags.bits.automatic && packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getPVT: Autoreporting"));
    //  }
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_PVT);
    return packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all;
  }
  else if (packetUBXNAVPVT->automaticFlags.flags.bits.automatic && !packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getPVT: Exit immediately"));
    //  }
    return (false);
  }
  else
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getPVT: Polling"));
    // }

    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_PVT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;
    // packetCfg.startingSpot = 20; //Begin listening at spot 20 so we can record up to 20+packetCfgPayloadSize = 84 bytes Note:now hard-coded in processUBX

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getPVT: data in packetCfg was OVERWRITTEN by another message (but that's OK)"));
      // }
      return (true);
    }

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("getPVT retVal: "));
    //   _debugSerial->println(statusString(retVal));
    // }
    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVT
// works.
bool SFE_UBLOX_GNSS::setAutoPVT(bool enable, uint16_t maxWait)
{
  return setAutoPVTrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVT
// works.
bool SFE_UBLOX_GNSS::setAutoPVT(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoPVTrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVT
// works.
bool SFE_UBLOX_GNSS::setAutoPVTrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_PVT;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVPVT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS. This changes the way getPVT works.
bool SFE_UBLOX_GNSS::setAutoPVTcallback(void (*callbackPointer)(UBX_NAV_PVT_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoPVT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAutoPVT failed

  if (packetUBXNAVPVT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVPVT->callbackData = new UBX_NAV_PVT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVPVT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoPVTcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVPVT->callbackPointer = callbackPointer; // RAM has been allocated so now update the pointer

  return (true);
}

bool SFE_UBLOX_GNSS::setAutoPVTcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_PVT_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoPVT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAutoPVT failed

  if (packetUBXNAVPVT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVPVT->callbackData = new UBX_NAV_PVT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVPVT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoPVTcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVPVT->callbackPointerPtr = callbackPointerPtr; // RAM has been allocated so now update the pointer

  return (true);
}

// In case no config access to the GNSS is possible and PVT is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoPVT(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVPVT->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVPVT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVPVT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVPVT and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVPVT()
{
  packetUBXNAVPVT = new UBX_NAV_PVT_t; // Allocate RAM for the main struct
  if (packetUBXNAVPVT == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVPVT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVPVT->automaticFlags.flags.all = 0;
  packetUBXNAVPVT->callbackPointer = NULL;
  packetUBXNAVPVT->callbackPointerPtr = NULL;
  packetUBXNAVPVT->callbackData = NULL;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0;
  packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0;
  return (true);
}

// Mark all the PVT data as read/stale. This is handy to get data alignment after CRC failure
void SFE_UBLOX_GNSS::flushPVT()
{
  if (packetUBXNAVPVT == NULL)
    return;                                              // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPVT->moduleQueried.moduleQueried1.all = 0; // Mark all datums as stale (read before)
  packetUBXNAVPVT->moduleQueried.moduleQueried2.all = 0;
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVPVT(bool enabled)
{
  if (packetUBXNAVPVT == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPVT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV ODO automatic support

bool SFE_UBLOX_GNSS::getNAVODO(uint16_t maxWait)
{
  if (packetUBXNAVODO == NULL)
    initPacketUBXNAVODO();     // Check that RAM has been allocated for the ODO data
  if (packetUBXNAVODO == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVODO->automaticFlags.flags.bits.automatic && packetUBXNAVODO->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_ODO);
    return packetUBXNAVODO->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVODO->automaticFlags.flags.bits.automatic && !packetUBXNAVODO->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_ODO;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getODO
// works.
bool SFE_UBLOX_GNSS::setAutoNAVODO(bool enable, uint16_t maxWait)
{
  return setAutoNAVODOrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getODO
// works.
bool SFE_UBLOX_GNSS::setAutoNAVODO(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVODOrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getODO
// works.
bool SFE_UBLOX_GNSS::setAutoNAVODOrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVODO == NULL)
    initPacketUBXNAVODO();     // Check that RAM has been allocated for the data
  if (packetUBXNAVODO == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_ODO;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVODO->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVODO->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVODO->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVODOcallback(void (*callbackPointer)(UBX_NAV_ODO_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVODO(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVODO->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVODO->callbackData = new UBX_NAV_ODO_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVODO->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVODOcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVODO->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoNAVODOcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_ODO_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVODO(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVODO->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVODO->callbackData = new UBX_NAV_ODO_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVODO->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVODOcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVODO->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ODO is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVODO(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVODO == NULL)
    initPacketUBXNAVODO();     // Check that RAM has been allocated for the data
  if (packetUBXNAVODO == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVODO->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVODO->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVODO->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVODO->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVODO and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVODO()
{
  packetUBXNAVODO = new UBX_NAV_ODO_t; // Allocate RAM for the main struct
  if (packetUBXNAVODO == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVODO: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVODO->automaticFlags.flags.all = 0;
  packetUBXNAVODO->callbackPointer = NULL;
  packetUBXNAVODO->callbackPointerPtr = NULL;
  packetUBXNAVODO->callbackData = NULL;
  packetUBXNAVODO->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushNAVODO()
{
  if (packetUBXNAVODO == NULL)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVODO->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVODO(bool enabled)
{
  if (packetUBXNAVODO == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVODO->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV VELECEF automatic support

bool SFE_UBLOX_GNSS::getNAVVELECEF(uint16_t maxWait)
{
  if (packetUBXNAVVELECEF == NULL)
    initPacketUBXNAVVELECEF();     // Check that RAM has been allocated for the VELECEF data
  if (packetUBXNAVVELECEF == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVVELECEF->automaticFlags.flags.bits.automatic && packetUBXNAVVELECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_VELECEF);
    return packetUBXNAVVELECEF->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVVELECEF->automaticFlags.flags.bits.automatic && !packetUBXNAVVELECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_VELECEF;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELECEF
// works.
bool SFE_UBLOX_GNSS::setAutoNAVVELECEF(bool enable, uint16_t maxWait)
{
  return setAutoNAVVELECEFrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELECEF
// works.
bool SFE_UBLOX_GNSS::setAutoNAVVELECEF(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVVELECEFrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELECEF
// works.
bool SFE_UBLOX_GNSS::setAutoNAVVELECEFrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVVELECEF == NULL)
    initPacketUBXNAVVELECEF();     // Check that RAM has been allocated for the data
  if (packetUBXNAVVELECEF == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_VELECEF;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVVELECEF->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVVELECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVVELECEF->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVVELECEFcallback(void (*callbackPointer)(UBX_NAV_VELECEF_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVVELECEF(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVVELECEF->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVVELECEF->callbackData = new UBX_NAV_VELECEF_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVVELECEF->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVVELECEFcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVVELECEF->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoNAVVELECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_VELECEF_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVVELECEF(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVVELECEF->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVVELECEF->callbackData = new UBX_NAV_VELECEF_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVVELECEF->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVVELECEFcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVVELECEF->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and VELECEF is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVVELECEF(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVVELECEF == NULL)
    initPacketUBXNAVVELECEF();     // Check that RAM has been allocated for the data
  if (packetUBXNAVVELECEF == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVVELECEF->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVVELECEF->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVVELECEF->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVVELECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVVELECEF and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVVELECEF()
{
  packetUBXNAVVELECEF = new UBX_NAV_VELECEF_t; // Allocate RAM for the main struct
  if (packetUBXNAVVELECEF == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVVELECEF: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVVELECEF->automaticFlags.flags.all = 0;
  packetUBXNAVVELECEF->callbackPointer = NULL;
  packetUBXNAVVELECEF->callbackPointerPtr = NULL;
  packetUBXNAVVELECEF->callbackData = NULL;
  packetUBXNAVVELECEF->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushNAVVELECEF()
{
  if (packetUBXNAVVELECEF == NULL)
    return;                                                 // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVVELECEF->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVVELECEF(bool enabled)
{
  if (packetUBXNAVVELECEF == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVVELECEF->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV VELNED automatic support

bool SFE_UBLOX_GNSS::getNAVVELNED(uint16_t maxWait)
{
  if (packetUBXNAVVELNED == NULL)
    initPacketUBXNAVVELNED();     // Check that RAM has been allocated for the VELNED data
  if (packetUBXNAVVELNED == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVVELNED->automaticFlags.flags.bits.automatic && packetUBXNAVVELNED->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_VELNED);
    return packetUBXNAVVELNED->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVVELNED->automaticFlags.flags.bits.automatic && !packetUBXNAVVELNED->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_VELNED;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELNED
// works.
bool SFE_UBLOX_GNSS::setAutoNAVVELNED(bool enable, uint16_t maxWait)
{
  return setAutoNAVVELNEDrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELNED
// works.
bool SFE_UBLOX_GNSS::setAutoNAVVELNED(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVVELNEDrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getVELNED
// works.
bool SFE_UBLOX_GNSS::setAutoNAVVELNEDrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVVELNED == NULL)
    initPacketUBXNAVVELNED();     // Check that RAM has been allocated for the data
  if (packetUBXNAVVELNED == NULL) // Only attempt this if RAM allocation was successful
    return false;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_VELNED;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVVELNED->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVVELNED->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVVELNED->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVVELNEDcallback(void (*callbackPointer)(UBX_NAV_VELNED_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVVELNED(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVVELNED->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVVELNED->callbackData = new UBX_NAV_VELNED_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVVELNED->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVVELNEDcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVVELNED->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoNAVVELNEDcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_VELNED_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVVELNED(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVVELNED->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVVELNED->callbackData = new UBX_NAV_VELNED_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVVELNED->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVVELNEDcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVVELNED->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and VELNED is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVVELNED(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVVELNED == NULL)
    initPacketUBXNAVVELNED();     // Check that RAM has been allocated for the data
  if (packetUBXNAVVELNED == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVVELNED->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVVELNED->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVVELNED->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVVELNED->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVVELNED and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVVELNED()
{
  packetUBXNAVVELNED = new UBX_NAV_VELNED_t; // Allocate RAM for the main struct
  if (packetUBXNAVVELNED == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVVELNED: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVVELNED->automaticFlags.flags.all = 0;
  packetUBXNAVVELNED->callbackPointer = NULL;
  packetUBXNAVVELNED->callbackPointerPtr = NULL;
  packetUBXNAVVELNED->callbackData = NULL;
  packetUBXNAVVELNED->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushNAVVELNED()
{
  if (packetUBXNAVVELNED == NULL)
    return;                                                // Bail if RAM has not been allocated (otherwise we could be writing anywhere!
  packetUBXNAVVELNED->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVVELNED(bool enabled)
{
  if (packetUBXNAVVELNED == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVVELNED->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV HPPOSECEF automatic support

bool SFE_UBLOX_GNSS::getNAVHPPOSECEF(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == NULL)
    initPacketUBXNAVHPPOSECEF();     // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.automatic && packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_HPPOSECEF);
    return packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.automatic && !packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_HPPOSECEF;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSECEF
// works.
bool SFE_UBLOX_GNSS::setAutoNAVHPPOSECEF(bool enable, uint16_t maxWait)
{
  return setAutoNAVHPPOSECEFrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSECEF
// works.
bool SFE_UBLOX_GNSS::setAutoNAVHPPOSECEF(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVHPPOSECEFrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSECEF
// works.
bool SFE_UBLOX_GNSS::setAutoNAVHPPOSECEFrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == NULL)
    initPacketUBXNAVHPPOSECEF();     // Check that RAM has been allocated for the data
  if (packetUBXNAVHPPOSECEF == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_HPPOSECEF;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVHPPOSECEFcallback(void (*callbackPointer)(UBX_NAV_HPPOSECEF_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVHPPOSECEF(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVHPPOSECEF->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVHPPOSECEF->callbackData = new UBX_NAV_HPPOSECEF_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVHPPOSECEF->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVHPPOSECEFcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVHPPOSECEF->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoNAVHPPOSECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_HPPOSECEF_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVHPPOSECEF(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVHPPOSECEF->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVHPPOSECEF->callbackData = new UBX_NAV_HPPOSECEF_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVHPPOSECEF->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVHPPOSECEFcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVHPPOSECEF->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and HPPOSECEF is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVHPPOSECEF(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVHPPOSECEF == NULL)
    initPacketUBXNAVHPPOSECEF();     // Check that RAM has been allocated for the data
  if (packetUBXNAVHPPOSECEF == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVHPPOSECEF and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVHPPOSECEF()
{
  packetUBXNAVHPPOSECEF = new UBX_NAV_HPPOSECEF_t; // Allocate RAM for the main struct
  if (packetUBXNAVHPPOSECEF == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVHPPOSECEF: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVHPPOSECEF->automaticFlags.flags.all = 0;
  packetUBXNAVHPPOSECEF->callbackPointer = NULL;
  packetUBXNAVHPPOSECEF->callbackPointerPtr = NULL;
  packetUBXNAVHPPOSECEF->callbackData = NULL;
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushNAVHPPOSECEF()
{
  if (packetUBXNAVHPPOSECEF == NULL)
    return;                                                   // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVHPPOSECEF(bool enabled)
{
  if (packetUBXNAVHPPOSECEF == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVHPPOSECEF->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV HPPOSLLH automatic support

bool SFE_UBLOX_GNSS::getHPPOSLLH(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic && packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getHPPOSLLH: Autoreporting"));
    //  }
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_HPPOSLLH);
    return packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic && !packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getHPPOSLLH: Exit immediately"));
    //  }
    return (false);
  }
  else
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getHPPOSLLH: Polling"));
    // }

    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_HPPOSLLH;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getHPPOSLLH: data in packetCfg was OVERWRITTEN by another message (but that's OK)"));
      // }
      return (true);
    }

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("getHPPOSLLH retVal: "));
    //   _debugSerial->println(statusString(retVal));
    // }
    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSLLH
// works.
bool SFE_UBLOX_GNSS::setAutoHPPOSLLH(bool enable, uint16_t maxWait)
{
  return setAutoHPPOSLLHrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSLLH
// works.
bool SFE_UBLOX_GNSS::setAutoHPPOSLLH(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoHPPOSLLHrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getHPPOSLLH
// works.
bool SFE_UBLOX_GNSS::setAutoHPPOSLLHrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the data
  if (packetUBXNAVHPPOSLLH == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_HPPOSLLH;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoHPPOSLLHcallback(void (*callbackPointer)(UBX_NAV_HPPOSLLH_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHPPOSLLH(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVHPPOSLLH->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVHPPOSLLH->callbackData = new UBX_NAV_HPPOSLLH_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVHPPOSLLH->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoHPPOSLLHcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVHPPOSLLH->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoHPPOSLLHcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_HPPOSLLH_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHPPOSLLH(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVHPPOSLLH->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVHPPOSLLH->callbackData = new UBX_NAV_HPPOSLLH_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVHPPOSLLH->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoHPPOSLLHcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVHPPOSLLH->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and HPPOSLLH is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoHPPOSLLH(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the data
  if (packetUBXNAVHPPOSLLH == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVHPPOSLLH and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVHPPOSLLH()
{
  packetUBXNAVHPPOSLLH = new UBX_NAV_HPPOSLLH_t; // Allocate RAM for the main struct
  if (packetUBXNAVHPPOSLLH == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVHPPOSLLH: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVHPPOSLLH->automaticFlags.flags.all = 0;
  packetUBXNAVHPPOSLLH->callbackPointer = NULL;
  packetUBXNAVHPPOSLLH->callbackPointerPtr = NULL;
  packetUBXNAVHPPOSLLH->callbackData = NULL;
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the HPPOSLLH data as read/stale. This is handy to get data alignment after CRC failure
void SFE_UBLOX_GNSS::flushHPPOSLLH()
{
  if (packetUBXNAVHPPOSLLH == NULL)
    return;                                                  // Bail if RAM has not been allocated (otherwise we could be writing anywhere!
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVHPPOSLLH(bool enabled)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVHPPOSLLH->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** PVAT automatic support

// Get the latest Position/Velocity/Time solution and fill all global variables
bool SFE_UBLOX_GNSS::getNAVPVAT(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == NULL)
    initPacketUBXNAVPVAT();     // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVAT->automaticFlags.flags.bits.automatic && packetUBXNAVPVAT->automaticFlags.flags.bits.implicitUpdate)
  {
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_PVAT);
    return packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all;
  }
  else if (packetUBXNAVPVAT->automaticFlags.flags.bits.automatic && !packetUBXNAVPVAT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_PVAT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVAT
// works.
bool SFE_UBLOX_GNSS::setAutoNAVPVAT(bool enable, uint16_t maxWait)
{
  return setAutoNAVPVATrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVAT
// works.
bool SFE_UBLOX_GNSS::setAutoNAVPVAT(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVPVATrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getPVAT
// works.
bool SFE_UBLOX_GNSS::setAutoNAVPVATrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVPVAT == NULL)
    initPacketUBXNAVPVAT();     // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_PVAT;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVPVAT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVPVAT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS. This changes the way getPVAT works.
bool SFE_UBLOX_GNSS::setAutoNAVPVATcallback(void (*callbackPointer)(UBX_NAV_PVAT_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVPVAT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAutoPVAT failed

  if (packetUBXNAVPVAT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVPVAT->callbackData = new UBX_NAV_PVAT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVPVAT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVPVATcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVPVAT->callbackPointer = callbackPointer; // RAM has been allocated so now update the pointer

  return (true);
}

bool SFE_UBLOX_GNSS::setAutoNAVPVATcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_PVAT_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVPVAT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAutoPVAT failed

  if (packetUBXNAVPVAT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVPVAT->callbackData = new UBX_NAV_PVAT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVPVAT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVPVATcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVPVAT->callbackPointerPtr = callbackPointerPtr; // RAM has been allocated so now update the pointer

  return (true);
}

// In case no config access to the GNSS is possible and PVAT is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVPVAT(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVPVAT == NULL)
    initPacketUBXNAVPVAT();     // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVPVAT->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVPVAT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVPVAT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVPVAT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVPVAT and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVPVAT()
{
  packetUBXNAVPVAT = new UBX_NAV_PVAT_t; // Allocate RAM for the main struct
  if (packetUBXNAVPVAT == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVPVAT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVPVAT->automaticFlags.flags.all = 0;
  packetUBXNAVPVAT->callbackPointer = NULL;
  packetUBXNAVPVAT->callbackPointerPtr = NULL;
  packetUBXNAVPVAT->callbackData = NULL;
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.all = 0;
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.all = 0;
  return (true);
}

// Mark all the PVAT data as read/stale. This is handy to get data alignment after CRC failure
void SFE_UBLOX_GNSS::flushNAVPVAT()
{
  if (packetUBXNAVPVAT == NULL)
    return;                                               // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.all = 0; // Mark all datums as stale (read before)
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.all = 0;
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVPVAT(bool enabled)
{
  if (packetUBXNAVPVAT == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVPVAT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV TIMEUTC automatic support

bool SFE_UBLOX_GNSS::getNAVTIMEUTC(uint16_t maxWait)
{
  if (packetUBXNAVTIMEUTC == NULL)
    initPacketUBXNAVTIMEUTC();     // Check that RAM has been allocated for the TIMEUTC data
  if (packetUBXNAVTIMEUTC == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.automatic && packetUBXNAVTIMEUTC->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_TIMEUTC);
    return packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVTIMEUTC->automaticFlags.flags.bits.automatic && !packetUBXNAVTIMEUTC->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_TIMEUTC;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMEUTC
// works.
bool SFE_UBLOX_GNSS::setAutoNAVTIMEUTC(bool enable, uint16_t maxWait)
{
  return setAutoNAVTIMEUTCrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMEUTC
// works.
bool SFE_UBLOX_GNSS::setAutoNAVTIMEUTC(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVTIMEUTCrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMEUTC
// works.
bool SFE_UBLOX_GNSS::setAutoNAVTIMEUTCrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVTIMEUTC == NULL)
    initPacketUBXNAVTIMEUTC();     // Check that RAM has been allocated for the data
  if (packetUBXNAVTIMEUTC == NULL) // Only attempt this if RAM allocation was successful
    return false;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_TIMEUTC;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVTIMEUTC->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVTIMEUTC->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVTIMEUTCcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_TIMEUTC_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVTIMEUTC(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVTIMEUTC->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVTIMEUTC->callbackData = new UBX_NAV_TIMEUTC_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVTIMEUTC->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVTIMEUTCcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVTIMEUTC->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and TIMEUTC is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVTIMEUTC(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVTIMEUTC == NULL)
    initPacketUBXNAVTIMEUTC();     // Check that RAM has been allocated for the data
  if (packetUBXNAVTIMEUTC == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVTIMEUTC->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVTIMEUTC->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVTIMEUTC->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVTIMEUTC->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVTIMEUTC and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVTIMEUTC()
{
  packetUBXNAVTIMEUTC = new UBX_NAV_TIMEUTC_t; // Allocate RAM for the main struct
  if (packetUBXNAVTIMEUTC == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVTIMEUTC: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVTIMEUTC->automaticFlags.flags.all = 0;
  packetUBXNAVTIMEUTC->callbackPointerPtr = NULL;
  packetUBXNAVTIMEUTC->callbackData = NULL;
  packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushNAVTIMEUTC()
{
  if (packetUBXNAVTIMEUTC == NULL)
    return;                                                 // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVTIMEUTC->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVTIMEUTC(bool enabled)
{
  if (packetUBXNAVTIMEUTC == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVTIMEUTC->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV CLOCK automatic support

bool SFE_UBLOX_GNSS::getNAVCLOCK(uint16_t maxWait)
{
  if (packetUBXNAVCLOCK == NULL)
    initPacketUBXNAVCLOCK();     // Check that RAM has been allocated for the CLOCK data
  if (packetUBXNAVCLOCK == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVCLOCK->automaticFlags.flags.bits.automatic && packetUBXNAVCLOCK->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_CLOCK);
    return packetUBXNAVCLOCK->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVCLOCK->automaticFlags.flags.bits.automatic && !packetUBXNAVCLOCK->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting CLOCK so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_CLOCK;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic CLOCK message generation by the GNSS. This changes the way getNAVCLOCK
// works.
bool SFE_UBLOX_GNSS::setAutoNAVCLOCK(bool enable, uint16_t maxWait)
{
  return setAutoNAVCLOCKrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic CLOCK message generation by the GNSS. This changes the way getNAVCLOCK
// works.
bool SFE_UBLOX_GNSS::setAutoNAVCLOCK(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVCLOCKrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic CLOCK message generation by the GNSS. This changes the way getNAVCLOCK
// works.
bool SFE_UBLOX_GNSS::setAutoNAVCLOCKrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVCLOCK == NULL)
    initPacketUBXNAVCLOCK();     // Check that RAM has been allocated for the data
  if (packetUBXNAVCLOCK == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_CLOCK;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVCLOCK->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVCLOCK->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVCLOCK->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVCLOCKcallback(void (*callbackPointer)(UBX_NAV_CLOCK_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVCLOCK(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVCLOCK->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVCLOCK->callbackData = new UBX_NAV_CLOCK_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVCLOCK->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVCLOCKcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVCLOCK->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoNAVCLOCKcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_CLOCK_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVCLOCK(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVCLOCK->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVCLOCK->callbackData = new UBX_NAV_CLOCK_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVCLOCK->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVCLOCKcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVCLOCK->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and NAV CLOCK is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVCLOCK(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVCLOCK == NULL)
    initPacketUBXNAVCLOCK();     // Check that RAM has been allocated for the CLOCK data
  if (packetUBXNAVCLOCK == NULL) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXNAVCLOCK->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVCLOCK->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVCLOCK->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVCLOCK->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVCLOCK and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVCLOCK()
{
  packetUBXNAVCLOCK = new UBX_NAV_CLOCK_t; // Allocate RAM for the main struct
  if (packetUBXNAVCLOCK == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVCLOCK: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVCLOCK->automaticFlags.flags.all = 0;
  packetUBXNAVCLOCK->callbackPointer = NULL;
  packetUBXNAVCLOCK->callbackPointerPtr = NULL;
  packetUBXNAVCLOCK->callbackData = NULL;
  packetUBXNAVCLOCK->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushNAVCLOCK()
{
  if (packetUBXNAVCLOCK == NULL)
    return;                                               // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVCLOCK->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVCLOCK(bool enabled)
{
  if (packetUBXNAVCLOCK == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVCLOCK->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV TIMELS automatic support

// Reads leap second event information and sets the global variables
// for future leap second change and number of leap seconds since GPS epoch
// Returns true if commands was successful
bool SFE_UBLOX_GNSS::getLeapSecondEvent(uint16_t maxWait)
{
  if (packetUBXNAVTIMELS == NULL)
    initPacketUBXNAVTIMELS();     // Check that RAM has been allocated for the TIMELS data
  if (packetUBXNAVTIMELS == NULL) // Abort if the RAM allocation failed
    return (false);

  packetCfg.cls = UBX_CLASS_NAV;
  packetCfg.id = UBX_NAV_TIMELS;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // The data is parsed as part of processing the response
  sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

  if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (true);

  if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
  {
    return (true);
  }

  return (false);
}

// PRIVATE: Allocate RAM for packetUBXNAVTIMELS and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVTIMELS()
{
  packetUBXNAVTIMELS = new UBX_NAV_TIMELS_t; // Allocate RAM for the main struct
  if (packetUBXNAVTIMELS == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVTIMELS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVTIMELS->automaticFlags.flags.all = 0;
  packetUBXNAVTIMELS->callbackPointer = NULL;
  packetUBXNAVTIMELS->callbackPointerPtr = NULL;
  packetUBXNAVTIMELS->callbackData = NULL;
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// ***** NAV SVIN automatic support

// Reads survey in status and sets the global variables
// for status, position valid, observation time, and mean 3D StdDev
// Returns true if commands was successful
bool SFE_UBLOX_GNSS::getSurveyStatus(uint16_t maxWait)
{
  if (packetUBXNAVSVIN == NULL)
    initPacketUBXNAVSVIN();     // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == NULL) // Abort if the RAM allocation failed
    return (false);

  if (packetUBXNAVSVIN->automaticFlags.flags.bits.automatic && packetUBXNAVSVIN->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_SVIN);
    return packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVSVIN->automaticFlags.flags.bits.automatic && !packetUBXNAVSVIN->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting SVIN so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_SVIN;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic SVIN message generation by the GNSS. This changes the way getSurveyStatus
// works.
bool SFE_UBLOX_GNSS::setAutoNAVSVIN(bool enable, uint16_t maxWait)
{
  return setAutoNAVSVINrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic SVIN message generation by the GNSS. This changes the way getSurveyStatus
// works.
bool SFE_UBLOX_GNSS::setAutoNAVSVIN(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVSVINrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic SVIN message generation by the GNSS. This changes the way getSurveyStatus
// works.
bool SFE_UBLOX_GNSS::setAutoNAVSVINrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVSVIN == NULL)
    initPacketUBXNAVSVIN();     // Check that RAM has been allocated for the data
  if (packetUBXNAVSVIN == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_SVIN;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVSVIN->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVSVIN->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVSVINcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_SVIN_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVSVIN(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVSVIN->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVSVIN->callbackData = new UBX_NAV_SVIN_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVSVIN->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVSVINcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVSVIN->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and SVIN is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVSVIN(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVSVIN == NULL)
    initPacketUBXNAVSVIN();     // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == NULL) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXNAVSVIN->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVSVIN->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVSVIN->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVSVIN->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVSVIN and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVSVIN()
{
  packetUBXNAVSVIN = new UBX_NAV_SVIN_t; // Allocate RAM for the main struct
  if (packetUBXNAVSVIN == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVSVIN: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVSVIN->automaticFlags.flags.all = 0;
  packetUBXNAVSVIN->callbackPointerPtr = NULL;
  packetUBXNAVSVIN->callbackData = NULL;
  packetUBXNAVSVIN->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushNAVSVIN()
{
  if (packetUBXNAVSVIN == NULL)
    return;                                              // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSVIN->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVSVIN(bool enabled)
{
  if (packetUBXNAVSVIN == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSVIN->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV SAT automatic support

// Signal information
// Returns true if commands was successful
bool SFE_UBLOX_GNSS::getNAVSAT(uint16_t maxWait)
{
  if (packetUBXNAVSAT == NULL)
    initPacketUBXNAVSAT();     // Check that RAM has been allocated for the NAVSAT data
  if (packetUBXNAVSAT == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVSAT->automaticFlags.flags.bits.automatic && packetUBXNAVSAT->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_SAT);
    return packetUBXNAVSAT->moduleQueried;
  }
  else if (packetUBXNAVSAT->automaticFlags.flags.bits.automatic && !packetUBXNAVSAT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting NAVSAT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_SAT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic NAVSAT message generation by the GNSS. This changes the way getNAVSAT
// works.
bool SFE_UBLOX_GNSS::setAutoNAVSAT(bool enable, uint16_t maxWait)
{
  return setAutoNAVSATrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic NAVSAT message generation by the GNSS. This changes the way getNAVSAT
// works.
bool SFE_UBLOX_GNSS::setAutoNAVSAT(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoNAVSATrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic NAV SAT message generation by the GNSS. This changes the way getNAVSAT
// works.
bool SFE_UBLOX_GNSS::setAutoNAVSATrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVSAT == NULL)
    initPacketUBXNAVSAT();     // Check that RAM has been allocated for the data
  if (packetUBXNAVSAT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_SAT;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVSAT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVSAT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVSAT->moduleQueried = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoNAVSATcallback(void (*callbackPointer)(UBX_NAV_SAT_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVSAT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVSAT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVSAT->callbackData = new UBX_NAV_SAT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVSAT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVSATcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVSAT->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoNAVSATcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_SAT_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoNAVSAT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVSAT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVSAT->callbackData = new UBX_NAV_SAT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVSAT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoNAVSATcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVSAT->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and NAV SAT is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoNAVSAT(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVSAT == NULL)
    initPacketUBXNAVSAT();     // Check that RAM has been allocated for the NAVSAT data
  if (packetUBXNAVSAT == NULL) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXNAVSAT->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVSAT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVSAT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVSAT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVSAT and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVSAT()
{
  packetUBXNAVSAT = new UBX_NAV_SAT_t; // Allocate RAM for the main struct
  if (packetUBXNAVSAT == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVSAT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVSAT->automaticFlags.flags.all = 0;
  packetUBXNAVSAT->callbackPointer = NULL;
  packetUBXNAVSAT->callbackPointerPtr = NULL;
  packetUBXNAVSAT->callbackData = NULL;
  packetUBXNAVSAT->moduleQueried = false;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushNAVSAT()
{
  if (packetUBXNAVSAT == NULL)
    return;                               // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSAT->moduleQueried = false; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVSAT(bool enabled)
{
  if (packetUBXNAVSAT == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVSAT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** NAV RELPOSNED automatic support

// Relative Positioning Information in NED frame
// Returns true if commands was successful
// Note:
//   RELPOSNED on the M8 is only 40 bytes long
//   RELPOSNED on the F9 is 64 bytes long and contains much more information
bool SFE_UBLOX_GNSS::getRELPOSNED(uint16_t maxWait)
{
  if (packetUBXNAVRELPOSNED == NULL)
    initPacketUBXNAVRELPOSNED();     // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.automatic && packetUBXNAVRELPOSNED->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_RELPOSNED);
    return packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVRELPOSNED->automaticFlags.flags.bits.automatic && !packetUBXNAVRELPOSNED->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting RELPOSNED so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_RELPOSNED;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic RELPOSNED message generation by the GNSS. This changes the way getRELPOSNED
// works.
bool SFE_UBLOX_GNSS::setAutoRELPOSNED(bool enable, uint16_t maxWait)
{
  return setAutoRELPOSNEDrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic RELPOSNED message generation by the GNSS. This changes the way getRELPOSNED
// works.
bool SFE_UBLOX_GNSS::setAutoRELPOSNED(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoRELPOSNEDrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic RELPOSNED message generation by the GNSS. This changes the way getRELPOSNED
// works.
bool SFE_UBLOX_GNSS::setAutoRELPOSNEDrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVRELPOSNED == NULL)
    initPacketUBXNAVRELPOSNED();     // Check that RAM has been allocated for the data
  if (packetUBXNAVRELPOSNED == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_RELPOSNED;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVRELPOSNED->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVRELPOSNED->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoRELPOSNEDcallback(void (*callbackPointer)(UBX_NAV_RELPOSNED_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoRELPOSNED(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVRELPOSNED->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVRELPOSNED->callbackData = new UBX_NAV_RELPOSNED_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVRELPOSNED->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoRELPOSNEDcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVRELPOSNED->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoRELPOSNEDcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_RELPOSNED_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoRELPOSNED(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVRELPOSNED->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVRELPOSNED->callbackData = new UBX_NAV_RELPOSNED_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVRELPOSNED->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoRELPOSNEDcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVRELPOSNED->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and RELPOSNED is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoRELPOSNED(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVRELPOSNED == NULL)
    initPacketUBXNAVRELPOSNED();     // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == NULL) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXNAVRELPOSNED->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVRELPOSNED->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVRELPOSNED->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVRELPOSNED->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVRELPOSNED and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVRELPOSNED()
{
  packetUBXNAVRELPOSNED = new UBX_NAV_RELPOSNED_t; // Allocate RAM for the main struct
  if (packetUBXNAVRELPOSNED == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVRELPOSNED: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVRELPOSNED->automaticFlags.flags.all = 0;
  packetUBXNAVRELPOSNED->callbackPointer = NULL;
  packetUBXNAVRELPOSNED->callbackPointerPtr = NULL;
  packetUBXNAVRELPOSNED->callbackData = NULL;
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushNAVRELPOSNED()
{
  if (packetUBXNAVRELPOSNED == NULL)
    return;                                                   // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logNAVRELPOSNED(bool enabled)
{
  if (packetUBXNAVRELPOSNED == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVRELPOSNED->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** AOPSTATUS automatic support

bool SFE_UBLOX_GNSS::getAOPSTATUS(uint16_t maxWait)
{
  if (packetUBXNAVAOPSTATUS == NULL)
    initPacketUBXNAVAOPSTATUS();     // Check that RAM has been allocated for the AOPSTATUS data
  if (packetUBXNAVAOPSTATUS == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.automatic && packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getAOPSTATUS: Autoreporting"));
    //  }
    checkUbloxInternal(&packetCfg, UBX_CLASS_NAV, UBX_NAV_AOPSTATUS);
    return packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.automatic && !packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getAOPSTATUS: Exit immediately"));
    //  }
    return (false);
  }
  else
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getAOPSTATUS: Polling"));
    // }

    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_NAV;
    packetCfg.id = UBX_NAV_AOPSTATUS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getAOPSTATUS: data in packetCfg was OVERWRITTEN by another message (but that's OK)"));
      // }
      return (true);
    }

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("getAOPSTATUS retVal: "));
    //   _debugSerial->println(statusString(retVal));
    // }
    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getAOPSTATUS
// works.
bool SFE_UBLOX_GNSS::setAutoAOPSTATUS(bool enable, uint16_t maxWait)
{
  return setAutoAOPSTATUSrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getAOPSTATUS
// works.
bool SFE_UBLOX_GNSS::setAutoAOPSTATUS(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoAOPSTATUSrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getAOPSTATUS
// works.
bool SFE_UBLOX_GNSS::setAutoAOPSTATUSrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXNAVAOPSTATUS == NULL)
    initPacketUBXNAVAOPSTATUS();     // Check that RAM has been allocated for the data
  if (packetUBXNAVAOPSTATUS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_NAV;
  payloadCfg[1] = UBX_NAV_AOPSTATUS;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoAOPSTATUScallback(void (*callbackPointer)(UBX_NAV_AOPSTATUS_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoAOPSTATUS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVAOPSTATUS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVAOPSTATUS->callbackData = new UBX_NAV_AOPSTATUS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVAOPSTATUS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoAOPSTATUScallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVAOPSTATUS->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoAOPSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_NAV_AOPSTATUS_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoAOPSTATUS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXNAVAOPSTATUS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXNAVAOPSTATUS->callbackData = new UBX_NAV_AOPSTATUS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXNAVAOPSTATUS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoAOPSTATUScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXNAVAOPSTATUS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and AOPSTATUS is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoAOPSTATUS(bool enabled, bool implicitUpdate)
{
  if (packetUBXNAVAOPSTATUS == NULL)
    initPacketUBXNAVAOPSTATUS();     // Check that RAM has been allocated for the data
  if (packetUBXNAVAOPSTATUS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.automatic != enabled || packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXNAVAOPSTATUS and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXNAVAOPSTATUS()
{
  packetUBXNAVAOPSTATUS = new UBX_NAV_AOPSTATUS_t; // Allocate RAM for the main struct
  if (packetUBXNAVAOPSTATUS == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXNAVAOPSTATUS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXNAVAOPSTATUS->automaticFlags.flags.all = 0;
  packetUBXNAVAOPSTATUS->callbackPointer = NULL;
  packetUBXNAVAOPSTATUS->callbackPointerPtr = NULL;
  packetUBXNAVAOPSTATUS->callbackData = NULL;
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the AOPSTATUS data as read/stale. This is handy to get data alignment after CRC failure
void SFE_UBLOX_GNSS::flushAOPSTATUS()
{
  if (packetUBXNAVAOPSTATUS == NULL)
    return;                                                   // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.all = 0; // Mark all AOPSTATUSs as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logAOPSTATUS(bool enabled)
{
  if (packetUBXNAVAOPSTATUS == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXNAVAOPSTATUS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** RXM PMP automatic support

// Callback receives a pointer to the data, instead of _all_ the data. Much kinder on the stack!
bool SFE_UBLOX_GNSS::setRXMPMPcallbackPtr(void (*callbackPointer)(UBX_RXM_PMP_data_t *))
{
  if (packetUBXRXMPMP == NULL)
    initPacketUBXRXMPMP();     // Check that RAM has been allocated for the data
  if (packetUBXRXMPMP == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXRXMPMP->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMPMP->callbackData = new UBX_RXM_PMP_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMPMP->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoRXMPMPcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMPMP->callbackPointerPtr = callbackPointer;
  return (true);
}

// PRIVATE: Allocate RAM for packetUBXRXMPMP and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXRXMPMP()
{
  packetUBXRXMPMP = new UBX_RXM_PMP_t; // Allocate RAM for the main struct
  if (packetUBXRXMPMP == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXRXMPMP: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMPMP->automaticFlags.flags.all = 0;
  packetUBXRXMPMP->callbackPointerPtr = NULL;
  packetUBXRXMPMP->callbackData = NULL;
  return (true);
}

// Callback receives a pointer to the data, instead of _all_ the data. Much kinder on the stack!
bool SFE_UBLOX_GNSS::setRXMPMPmessageCallbackPtr(void (*callbackPointer)(UBX_RXM_PMP_message_data_t *))
{
  if (packetUBXRXMPMPmessage == NULL)
    initPacketUBXRXMPMPmessage();     // Check that RAM has been allocated for the data
  if (packetUBXRXMPMPmessage == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXRXMPMPmessage->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMPMPmessage->callbackData = new UBX_RXM_PMP_message_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMPMPmessage->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoRXMPMPmessagecallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMPMPmessage->callbackPointerPtr = callbackPointer;
  return (true);
}

// PRIVATE: Allocate RAM for packetUBXRXMPMPmessage and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXRXMPMPmessage()
{
  packetUBXRXMPMPmessage = new UBX_RXM_PMP_message_t; // Allocate RAM for the main struct
  if (packetUBXRXMPMPmessage == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXRXMPMPmessage: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMPMPmessage->automaticFlags.flags.all = 0;
  packetUBXRXMPMPmessage->callbackPointerPtr = NULL;
  packetUBXRXMPMPmessage->callbackData = NULL;
  return (true);
}

// ***** RXM QZSSL6 automatic support

// Callback receives a pointer to the data, instead of _all_ the data. Much kinder on the stack!
bool SFE_UBLOX_GNSS::setRXMQZSSL6messageCallbackPtr(void (*callbackPointer)(UBX_RXM_QZSSL6_message_data_t *))
{
  if (packetUBXRXMQZSSL6message == NULL)
    initPacketUBXRXMQZSSL6message();     // Check that RAM has been allocated for the data
  if (packetUBXRXMQZSSL6message == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXRXMQZSSL6message->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMQZSSL6message->callbackData = new UBX_RXM_QZSSL6_message_data_t[UBX_RXM_QZSSL6_NUM_CHANNELS]; // Allocate RAM for the main struct
  }

  if (packetUBXRXMQZSSL6message->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoRXMQZSSL6messagecallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMQZSSL6message->callbackPointerPtr = callbackPointer;
  return (true);
}

// PRIVATE: Allocate RAM for packetUBXRXMQZSSL6message and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXRXMQZSSL6message()
{
  packetUBXRXMQZSSL6message = new UBX_RXM_QZSSL6_message_t; // Allocate RAM for the main struct
  if (packetUBXRXMQZSSL6message == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXRXMQZSSL6message: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMQZSSL6message->automaticFlags.flags.all = 0;
  packetUBXRXMQZSSL6message->callbackPointerPtr = NULL;
  packetUBXRXMQZSSL6message->callbackData = NULL;
  return (true);
}

bool SFE_UBLOX_GNSS::setRXMCORcallbackPtr(void (*callbackPointer)(UBX_RXM_COR_data_t *))
{
  if (packetUBXRXMCOR == NULL)
    initPacketUBXRXMCOR();     // Check that RAM has been allocated for the data
  if (packetUBXRXMCOR == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXRXMCOR->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMCOR->callbackData = new UBX_RXM_COR_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMCOR->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoRXMCORcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMCOR->callbackPointerPtr = callbackPointer;
  return (true);
}

// PRIVATE: Allocate RAM for packetUBXRXMCOR and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXRXMCOR()
{
  packetUBXRXMCOR = new UBX_RXM_COR_t; // Allocate RAM for the main struct
  if (packetUBXRXMCOR == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXRXMCOR: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMCOR->automaticFlags.flags.all = 0;
  packetUBXRXMCOR->callbackPointerPtr = NULL;
  packetUBXRXMCOR->callbackData = NULL;
  return (true);
}

// ***** RXM SFRBX automatic support

bool SFE_UBLOX_GNSS::getRXMSFRBX(uint16_t maxWait)
{
  if (packetUBXRXMSFRBX == NULL)
    initPacketUBXRXMSFRBX();     // Check that RAM has been allocated for the SFRBX data
  if (packetUBXRXMSFRBX == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXRXMSFRBX->automaticFlags.flags.bits.automatic && packetUBXRXMSFRBX->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_RXM, UBX_RXM_SFRBX);
    return packetUBXRXMSFRBX->moduleQueried;
  }
  else if (packetUBXRXMSFRBX->automaticFlags.flags.bits.automatic && !packetUBXRXMSFRBX->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // SFRBX is output-only. It cannot be polled...
    // Strictly, getRXMSFRBX should be deprecated. But, to keep the library backward compatible, return(false) here.
    // See issue #167 for details
    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMSFRBX
// works.
bool SFE_UBLOX_GNSS::setAutoRXMSFRBX(bool enable, uint16_t maxWait)
{
  return setAutoRXMSFRBXrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMSFRBX
// works.
bool SFE_UBLOX_GNSS::setAutoRXMSFRBX(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoRXMSFRBXrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMSFRBX
// works.
bool SFE_UBLOX_GNSS::setAutoRXMSFRBXrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXRXMSFRBX == NULL)
    initPacketUBXRXMSFRBX();     // Check that RAM has been allocated for the data
  if (packetUBXRXMSFRBX == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_RXM;
  payloadCfg[1] = UBX_RXM_SFRBX;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXRXMSFRBX->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXRXMSFRBX->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXRXMSFRBX->moduleQueried = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoRXMSFRBXcallback(void (*callbackPointer)(UBX_RXM_SFRBX_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoRXMSFRBX(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXRXMSFRBX->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMSFRBX->callbackData = new UBX_RXM_SFRBX_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMSFRBX->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoRXMSFRBXcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMSFRBX->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoRXMSFRBXcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_SFRBX_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoRXMSFRBX(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXRXMSFRBX->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMSFRBX->callbackData = new UBX_RXM_SFRBX_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMSFRBX->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoRXMSFRBXcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMSFRBX->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and SFRBX is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoRXMSFRBX(bool enabled, bool implicitUpdate)
{
  if (packetUBXRXMSFRBX == NULL)
    initPacketUBXRXMSFRBX();     // Check that RAM has been allocated for the data
  if (packetUBXRXMSFRBX == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXRXMSFRBX->automaticFlags.flags.bits.automatic != enabled || packetUBXRXMSFRBX->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXRXMSFRBX->automaticFlags.flags.bits.automatic = enabled;
    packetUBXRXMSFRBX->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXRXMSFRBX and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXRXMSFRBX()
{
  packetUBXRXMSFRBX = new UBX_RXM_SFRBX_t; // Allocate RAM for the main struct
  if (packetUBXRXMSFRBX == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXRXMSFRBX: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMSFRBX->automaticFlags.flags.all = 0;
  packetUBXRXMSFRBX->callbackPointer = NULL;
  packetUBXRXMSFRBX->callbackPointerPtr = NULL;
  packetUBXRXMSFRBX->callbackData = NULL;
  packetUBXRXMSFRBX->moduleQueried = false;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushRXMSFRBX()
{
  if (packetUBXRXMSFRBX == NULL)
    return;                                 // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXRXMSFRBX->moduleQueried = false; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logRXMSFRBX(bool enabled)
{
  if (packetUBXRXMSFRBX == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXRXMSFRBX->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** RXM RAWX automatic support

bool SFE_UBLOX_GNSS::getRXMRAWX(uint16_t maxWait)
{
  if (packetUBXRXMRAWX == NULL)
    initPacketUBXRXMRAWX();     // Check that RAM has been allocated for the RAWX data
  if (packetUBXRXMRAWX == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXRXMRAWX->automaticFlags.flags.bits.automatic && packetUBXRXMRAWX->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_RXM, UBX_RXM_RAWX);
    return packetUBXRXMRAWX->moduleQueried;
  }
  else if (packetUBXRXMRAWX->automaticFlags.flags.bits.automatic && !packetUBXRXMRAWX->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_RXM;
    packetCfg.id = UBX_RXM_RAWX;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMRAWX
// works.
bool SFE_UBLOX_GNSS::setAutoRXMRAWX(bool enable, uint16_t maxWait)
{
  return setAutoRXMRAWXrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMRAWX
// works.
bool SFE_UBLOX_GNSS::setAutoRXMRAWX(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoRXMRAWXrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getRXMRAWX
// works.
bool SFE_UBLOX_GNSS::setAutoRXMRAWXrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXRXMRAWX == NULL)
    initPacketUBXRXMRAWX();     // Check that RAM has been allocated for the data
  if (packetUBXRXMRAWX == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_RXM;
  payloadCfg[1] = UBX_RXM_RAWX;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXRXMRAWX->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXRXMRAWX->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXRXMRAWX->moduleQueried = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoRXMRAWXcallback(void (*callbackPointer)(UBX_RXM_RAWX_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoRXMRAWX(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXRXMRAWX->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMRAWX->callbackData = new UBX_RXM_RAWX_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMRAWX->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoRXMRAWXcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMRAWX->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoRXMRAWXcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_RAWX_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoRXMRAWX(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXRXMRAWX->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXRXMRAWX->callbackData = new UBX_RXM_RAWX_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXRXMRAWX->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoRXMRAWXcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXRXMRAWX->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and VELNED is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoRXMRAWX(bool enabled, bool implicitUpdate)
{
  if (packetUBXRXMRAWX == NULL)
    initPacketUBXRXMRAWX();     // Check that RAM has been allocated for the data
  if (packetUBXRXMRAWX == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXRXMRAWX->automaticFlags.flags.bits.automatic != enabled || packetUBXRXMRAWX->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXRXMRAWX->automaticFlags.flags.bits.automatic = enabled;
    packetUBXRXMRAWX->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXRXMRAWX and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXRXMRAWX()
{
  packetUBXRXMRAWX = new UBX_RXM_RAWX_t; // Allocate RAM for the main struct
  if (packetUBXRXMRAWX == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXRXMRAWX: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXRXMRAWX->automaticFlags.flags.all = 0;
  packetUBXRXMRAWX->callbackPointer = NULL;
  packetUBXRXMRAWX->callbackPointerPtr = NULL;
  packetUBXRXMRAWX->callbackData = NULL;
  packetUBXRXMRAWX->moduleQueried = false;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushRXMRAWX()
{
  if (packetUBXRXMRAWX == NULL)
    return;                                // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXRXMRAWX->moduleQueried = false; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logRXMRAWX(bool enabled)
{
  if (packetUBXRXMRAWX == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXRXMRAWX->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** CFG automatic support

// Get the latest CFG PRT - as used by isConnected
//  Here's the dilemma:
//  The NEO-D9S doesn't support NAV-RATE so, if we want to include the D9 without creating a special class for it,
//  we need to use something else as the 'isConnected' test. The D9 does support CFG-PRT so we'll use that.
//  BUT many users could already be using getPortSettings and expecting the settings to be returned in packetCfg.
//  So, for isConnected ONLY, we need to enable auto support for CFG-PRT and then disable it afterwards so the settings
//  go back to being returned in packetCfg... What a tangled web we weave...!
bool SFE_UBLOX_GNSS::getPortSettingsInternal(uint8_t portID, uint16_t maxWait)
{
  if (packetUBXCFGPRT == NULL)
    initPacketUBXCFGPRT();     // Check that RAM has been allocated for the data
  if (packetUBXCFGPRT == NULL) // Bail if the RAM allocation failed
    return (false);

  // The CFG PRT message will never be produced automatically - that would be pointless.
  // There is no setAutoCFGPRT function. We always need to poll explicitly.
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 1;
  packetCfg.startingSpot = 0;

  payloadCfg[0] = portID;

  // The data is parsed as part of processing the response
  sfe_ublox_status_e result = sendCommand(&packetCfg, maxWait);
  bool retVal = false;

  if (result == SFE_UBLOX_STATUS_DATA_RECEIVED)
    retVal = true;

  if (result == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    retVal = true;

  // Now disable automatic support for CFG-PRT (see above)
  delete packetUBXCFGPRT;
  packetUBXCFGPRT = NULL;

  return (retVal);
}

// PRIVATE: Allocate RAM for packetUBXCFGPRT and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXCFGPRT()
{
  packetUBXCFGPRT = new UBX_CFG_PRT_t; // Allocate RAM for the main struct
  if (packetUBXCFGPRT == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXCFGPRT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXCFGPRT->dataValid = false;
  return (true);
}

// Get the latest CFG RATE
bool SFE_UBLOX_GNSS::getNavigationFrequencyInternal(uint16_t maxWait)
{
  if (packetUBXCFGRATE == NULL)
    initPacketUBXCFGRATE();     // Check that RAM has been allocated for the data
  if (packetUBXCFGRATE == NULL) // Bail if the RAM allocation failed
    return (false);

  // The CFG RATE message will never be produced automatically - that would be pointless.
  // There is no setAutoCFGRATE function. We always need to poll explicitly.
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RATE;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // The data is parsed as part of processing the response
  sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

  if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (true);

  if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    return (true);

  return (false);
}

// PRIVATE: Allocate RAM for packetUBXCFGRATE and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXCFGRATE()
{
  packetUBXCFGRATE = new UBX_CFG_RATE_t; // Allocate RAM for the main struct
  if (packetUBXCFGRATE == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXCFGRATE: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXCFGRATE->automaticFlags.flags.all = 0;        // Redundant
  packetUBXCFGRATE->moduleQueried.moduleQueried.all = 0; // Mark all data as stale/read
  return (true);
}

// ***** TIM TM2 automatic support

bool SFE_UBLOX_GNSS::getTIMTM2(uint16_t maxWait)
{
  if (packetUBXTIMTM2 == NULL)
    initPacketUBXTIMTM2();     // Check that RAM has been allocated for the TM2 data
  if (packetUBXTIMTM2 == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXTIMTM2->automaticFlags.flags.bits.automatic && packetUBXTIMTM2->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    checkUbloxInternal(&packetCfg, UBX_CLASS_TIM, UBX_TIM_TM2);
    return packetUBXTIMTM2->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXTIMTM2->automaticFlags.flags.bits.automatic && !packetUBXTIMTM2->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    return (false);
  }
  else
  {
    // The GPS is not automatically reporting navigation position so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_TIM;
    packetCfg.id = UBX_TIM_TM2;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      return (true);
    }

    return (false);
  }
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMTM2
// works.
bool SFE_UBLOX_GNSS::setAutoTIMTM2(bool enable, uint16_t maxWait)
{
  return setAutoTIMTM2rate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMTM2
// works.
bool SFE_UBLOX_GNSS::setAutoTIMTM2(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoTIMTM2rate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic navigation message generation by the GNSS. This changes the way getTIMTM2
// works.
bool SFE_UBLOX_GNSS::setAutoTIMTM2rate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXTIMTM2 == NULL)
    initPacketUBXTIMTM2();     // Check that RAM has been allocated for the data
  if (packetUBXTIMTM2 == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_TIM;
  payloadCfg[1] = UBX_TIM_TM2;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXTIMTM2->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXTIMTM2->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXTIMTM2->moduleQueried.moduleQueried.bits.all = false;
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoTIMTM2callback(void (*callbackPointer)(UBX_TIM_TM2_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoTIMTM2(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXTIMTM2->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXTIMTM2->callbackData = new UBX_TIM_TM2_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXTIMTM2->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoTIMTM2callback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXTIMTM2->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoTIMTM2callbackPtr(void (*callbackPointerPtr)(UBX_TIM_TM2_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoTIMTM2(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXTIMTM2->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXTIMTM2->callbackData = new UBX_TIM_TM2_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXTIMTM2->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoTIMTM2callbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXTIMTM2->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and VELNED is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoTIMTM2(bool enabled, bool implicitUpdate)
{
  if (packetUBXTIMTM2 == NULL)
    initPacketUBXTIMTM2();     // Check that RAM has been allocated for the data
  if (packetUBXTIMTM2 == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXTIMTM2->automaticFlags.flags.bits.automatic != enabled || packetUBXTIMTM2->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXTIMTM2->automaticFlags.flags.bits.automatic = enabled;
    packetUBXTIMTM2->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXTIMTM2 and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXTIMTM2()
{
  packetUBXTIMTM2 = new UBX_TIM_TM2_t; // Allocate RAM for the main struct
  if (packetUBXTIMTM2 == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXTIMTM2: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXTIMTM2->automaticFlags.flags.all = 0;
  packetUBXTIMTM2->callbackPointer = NULL;
  packetUBXTIMTM2->callbackPointerPtr = NULL;
  packetUBXTIMTM2->callbackData = NULL;
  packetUBXTIMTM2->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushTIMTM2()
{
  if (packetUBXTIMTM2 == NULL)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXTIMTM2->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logTIMTM2(bool enabled)
{
  if (packetUBXTIMTM2 == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXTIMTM2->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** ESF ALG automatic support

bool SFE_UBLOX_GNSS::getEsfAlignment(uint16_t maxWait)
{
  return (getESFALG(maxWait));
}

bool SFE_UBLOX_GNSS::getESFALG(uint16_t maxWait)
{
  if (packetUBXESFALG == NULL)
    initPacketUBXESFALG();     // Check that RAM has been allocated for the ESF alignment data
  if (packetUBXESFALG == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXESFALG->automaticFlags.flags.bits.automatic && packetUBXESFALG->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getEsfAlignment: Autoreporting"));
    //  }
    checkUbloxInternal(&packetCfg, UBX_CLASS_ESF, UBX_ESF_ALG);
    return packetUBXESFALG->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXESFALG->automaticFlags.flags.bits.automatic && !packetUBXESFALG->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getEsfAlignment: Exit immediately"));
    //  }
    return (false);
  }
  else
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getEsfAlignment: Polling"));
    // }

    // The GPS is not automatically reporting HNR PVT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_ESF;
    packetCfg.id = UBX_ESF_ALG;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getEsfAlignment: data in packetCfg was OVERWRITTEN by another message (but that's OK)"));
      // }
      return (true);
    }

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("getEsfAlignment retVal: "));
    //   _debugSerial->println(statusString(retVal));
    // }
    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic ESF ALG message generation by the GNSS. This changes the way getEsfAlignment
// works.
bool SFE_UBLOX_GNSS::setAutoESFALG(bool enable, uint16_t maxWait)
{
  return setAutoESFALGrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic ESF ALG message generation by the GNSS. This changes the way getEsfAlignment
// works.
bool SFE_UBLOX_GNSS::setAutoESFALG(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoESFALGrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic ESF ALG message generation by the GNSS. This changes the way getEsfAlignment
// works.
bool SFE_UBLOX_GNSS::setAutoESFALGrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXESFALG == NULL)
    initPacketUBXESFALG();     // Check that RAM has been allocated for the data
  if (packetUBXESFALG == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_ESF;
  payloadCfg[1] = UBX_ESF_ALG;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXESFALG->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXESFALG->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXESFALG->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoESFALGcallback(void (*callbackPointer)(UBX_ESF_ALG_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFALG(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFALG->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFALG->callbackData = new UBX_ESF_ALG_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFALG->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoESFALGcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFALG->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoESFALGcallbackPtr(void (*callbackPointerPtr)(UBX_ESF_ALG_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFALG(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFALG->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFALG->callbackData = new UBX_ESF_ALG_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFALG->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoESFALGcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFALG->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ESF ALG is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoESFALG(bool enabled, bool implicitUpdate)
{
  if (packetUBXESFALG == NULL)
    initPacketUBXESFALG();     // Check that RAM has been allocated for the ESF alignment data
  if (packetUBXESFALG == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXESFALG->automaticFlags.flags.bits.automatic != enabled || packetUBXESFALG->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXESFALG->automaticFlags.flags.bits.automatic = enabled;
    packetUBXESFALG->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXESFALG and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXESFALG()
{
  packetUBXESFALG = new UBX_ESF_ALG_t; // Allocate RAM for the main struct
  if (packetUBXESFALG == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXESFALG: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXESFALG->automaticFlags.flags.all = 0;
  packetUBXESFALG->callbackPointer = NULL;
  packetUBXESFALG->callbackPointerPtr = NULL;
  packetUBXESFALG->callbackData = NULL;
  packetUBXESFALG->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushESFALG()
{
  if (packetUBXESFALG == NULL)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFALG->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logESFALG(bool enabled)
{
  if (packetUBXESFALG == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFALG->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** ESF STATUS automatic support

bool SFE_UBLOX_GNSS::getEsfInfo(uint16_t maxWait)
{
  return (getESFSTATUS(maxWait));
}

bool SFE_UBLOX_GNSS::getESFSTATUS(uint16_t maxWait)
{
  if (packetUBXESFSTATUS == NULL)
    initPacketUBXESFSTATUS();     // Check that RAM has been allocated for the ESF status data
  if (packetUBXESFSTATUS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXESFSTATUS->automaticFlags.flags.bits.automatic && packetUBXESFSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getEsfInfo: Autoreporting"));
    //  }
    checkUbloxInternal(&packetCfg, UBX_CLASS_ESF, UBX_ESF_STATUS);
    return packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXESFSTATUS->automaticFlags.flags.bits.automatic && !packetUBXESFSTATUS->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getEsfInfo: Exit immediately"));
    //  }
    return (false);
  }
  else
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getEsfInfo: Polling"));
    // }

    // The GPS is not automatically reporting HNR PVT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_ESF;
    packetCfg.id = UBX_ESF_STATUS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getEsfInfo: data in packetCfg was OVERWRITTEN by another message (but that's OK)"));
      // }
      return (true);
    }

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("getEsfInfo retVal: "));
    //   _debugSerial->println(statusString(retVal));
    // }
    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic ESF STATUS message generation by the GNSS. This changes the way getESFInfo
// works.
bool SFE_UBLOX_GNSS::setAutoESFSTATUS(bool enable, uint16_t maxWait)
{
  return setAutoESFSTATUSrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic ESF STATUS message generation by the GNSS. This changes the way getESFInfo
// works.
bool SFE_UBLOX_GNSS::setAutoESFSTATUS(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoESFSTATUSrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic ESF STATUS message generation by the GNSS. This changes the way getESFInfo
// works.
bool SFE_UBLOX_GNSS::setAutoESFSTATUSrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXESFSTATUS == NULL)
    initPacketUBXESFSTATUS();     // Check that RAM has been allocated for the data
  if (packetUBXESFSTATUS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_ESF;
  payloadCfg[1] = UBX_ESF_STATUS;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXESFSTATUS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXESFSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoESFSTATUScallback(void (*callbackPointer)(UBX_ESF_STATUS_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFSTATUS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFSTATUS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFSTATUS->callbackData = new UBX_ESF_STATUS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFSTATUS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoESFSTATUScallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFSTATUS->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoESFSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_STATUS_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFSTATUS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFSTATUS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFSTATUS->callbackData = new UBX_ESF_STATUS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFSTATUS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoESFSTATUScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFSTATUS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ESF STATUS is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoESFSTATUS(bool enabled, bool implicitUpdate)
{
  if (packetUBXESFSTATUS == NULL)
    initPacketUBXESFSTATUS();     // Check that RAM has been allocated for the ESF status data
  if (packetUBXESFSTATUS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXESFSTATUS->automaticFlags.flags.bits.automatic != enabled || packetUBXESFSTATUS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXESFSTATUS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXESFSTATUS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXESFSTATUS and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXESFSTATUS()
{
  packetUBXESFSTATUS = new UBX_ESF_STATUS_t; // Allocate RAM for the main struct

  if (packetUBXESFSTATUS == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXESFSTATUS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXESFSTATUS->automaticFlags.flags.all = 0;
  packetUBXESFSTATUS->callbackPointer = NULL;
  packetUBXESFSTATUS->callbackPointerPtr = NULL;
  packetUBXESFSTATUS->callbackData = NULL;
  packetUBXESFSTATUS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushESFSTATUS()
{
  if (packetUBXESFSTATUS == NULL)
    return;                                                // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFSTATUS->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logESFSTATUS(bool enabled)
{
  if (packetUBXESFSTATUS == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFSTATUS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** ESF INS automatic support

bool SFE_UBLOX_GNSS::getEsfIns(uint16_t maxWait)
{
  return (getESFINS(maxWait));
}

bool SFE_UBLOX_GNSS::getESFINS(uint16_t maxWait)
{
  if (packetUBXESFINS == NULL)
    initPacketUBXESFINS();     // Check that RAM has been allocated for the ESF INS data
  if (packetUBXESFINS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXESFINS->automaticFlags.flags.bits.automatic && packetUBXESFINS->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getEsfIns: Autoreporting"));
    //  }
    checkUbloxInternal(&packetCfg, UBX_CLASS_ESF, UBX_ESF_INS);
    return packetUBXESFINS->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXESFINS->automaticFlags.flags.bits.automatic && !packetUBXESFINS->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getEsfIns: Exit immediately"));
    //  }
    return (false);
  }
  else
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getEsfIns: Polling"));
    // }

    // The GPS is not automatically reporting HNR PVT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_ESF;
    packetCfg.id = UBX_ESF_INS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getEsfIns: data in packetCfg was OVERWRITTEN by another message (but that's OK)"));
      // }
      return (true);
    }

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("getEsfIns retVal: "));
    //   _debugSerial->println(statusString(retVal));
    // }
    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic ESF INS message generation by the GNSS. This changes the way getESFIns
// works.
bool SFE_UBLOX_GNSS::setAutoESFINS(bool enable, uint16_t maxWait)
{
  return setAutoESFINSrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic ESF INS message generation by the GNSS. This changes the way getESFIns
// works.
bool SFE_UBLOX_GNSS::setAutoESFINS(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoESFINSrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic ESF INS message generation by the GNSS. This changes the way getESFIns
// works.
bool SFE_UBLOX_GNSS::setAutoESFINSrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXESFINS == NULL)
    initPacketUBXESFINS();     // Check that RAM has been allocated for the data
  if (packetUBXESFINS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_ESF;
  payloadCfg[1] = UBX_ESF_INS;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXESFINS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXESFINS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXESFINS->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoESFINScallback(void (*callbackPointer)(UBX_ESF_INS_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFINS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFINS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFINS->callbackData = new UBX_ESF_INS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFINS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoESFINScallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFINS->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoESFINScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_INS_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFINS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFINS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFINS->callbackData = new UBX_ESF_INS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFINS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoESFINScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFINS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ESF INS is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoESFINS(bool enabled, bool implicitUpdate)
{
  if (packetUBXESFINS == NULL)
    initPacketUBXESFINS();     // Check that RAM has been allocated for the ESF INS data
  if (packetUBXESFINS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXESFINS->automaticFlags.flags.bits.automatic != enabled || packetUBXESFINS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXESFINS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXESFINS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXESFINS and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXESFINS()
{
  packetUBXESFINS = new UBX_ESF_INS_t; // Allocate RAM for the main struct
  if (packetUBXESFINS == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXESFINS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXESFINS->automaticFlags.flags.all = 0;
  packetUBXESFINS->callbackPointer = NULL;
  packetUBXESFINS->callbackPointerPtr = NULL;
  packetUBXESFINS->callbackData = NULL;
  packetUBXESFINS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushESFINS()
{
  if (packetUBXESFINS == NULL)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFINS->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logESFINS(bool enabled)
{
  if (packetUBXESFINS == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFINS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** ESF MEAS automatic support

// Enable or disable automatic ESF MEAS message generation by the GNSS
bool SFE_UBLOX_GNSS::setAutoESFMEAS(bool enable, uint16_t maxWait)
{
  return setAutoESFMEASrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic ESF MEAS message generation by the GNSS
bool SFE_UBLOX_GNSS::setAutoESFMEAS(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoESFMEASrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic ESF MEAS message generation by the GNSS
bool SFE_UBLOX_GNSS::setAutoESFMEASrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXESFMEAS == NULL)
    initPacketUBXESFMEAS();     // Check that RAM has been allocated for the data
  if (packetUBXESFMEAS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_ESF;
  payloadCfg[1] = UBX_ESF_MEAS;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXESFMEAS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXESFMEAS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoESFMEAScallback(void (*callbackPointer)(UBX_ESF_MEAS_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFMEAS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFMEAS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFMEAS->callbackData = new UBX_ESF_MEAS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFMEAS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoESFMEAScallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFMEAS->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoESFMEAScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_MEAS_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFMEAS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFMEAS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFMEAS->callbackData = new UBX_ESF_MEAS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFMEAS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoESFMEAScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFMEAS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ESF MEAS is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoESFMEAS(bool enabled, bool implicitUpdate)
{
  if (packetUBXESFMEAS == NULL)
    initPacketUBXESFMEAS();     // Check that RAM has been allocated for the ESF MEAS data
  if (packetUBXESFMEAS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXESFMEAS->automaticFlags.flags.bits.automatic != enabled || packetUBXESFMEAS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXESFMEAS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXESFMEAS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXESFMEAS and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXESFMEAS()
{
  packetUBXESFMEAS = new UBX_ESF_MEAS_t; // Allocate RAM for the main struct
  if (packetUBXESFMEAS == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXESFMEAS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXESFMEAS->automaticFlags.flags.all = 0;
  packetUBXESFMEAS->callbackPointer = NULL;
  packetUBXESFMEAS->callbackPointerPtr = NULL;
  packetUBXESFMEAS->callbackData = NULL;
  return (true);
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logESFMEAS(bool enabled)
{
  if (packetUBXESFMEAS == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFMEAS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** ESF RAW automatic support

// ESF RAW messages are output only. They cannot be polled.

// Enable or disable automatic ESF RAW message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoESFRAW(bool enable, uint16_t maxWait)
{
  return setAutoESFRAWrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic ESF RAW message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoESFRAW(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoESFRAWrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic ESF RAW message generation by the GNSS.
// Note: this function can only be used to enable or disable the messages. A rate of zero disables the messages.
// A rate of 1 or more causes the messages to be generated at the full 100Hz.
bool SFE_UBLOX_GNSS::setAutoESFRAWrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXESFRAW == NULL)
    initPacketUBXESFRAW();     // Check that RAM has been allocated for the data
  if (packetUBXESFRAW == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_ESF;
  payloadCfg[1] = UBX_ESF_RAW;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXESFRAW->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXESFRAW->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return ok;
}

// Enable automatic message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoESFRAWcallback(void (*callbackPointer)(UBX_ESF_RAW_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFRAW(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFRAW->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFRAW->callbackData = new UBX_ESF_RAW_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFRAW->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoESFRAWcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFRAW->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoESFRAWcallbackPtr(void (*callbackPointerPtr)(UBX_ESF_RAW_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoESFRAW(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXESFRAW->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXESFRAW->callbackData = new UBX_ESF_RAW_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXESFRAW->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoESFRAWcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXESFRAW->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and ESF RAW is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoESFRAW(bool enabled, bool implicitUpdate)
{
  if (packetUBXESFRAW == NULL)
    initPacketUBXESFRAW();     // Check that RAM has been allocated for the ESF RAW data
  if (packetUBXESFRAW == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXESFRAW->automaticFlags.flags.bits.automatic != enabled || packetUBXESFRAW->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXESFRAW->automaticFlags.flags.bits.automatic = enabled;
    packetUBXESFRAW->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXESFRAW and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXESFRAW()
{
  packetUBXESFRAW = new UBX_ESF_RAW_t; // Allocate RAM for the main struct
  if (packetUBXESFRAW == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXESFRAW: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXESFRAW->automaticFlags.flags.all = 0;
  packetUBXESFRAW->callbackPointer = NULL;
  packetUBXESFRAW->callbackPointerPtr = NULL;
  packetUBXESFRAW->callbackData = NULL;
  return (true);
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logESFRAW(bool enabled)
{
  if (packetUBXESFRAW == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXESFRAW->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** HNR ATT automatic support

bool SFE_UBLOX_GNSS::getHNRAtt(uint16_t maxWait)
{
  return (getHNRATT(maxWait));
}

// Get the HNR Attitude data
//  Returns true if the get HNR attitude is successful. Data is returned in hnrAtt
//  Note: if hnrAttQueried is true, it gets set to false by this function since we assume
//        that the user will read hnrAtt immediately after this. I.e. this function will
//        only return true _once_ after each auto HNR Att is processed
bool SFE_UBLOX_GNSS::getHNRATT(uint16_t maxWait)
{
  if (packetUBXHNRATT == NULL)
    initPacketUBXHNRATT();     // Check that RAM has been allocated for the data
  if (packetUBXHNRATT == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXHNRATT->automaticFlags.flags.bits.automatic && packetUBXHNRATT->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getHNRAtt: Autoreporting"));
    //  }
    checkUbloxInternal(&packetCfg, UBX_CLASS_HNR, UBX_HNR_ATT);
    return packetUBXHNRATT->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXHNRATT->automaticFlags.flags.bits.automatic && !packetUBXHNRATT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getHNRAtt: Exit immediately"));
    //  }
    return (false);
  }
  else
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getHNRAtt: Polling"));
    // }

    // The GPS is not automatically reporting HNR attitude so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_HNR;
    packetCfg.id = UBX_HNR_ATT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getHNRAtt: data in packetCfg was OVERWRITTEN by another message (but that's OK)"));
      // }
      return (true);
    }

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("getHNRAtt retVal: "));
    //   _debugSerial->println(statusString(retVal));
    // }
    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic HNR attitude message generation by the GNSS. This changes the way getHNRAtt
// works.
bool SFE_UBLOX_GNSS::setAutoHNRATT(bool enable, uint16_t maxWait)
{
  return setAutoHNRATTrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic HNR attitude message generation by the GNSS. This changes the way getHNRAtt
// works.
bool SFE_UBLOX_GNSS::setAutoHNRATT(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoHNRATTrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic HNR attitude message generation by the GNSS. This changes the way getHNRAtt
// works.
bool SFE_UBLOX_GNSS::setAutoHNRATTrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXHNRATT == NULL)
    initPacketUBXHNRATT();     // Check that RAM has been allocated for the data
  if (packetUBXHNRATT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_HNR;
  payloadCfg[1] = UBX_HNR_ATT;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXHNRATT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXHNRATT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoHNRATTcallback(void (*callbackPointer)(UBX_HNR_ATT_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHNRATT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXHNRATT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXHNRATT->callbackData = new UBX_HNR_ATT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXHNRATT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoHNRAttcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXHNRATT->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoHNRATTcallbackPtr(void (*callbackPointerPtr)(UBX_HNR_ATT_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHNRATT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXHNRATT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXHNRATT->callbackData = new UBX_HNR_ATT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXHNRATT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoHNRAttcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXHNRATT->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and HNR attitude is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoHNRATT(bool enabled, bool implicitUpdate)
{
  if (packetUBXHNRATT == NULL)
    initPacketUBXHNRATT();     // Check that RAM has been allocated for the data
  if (packetUBXHNRATT == NULL) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXHNRATT->automaticFlags.flags.bits.automatic != enabled || packetUBXHNRATT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXHNRATT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXHNRATT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXHNRATT and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXHNRATT()
{
  packetUBXHNRATT = new UBX_HNR_ATT_t; // Allocate RAM for the main struct
  if (packetUBXHNRATT == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXHNRATT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXHNRATT->automaticFlags.flags.all = 0;
  packetUBXHNRATT->callbackPointer = NULL;
  packetUBXHNRATT->callbackPointerPtr = NULL;
  packetUBXHNRATT->callbackData = NULL;
  packetUBXHNRATT->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushHNRATT()
{
  if (packetUBXHNRATT == NULL)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRATT->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logHNRATT(bool enabled)
{
  if (packetUBXHNRATT == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRATT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** HNR DYN automatic support

bool SFE_UBLOX_GNSS::getHNRDyn(uint16_t maxWait)
{
  return (getHNRINS(maxWait));
}

// Get the HNR vehicle dynamics data
//  Returns true if the get HNR vehicle dynamics is successful. Data is returned in hnrVehDyn
//  Note: if hnrDynQueried is true, it gets set to false by this function since we assume
//        that the user will read hnrVehDyn immediately after this. I.e. this function will
//        only return true _once_ after each auto HNR Dyn is processed
bool SFE_UBLOX_GNSS::getHNRINS(uint16_t maxWait)
{
  if (packetUBXHNRINS == NULL)
    initPacketUBXHNRINS();     // Check that RAM has been allocated for the data
  if (packetUBXHNRINS == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXHNRINS->automaticFlags.flags.bits.automatic && packetUBXHNRINS->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getHNRINS: Autoreporting"));
    //  }
    checkUbloxInternal(&packetCfg, UBX_CLASS_HNR, UBX_HNR_INS);
    return packetUBXHNRINS->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXHNRINS->automaticFlags.flags.bits.automatic && !packetUBXHNRINS->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getHNRINS: Exit immediately"));
    //  }
    return (false);
  }
  else
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getHNRINS: Polling"));
    // }

    // The GPS is not automatically reporting HNR vehicle dynamics so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_HNR;
    packetCfg.id = UBX_HNR_INS;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getHNRINS: data in packetCfg was OVERWRITTEN by another message (but that's OK)"));
      // }
      return (true);
    }

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("getHNRINS retVal: "));
    //   _debugSerial->println(statusString(retVal));
    // }
    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic HNR vehicle dynamics message generation by the GNSS. This changes the way getHNRINS
// works.
bool SFE_UBLOX_GNSS::setAutoHNRINS(bool enable, uint16_t maxWait)
{
  return setAutoHNRINSrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic HNR vehicle dynamics message generation by the GNSS. This changes the way getHNRINS
// works.
bool SFE_UBLOX_GNSS::setAutoHNRINS(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoHNRINSrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic HNR vehicle dynamics message generation by the GNSS. This changes the way getHNRINS
// works.
bool SFE_UBLOX_GNSS::setAutoHNRINSrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXHNRINS == NULL)
    initPacketUBXHNRINS();     // Check that RAM has been allocated for the data
  if (packetUBXHNRINS == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_HNR;
  payloadCfg[1] = UBX_HNR_INS;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXHNRINS->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXHNRINS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXHNRINS->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoHNRINScallback(void (*callbackPointer)(UBX_HNR_INS_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHNRINS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXHNRINS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXHNRINS->callbackData = new UBX_HNR_INS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXHNRINS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoHNRINScallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXHNRINS->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoHNRINScallbackPtr(void (*callbackPointerPtr)(UBX_HNR_INS_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHNRINS(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXHNRINS->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXHNRINS->callbackData = new UBX_HNR_INS_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXHNRINS->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoHNRINScallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXHNRINS->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and HNR vehicle dynamics is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoHNRINS(bool enabled, bool implicitUpdate)
{
  if (packetUBXHNRINS == NULL)
    initPacketUBXHNRINS();     // Check that RAM has been allocated for the data
  if (packetUBXHNRINS == NULL) // Bail if the RAM allocation failed
    return (false);

  bool changes = packetUBXHNRINS->automaticFlags.flags.bits.automatic != enabled || packetUBXHNRINS->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXHNRINS->automaticFlags.flags.bits.automatic = enabled;
    packetUBXHNRINS->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXHNRINS and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXHNRINS()
{
  packetUBXHNRINS = new UBX_HNR_INS_t; // Allocate RAM for the main struct
  if (packetUBXHNRINS == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXHNRINS: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXHNRINS->automaticFlags.flags.all = 0;
  packetUBXHNRINS->callbackPointer = NULL;
  packetUBXHNRINS->callbackPointerPtr = NULL;
  packetUBXHNRINS->callbackData = NULL;
  packetUBXHNRINS->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushHNRINS()
{
  if (packetUBXHNRINS == NULL)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRINS->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logHNRINS(bool enabled)
{
  if (packetUBXHNRINS == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRINS->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** HNR PVT automatic support

// Get the HNR PVT data
//  Returns true if the get HNR PVT is successful. Data is returned in hnrPVT
//  Note: if hnrPVTQueried is true, it gets set to false by this function since we assume
//        that the user will read hnrPVT immediately after this. I.e. this function will
//        only return true _once_ after each auto HNR PVT is processed
bool SFE_UBLOX_GNSS::getHNRPVT(uint16_t maxWait)
{
  if (packetUBXHNRPVT == NULL)
    initPacketUBXHNRPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXHNRPVT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (packetUBXHNRPVT->automaticFlags.flags.bits.automatic && packetUBXHNRPVT->automaticFlags.flags.bits.implicitUpdate)
  {
    // The GPS is automatically reporting, we just check whether we got unread data
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getHNRPVT: Autoreporting"));
    //  }
    checkUbloxInternal(&packetCfg, UBX_CLASS_HNR, UBX_HNR_PVT);
    return packetUBXHNRPVT->moduleQueried.moduleQueried.bits.all;
  }
  else if (packetUBXHNRPVT->automaticFlags.flags.bits.automatic && !packetUBXHNRPVT->automaticFlags.flags.bits.implicitUpdate)
  {
    // Someone else has to call checkUblox for us...
    //  if (_printDebug == true)
    //  {
    //    _debugSerial->println(F("getHNRPVT: Exit immediately"));
    //  }
    return (false);
  }
  else
  {
    // if (_printDebug == true)
    // {
    //   _debugSerial->println(F("getHNRPVT: Polling"));
    // }

    // The GPS is not automatically reporting HNR PVT so we have to poll explicitly
    packetCfg.cls = UBX_CLASS_HNR;
    packetCfg.id = UBX_HNR_PVT;
    packetCfg.len = 0;
    packetCfg.startingSpot = 0;

    // The data is parsed as part of processing the response
    sfe_ublox_status_e retVal = sendCommand(&packetCfg, maxWait);

    if (retVal == SFE_UBLOX_STATUS_DATA_RECEIVED)
      return (true);

    if (retVal == SFE_UBLOX_STATUS_DATA_OVERWRITTEN)
    {
      // if (_printDebug == true)
      // {
      //   _debugSerial->println(F("getHNRPVT: data in packetCfg was OVERWRITTEN by another message (but that's OK)"));
      // }
      return (true);
    }

    // if (_printDebug == true)
    // {
    //   _debugSerial->print(F("getHNRPVT retVal: "));
    //   _debugSerial->println(statusString(retVal));
    // }
    return (false);
  }

  return (false); // Trap. We should never get here...
}

// Enable or disable automatic HNR PVT message generation by the GNSS. This changes the way getHNRPVT
// works.
bool SFE_UBLOX_GNSS::setAutoHNRPVT(bool enable, uint16_t maxWait)
{
  return setAutoHNRPVTrate(enable ? 1 : 0, true, maxWait);
}

// Enable or disable automatic HNR PVT message generation by the GNSS. This changes the way getHNRPVT
// works.
bool SFE_UBLOX_GNSS::setAutoHNRPVT(bool enable, bool implicitUpdate, uint16_t maxWait)
{
  return setAutoHNRPVTrate(enable ? 1 : 0, implicitUpdate, maxWait);
}

// Enable or disable automatic HNR PVT message generation by the GNSS. This changes the way getHNRPVT
// works.
bool SFE_UBLOX_GNSS::setAutoHNRPVTrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait)
{
  if (packetUBXHNRPVT == NULL)
    initPacketUBXHNRPVT();     // Check that RAM has been allocated for the data
  if (packetUBXHNRPVT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  if (rate > 127)
    rate = 127;

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_MSG;
  packetCfg.len = 3;
  packetCfg.startingSpot = 0;
  payloadCfg[0] = UBX_CLASS_HNR;
  payloadCfg[1] = UBX_HNR_PVT;
  payloadCfg[2] = rate; // rate relative to navigation freq.

  bool ok = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
  if (ok)
  {
    packetUBXHNRPVT->automaticFlags.flags.bits.automatic = (rate > 0);
    packetUBXHNRPVT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  packetUBXHNRPVT->moduleQueried.moduleQueried.bits.all = false; // Mark data as stale
  return ok;
}

// Enable automatic navigation message generation by the GNSS.
bool SFE_UBLOX_GNSS::setAutoHNRPVTcallback(void (*callbackPointer)(UBX_HNR_PVT_data_t), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHNRPVT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXHNRPVT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXHNRPVT->callbackData = new UBX_HNR_PVT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXHNRPVT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoHNRPVTcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXHNRPVT->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setAutoHNRPVTcallbackPtr(void (*callbackPointerPtr)(UBX_HNR_PVT_data_t *), uint16_t maxWait)
{
  // Enable auto messages. Set implicitUpdate to false as we expect the user to call checkUblox manually.
  bool result = setAutoHNRPVT(true, false, maxWait);
  if (!result)
    return (result); // Bail if setAuto failed

  if (packetUBXHNRPVT->callbackData == NULL) // Check if RAM has been allocated for the callback copy
  {
    packetUBXHNRPVT->callbackData = new UBX_HNR_PVT_data_t; // Allocate RAM for the main struct
  }

  if (packetUBXHNRPVT->callbackData == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setAutoHNRPVTcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  packetUBXHNRPVT->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// In case no config access to the GNSS is possible and HNR PVT is send cyclically already
// set config to suitable parameters
bool SFE_UBLOX_GNSS::assumeAutoHNRPVT(bool enabled, bool implicitUpdate)
{
  if (packetUBXHNRPVT == NULL)
    initPacketUBXHNRPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXHNRPVT == NULL) // Only attempt this if RAM allocation was successful
    return false;

  bool changes = packetUBXHNRPVT->automaticFlags.flags.bits.automatic != enabled || packetUBXHNRPVT->automaticFlags.flags.bits.implicitUpdate != implicitUpdate;
  if (changes)
  {
    packetUBXHNRPVT->automaticFlags.flags.bits.automatic = enabled;
    packetUBXHNRPVT->automaticFlags.flags.bits.implicitUpdate = implicitUpdate;
  }
  return changes;
}

// PRIVATE: Allocate RAM for packetUBXHNRPVT and initialize it
bool SFE_UBLOX_GNSS::initPacketUBXHNRPVT()
{
  packetUBXHNRPVT = new UBX_HNR_PVT_t; // Allocate RAM for the main struct
  if (packetUBXHNRPVT == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initPacketUBXHNRPVT: RAM alloc failed!"));
#endif
    return (false);
  }
  packetUBXHNRPVT->automaticFlags.flags.all = 0;
  packetUBXHNRPVT->callbackPointer = NULL;
  packetUBXHNRPVT->callbackPointerPtr = NULL;
  packetUBXHNRPVT->callbackData = NULL;
  packetUBXHNRPVT->moduleQueried.moduleQueried.all = 0;
  return (true);
}

// Mark all the data as read/stale
void SFE_UBLOX_GNSS::flushHNRPVT()
{
  if (packetUBXHNRPVT == NULL)
    return;                                             // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRPVT->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// Log this data in file buffer
void SFE_UBLOX_GNSS::logHNRPVT(bool enabled)
{
  if (packetUBXHNRPVT == NULL)
    return; // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXHNRPVT->automaticFlags.flags.bits.addToFileBuffer = (uint8_t)enabled;
}

// ***** Helper Functions for NMEA Logging / Processing

// Set the mainTalkerId used by NMEA messages - allows all NMEA messages except GSV to be prefixed with GP instead of GN
bool SFE_UBLOX_GNSS::setMainTalkerID(sfe_ublox_talker_ids_e id, uint16_t maxWait)
{
  // Get the current extended NMEA protocol configuration (V1)
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_NMEA;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the current settings. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  payloadCfg[9] = (uint8_t)id;

  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Enable/Disable NMEA High Precision Mode - include extra decimal places in the Lat and Lon
bool SFE_UBLOX_GNSS::setHighPrecisionMode(bool enable, uint16_t maxWait)
{
  // Get the current extended NMEA protocol configuration (V1)
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_NMEA;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the current settings. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);

  if (enable)
  {
    payloadCfg[3] |= (1 << 3);               // Set the highPrec flag
    payloadCfg[3] &= ~((1 << 0) | (1 << 2)); // Clear the compat and limit82 flags
  }
  else
    payloadCfg[3] &= ~(1 << 3); // Clear the highPrec flag

  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  return (sendCommand(&packetCfg, maxWait) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}

// Log selected NMEA messages to file buffer - if the messages are enabled and if the file buffer exists
// User needs to call setFileBufferSize before .begin
void SFE_UBLOX_GNSS::setNMEALoggingMask(uint32_t messages)
{
  _logNMEA.all = messages;
}
uint32_t SFE_UBLOX_GNSS::getNMEALoggingMask()
{
  return (_logNMEA.all);
}

// Pass selected NMEA messages to processNMEA
void SFE_UBLOX_GNSS::setProcessNMEAMask(uint32_t messages)
{
  _processNMEA.all = messages;
}
uint32_t SFE_UBLOX_GNSS::getProcessNMEAMask()
{
  return (_processNMEA.all);
}

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
// Initiate automatic storage of NMEA GPGGA messages

// Get the most recent GPGGA message
// Return 0 if the message has not been received from the module
// Return 1 if the data is valid but has been read before
// Return 2 if the data is valid and is fresh/unread
uint8_t SFE_UBLOX_GNSS::getLatestNMEAGPGGA(NMEA_GGA_data_t *data)
{
  if (storageNMEAGPGGA == NULL)
    initStorageNMEAGPGGA();     // Check that RAM has been allocated for the message
  if (storageNMEAGPGGA == NULL) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Use a fake UBX class and ID.

  memcpy(data, &storageNMEAGPGGA->completeCopy, sizeof(NMEA_GGA_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGPGGA->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGPGGA->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGPGGA->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

// Enable a callback on the arrival of a GPGGA message
bool SFE_UBLOX_GNSS::setNMEAGPGGAcallback(void (*callbackPointer)(NMEA_GGA_data_t))
{
  if (storageNMEAGPGGA == NULL)
    initStorageNMEAGPGGA();     // Check that RAM has been allocated for the message
  if (storageNMEAGPGGA == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPGGA->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPGGA->callbackCopy = new NMEA_GGA_data_t;
  }

  if (storageNMEAGPGGA->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGPGGAcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPGGA->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setNMEAGPGGAcallbackPtr(void (*callbackPointerPtr)(NMEA_GGA_data_t *))
{
  if (storageNMEAGPGGA == NULL)
    initStorageNMEAGPGGA();     // Check that RAM has been allocated for the message
  if (storageNMEAGPGGA == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPGGA->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPGGA->callbackCopy = new NMEA_GGA_data_t;
  }

  if (storageNMEAGPGGA->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGPGGAcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPGGA->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GPGGA messages and initialize it
bool SFE_UBLOX_GNSS::initStorageNMEAGPGGA()
{
  storageNMEAGPGGA = new NMEA_GPGGA_t; // Allocate RAM for the main struct
  if (storageNMEAGPGGA == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initStorageNMEAGPGGA: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPGGA->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGPGGA->workingCopy.nmea, 0, NMEA_GGA_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGPGGA->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGPGGA->completeCopy.nmea, 0, NMEA_GGA_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGPGGA->callbackPointer = NULL;    // Clear the callback pointers
  storageNMEAGPGGA->callbackPointerPtr = NULL; // Clear the callback pointers
  storageNMEAGPGGA->callbackCopy = NULL;

  storageNMEAGPGGA->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

uint8_t SFE_UBLOX_GNSS::getLatestNMEAGNGGA(NMEA_GGA_data_t *data)
{
  if (storageNMEAGNGGA == NULL)
    initStorageNMEAGNGGA();     // Check that RAM has been allocated for the message
  if (storageNMEAGNGGA == NULL) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Use a fake UBX class and ID.

  memcpy(data, &storageNMEAGNGGA->completeCopy, sizeof(NMEA_GGA_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGNGGA->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGNGGA->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGNGGA->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

bool SFE_UBLOX_GNSS::setNMEAGNGGAcallback(void (*callbackPointer)(NMEA_GGA_data_t))
{
  if (storageNMEAGNGGA == NULL)
    initStorageNMEAGNGGA();     // Check that RAM has been allocated for the message
  if (storageNMEAGNGGA == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNGGA->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNGGA->callbackCopy = new NMEA_GGA_data_t;
  }

  if (storageNMEAGNGGA->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGNGGAcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNGGA->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setNMEAGNGGAcallbackPtr(void (*callbackPointerPtr)(NMEA_GGA_data_t *))
{
  if (storageNMEAGNGGA == NULL)
    initStorageNMEAGNGGA();     // Check that RAM has been allocated for the message
  if (storageNMEAGNGGA == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNGGA->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNGGA->callbackCopy = new NMEA_GGA_data_t;
  }

  if (storageNMEAGNGGA->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGNGGAcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNGGA->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GNGGA messages and initialize it
bool SFE_UBLOX_GNSS::initStorageNMEAGNGGA()
{
  storageNMEAGNGGA = new NMEA_GNGGA_t; // Allocate RAM for the main struct
  if (storageNMEAGNGGA == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initStorageNMEAGNGGA: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNGGA->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGNGGA->workingCopy.nmea, 0, NMEA_GGA_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGNGGA->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGNGGA->completeCopy.nmea, 0, NMEA_GGA_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGNGGA->callbackPointer = NULL;    // Clear the callback pointers
  storageNMEAGNGGA->callbackPointerPtr = NULL; // Clear the callback pointers
  storageNMEAGNGGA->callbackCopy = NULL;

  storageNMEAGNGGA->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

// Initiate automatic storage of NMEA GPVTG messages

// Get the most recent GPVTG message
// Return 0 if the message has not been received from the module
// Return 1 if the data is valid but has been read before
// Return 2 if the data is valid and is fresh/unread
uint8_t SFE_UBLOX_GNSS::getLatestNMEAGPVTG(NMEA_VTG_data_t *data)
{
  if (storageNMEAGPVTG == NULL)
    initStorageNMEAGPVTG();     // Check that RAM has been allocated for the message
  if (storageNMEAGPVTG == NULL) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Use a fake UBX class and ID.

  memcpy(data, &storageNMEAGPVTG->completeCopy, sizeof(NMEA_VTG_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGPVTG->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGPVTG->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGPVTG->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

// Enable a callback on the arrival of a GPVTG message
bool SFE_UBLOX_GNSS::setNMEAGPVTGcallback(void (*callbackPointer)(NMEA_VTG_data_t))
{
  if (storageNMEAGPVTG == NULL)
    initStorageNMEAGPVTG();     // Check that RAM has been allocated for the message
  if (storageNMEAGPVTG == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPVTG->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPVTG->callbackCopy = new NMEA_VTG_data_t;
  }

  if (storageNMEAGPVTG->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGPVTGcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPVTG->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setNMEAGPVTGcallbackPtr(void (*callbackPointerPtr)(NMEA_VTG_data_t *))
{
  if (storageNMEAGPVTG == NULL)
    initStorageNMEAGPVTG();     // Check that RAM has been allocated for the message
  if (storageNMEAGPVTG == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPVTG->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPVTG->callbackCopy = new NMEA_VTG_data_t;
  }

  if (storageNMEAGPVTG->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGPVTGcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPVTG->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GPVTG messages and initialize it
bool SFE_UBLOX_GNSS::initStorageNMEAGPVTG()
{
  storageNMEAGPVTG = new NMEA_GPVTG_t; // Allocate RAM for the main struct
  if (storageNMEAGPVTG == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initStorageNMEAGPVTG: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPVTG->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGPVTG->workingCopy.nmea, 0, NMEA_VTG_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGPVTG->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGPVTG->completeCopy.nmea, 0, NMEA_VTG_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGPVTG->callbackPointer = NULL;    // Clear the callback pointers
  storageNMEAGPVTG->callbackPointerPtr = NULL; // Clear the callback pointers
  storageNMEAGPVTG->callbackCopy = NULL;

  storageNMEAGPVTG->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

uint8_t SFE_UBLOX_GNSS::getLatestNMEAGNVTG(NMEA_VTG_data_t *data)
{
  if (storageNMEAGNVTG == NULL)
    initStorageNMEAGNVTG();     // Check that RAM has been allocated for the message
  if (storageNMEAGNVTG == NULL) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Use a fake UBX class and ID.

  memcpy(data, &storageNMEAGNVTG->completeCopy, sizeof(NMEA_VTG_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGNVTG->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGNVTG->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGNVTG->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

bool SFE_UBLOX_GNSS::setNMEAGNVTGcallback(void (*callbackPointer)(NMEA_VTG_data_t))
{
  if (storageNMEAGNVTG == NULL)
    initStorageNMEAGNVTG();     // Check that RAM has been allocated for the message
  if (storageNMEAGNVTG == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNVTG->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNVTG->callbackCopy = new NMEA_VTG_data_t;
  }

  if (storageNMEAGNVTG->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGNVTGcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNVTG->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setNMEAGNVTGcallbackPtr(void (*callbackPointerPtr)(NMEA_VTG_data_t *))
{
  if (storageNMEAGNVTG == NULL)
    initStorageNMEAGNVTG();     // Check that RAM has been allocated for the message
  if (storageNMEAGNVTG == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNVTG->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNVTG->callbackCopy = new NMEA_VTG_data_t;
  }

  if (storageNMEAGNVTG->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGNVTGcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNVTG->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GNVTG messages and initialize it
bool SFE_UBLOX_GNSS::initStorageNMEAGNVTG()
{
  storageNMEAGNVTG = new NMEA_GNVTG_t; // Allocate RAM for the main struct
  if (storageNMEAGNVTG == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initStorageNMEAGNVTG: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNVTG->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGNVTG->workingCopy.nmea, 0, NMEA_VTG_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGNVTG->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGNVTG->completeCopy.nmea, 0, NMEA_VTG_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGNVTG->callbackPointer = NULL;    // Clear the callback pointers
  storageNMEAGNVTG->callbackPointerPtr = NULL; // Clear the callback pointers
  storageNMEAGNVTG->callbackCopy = NULL;

  storageNMEAGNVTG->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

// Initiate automatic storage of NMEA GPRMC messages

// Get the most recent GPRMC message
// Return 0 if the message has not been received from the module
// Return 1 if the data is valid but has been read before
// Return 2 if the data is valid and is fresh/unread
uint8_t SFE_UBLOX_GNSS::getLatestNMEAGPRMC(NMEA_RMC_data_t *data)
{
  if (storageNMEAGPRMC == NULL)
    initStorageNMEAGPRMC();     // Check that RAM has been allocated for the message
  if (storageNMEAGPRMC == NULL) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Use a fake UBX class and ID.

  memcpy(data, &storageNMEAGPRMC->completeCopy, sizeof(NMEA_RMC_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGPRMC->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGPRMC->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGPRMC->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

// Enable a callback on the arrival of a GPRMC message
bool SFE_UBLOX_GNSS::setNMEAGPRMCcallback(void (*callbackPointer)(NMEA_RMC_data_t))
{
  if (storageNMEAGPRMC == NULL)
    initStorageNMEAGPRMC();     // Check that RAM has been allocated for the message
  if (storageNMEAGPRMC == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPRMC->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPRMC->callbackCopy = new NMEA_RMC_data_t;
  }

  if (storageNMEAGPRMC->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGPRMCcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPRMC->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setNMEAGPRMCcallbackPtr(void (*callbackPointerPtr)(NMEA_RMC_data_t *))
{
  if (storageNMEAGPRMC == NULL)
    initStorageNMEAGPRMC();     // Check that RAM has been allocated for the message
  if (storageNMEAGPRMC == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPRMC->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPRMC->callbackCopy = new NMEA_RMC_data_t;
  }

  if (storageNMEAGPRMC->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGPRMCcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPRMC->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GPRMC messages and initialize it
bool SFE_UBLOX_GNSS::initStorageNMEAGPRMC()
{
  storageNMEAGPRMC = new NMEA_GPRMC_t; // Allocate RAM for the main struct
  if (storageNMEAGPRMC == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initStorageNMEAGPRMC: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPRMC->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGPRMC->workingCopy.nmea, 0, NMEA_RMC_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGPRMC->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGPRMC->completeCopy.nmea, 0, NMEA_RMC_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGPRMC->callbackPointer = NULL;    // Clear the callback pointers
  storageNMEAGPRMC->callbackPointerPtr = NULL; // Clear the callback pointers
  storageNMEAGPRMC->callbackCopy = NULL;

  storageNMEAGPRMC->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

uint8_t SFE_UBLOX_GNSS::getLatestNMEAGNRMC(NMEA_RMC_data_t *data)
{
  if (storageNMEAGNRMC == NULL)
    initStorageNMEAGNRMC();     // Check that RAM has been allocated for the message
  if (storageNMEAGNRMC == NULL) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Use a fake UBX class and ID.

  memcpy(data, &storageNMEAGNRMC->completeCopy, sizeof(NMEA_RMC_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGNRMC->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGNRMC->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGNRMC->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

bool SFE_UBLOX_GNSS::setNMEAGNRMCcallback(void (*callbackPointer)(NMEA_RMC_data_t))
{
  if (storageNMEAGNRMC == NULL)
    initStorageNMEAGNRMC();     // Check that RAM has been allocated for the message
  if (storageNMEAGNRMC == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNRMC->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNRMC->callbackCopy = new NMEA_RMC_data_t;
  }

  if (storageNMEAGNRMC->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGNRMCcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNRMC->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setNMEAGNRMCcallbackPtr(void (*callbackPointerPtr)(NMEA_RMC_data_t *))
{
  if (storageNMEAGNRMC == NULL)
    initStorageNMEAGNRMC();     // Check that RAM has been allocated for the message
  if (storageNMEAGNRMC == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNRMC->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNRMC->callbackCopy = new NMEA_RMC_data_t;
  }

  if (storageNMEAGNRMC->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGNRMCcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNRMC->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GNRMC messages and initialize it
bool SFE_UBLOX_GNSS::initStorageNMEAGNRMC()
{
  storageNMEAGNRMC = new NMEA_GNRMC_t; // Allocate RAM for the main struct
  if (storageNMEAGNRMC == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initStorageNMEAGNRMC: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNRMC->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGNRMC->workingCopy.nmea, 0, NMEA_RMC_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGNRMC->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGNRMC->completeCopy.nmea, 0, NMEA_RMC_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGNRMC->callbackPointer = NULL;    // Clear the callback pointers
  storageNMEAGNRMC->callbackPointerPtr = NULL; // Clear the callback pointers
  storageNMEAGNRMC->callbackCopy = NULL;

  storageNMEAGNRMC->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

// Initiate automatic storage of NMEA GPZDA messages

// Get the most recent GPZDA message
// Return 0 if the message has not been received from the module
// Return 1 if the data is valid but has been read before
// Return 2 if the data is valid and is fresh/unread
uint8_t SFE_UBLOX_GNSS::getLatestNMEAGPZDA(NMEA_ZDA_data_t *data)
{
  if (storageNMEAGPZDA == NULL)
    initStorageNMEAGPZDA();     // Check that RAM has been allocated for the message
  if (storageNMEAGPZDA == NULL) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Use a fake UBX class and ID.

  memcpy(data, &storageNMEAGPZDA->completeCopy, sizeof(NMEA_ZDA_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGPZDA->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGPZDA->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGPZDA->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

// Enable a callback on the arrival of a GPZDA message
bool SFE_UBLOX_GNSS::setNMEAGPZDAcallback(void (*callbackPointer)(NMEA_ZDA_data_t))
{
  if (storageNMEAGPZDA == NULL)
    initStorageNMEAGPZDA();     // Check that RAM has been allocated for the message
  if (storageNMEAGPZDA == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPZDA->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPZDA->callbackCopy = new NMEA_ZDA_data_t;
  }

  if (storageNMEAGPZDA->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGPZDAcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPZDA->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setNMEAGPZDAcallbackPtr(void (*callbackPointerPtr)(NMEA_ZDA_data_t *))
{
  if (storageNMEAGPZDA == NULL)
    initStorageNMEAGPZDA();     // Check that RAM has been allocated for the message
  if (storageNMEAGPZDA == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGPZDA->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGPZDA->callbackCopy = new NMEA_ZDA_data_t;
  }

  if (storageNMEAGPZDA->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGPZDAcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPZDA->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GPZDA messages and initialize it
bool SFE_UBLOX_GNSS::initStorageNMEAGPZDA()
{
  storageNMEAGPZDA = new NMEA_GPZDA_t; // Allocate RAM for the main struct
  if (storageNMEAGPZDA == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initStorageNMEAGPZDA: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGPZDA->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGPZDA->workingCopy.nmea, 0, NMEA_ZDA_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGPZDA->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGPZDA->completeCopy.nmea, 0, NMEA_ZDA_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGPZDA->callbackPointer = NULL;    // Clear the callback pointers
  storageNMEAGPZDA->callbackPointerPtr = NULL; // Clear the callback pointers
  storageNMEAGPZDA->callbackCopy = NULL;

  storageNMEAGPZDA->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}

uint8_t SFE_UBLOX_GNSS::getLatestNMEAGNZDA(NMEA_ZDA_data_t *data)
{
  if (storageNMEAGNZDA == NULL)
    initStorageNMEAGNZDA();     // Check that RAM has been allocated for the message
  if (storageNMEAGNZDA == NULL) // Bail if the RAM allocation failed
    return (false);

  checkUbloxInternal(&packetCfg, 0, 0); // Call checkUbloxInternal to parse any incoming data. Use a fake UBX class and ID.

  memcpy(data, &storageNMEAGNZDA->completeCopy, sizeof(NMEA_ZDA_data_t)); // Copy the complete copy

  uint8_t result = 0;
  if (storageNMEAGNZDA->automaticFlags.flags.bits.completeCopyValid == 1) // Is the complete copy valid?
  {
    result = 1;
    if (storageNMEAGNZDA->automaticFlags.flags.bits.completeCopyRead == 0) // Has the data already been read?
    {
      result = 2;
      storageNMEAGNZDA->automaticFlags.flags.bits.completeCopyRead = 1; // Mark the data as read
    }
  }

  return (result);
}

bool SFE_UBLOX_GNSS::setNMEAGNZDAcallback(void (*callbackPointer)(NMEA_ZDA_data_t))
{
  if (storageNMEAGNZDA == NULL)
    initStorageNMEAGNZDA();     // Check that RAM has been allocated for the message
  if (storageNMEAGNZDA == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNZDA->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNZDA->callbackCopy = new NMEA_ZDA_data_t;
  }

  if (storageNMEAGNZDA->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGNZDAcallback: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNZDA->callbackPointer = callbackPointer;
  return (true);
}

bool SFE_UBLOX_GNSS::setNMEAGNZDAcallbackPtr(void (*callbackPointerPtr)(NMEA_ZDA_data_t *))
{
  if (storageNMEAGNZDA == NULL)
    initStorageNMEAGNZDA();     // Check that RAM has been allocated for the message
  if (storageNMEAGNZDA == NULL) // Bail if the RAM allocation failed
    return (false);

  if (storageNMEAGNZDA->callbackCopy == NULL) // Check if RAM has been allocated for the callback copy
  {
    storageNMEAGNZDA->callbackCopy = new NMEA_ZDA_data_t;
  }

  if (storageNMEAGNZDA->callbackCopy == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("setNMEAGNZDAcallbackPtr: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNZDA->callbackPointerPtr = callbackPointerPtr;
  return (true);
}

// Private: allocate RAM for incoming NMEA GNZDA messages and initialize it
bool SFE_UBLOX_GNSS::initStorageNMEAGNZDA()
{
  storageNMEAGNZDA = new NMEA_GNZDA_t; // Allocate RAM for the main struct
  if (storageNMEAGNZDA == NULL)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("initStorageNMEAGNZDA: RAM alloc failed!"));
#endif
    return (false);
  }

  storageNMEAGNZDA->workingCopy.length = 0;                            // Clear the data length
  memset(storageNMEAGNZDA->workingCopy.nmea, 0, NMEA_ZDA_MAX_LENGTH);  // Clear the nmea storage
  storageNMEAGNZDA->completeCopy.length = 0;                           // Clear the data length
  memset(storageNMEAGNZDA->completeCopy.nmea, 0, NMEA_ZDA_MAX_LENGTH); // Clear the nmea storage

  storageNMEAGNZDA->callbackPointer = NULL;    // Clear the callback pointers
  storageNMEAGNZDA->callbackPointerPtr = NULL; // Clear the callback pointers
  storageNMEAGNZDA->callbackCopy = NULL;

  storageNMEAGNZDA->automaticFlags.flags.all = 0; // Mark the data as invalid/stale and unread

  return (true);
}
#endif

// ***** CFG RATE Helper Functions

// Set the rate at which the module will give us an updated navigation solution
// Expects a number that is the updates per second. For example 1 = 1Hz, 2 = 2Hz, etc.
// Max is 40Hz(?!)
bool SFE_UBLOX_GNSS::setNavigationFrequency(uint8_t navFreq, uint16_t maxWait)
{
  if (navFreq == 0) // Return now if navFreq is zero
    return (false);

  if (navFreq > 40)
    navFreq = 40; // Limit navFreq to 40Hz so i2cPollingWait is set correctly

  // Adjust the I2C polling timeout based on update rate
  // Do this even if the sendCommand fails
  i2cPollingWaitNAV = 1000 / (((int)navFreq) * 4);                                                // This is the number of ms to wait between checks for new I2C data. Max is 250. Min is 6.
  i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR ? i2cPollingWaitNAV : i2cPollingWaitHNR; // Set i2cPollingWait to the lower of NAV and HNR

  // Query the module
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RATE;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // This will load the payloadCfg array with current settings of the given register
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);                                                       // If command send fails then bail

  uint16_t measurementRate = 1000 / navFreq;

  // payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[0] = measurementRate & 0xFF; // measRate LSB
  payloadCfg[1] = measurementRate >> 8;   // measRate MSB

  bool result = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK

  flushCFGRATE(); // Mark the polled measurement and navigation rate data as stale

  return (result);
}

// Get the rate at which the module is outputting nav solutions
uint8_t SFE_UBLOX_GNSS::getNavigationFrequency(uint16_t maxWait)
{
  if (packetUBXCFGRATE == NULL)
    initPacketUBXCFGRATE();     // Check that RAM has been allocated for the RATE data
  if (packetUBXCFGRATE == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXCFGRATE->moduleQueried.moduleQueried.bits.measRate == false)
    getNavigationFrequencyInternal(maxWait);
  packetUBXCFGRATE->moduleQueried.moduleQueried.bits.measRate = false; // Since we are about to give this to user, mark this data as stale
  packetUBXCFGRATE->moduleQueried.moduleQueried.bits.all = false;

  uint16_t measurementRate = packetUBXCFGRATE->data.measRate;

  if (measurementRate == 0)
  {
#ifndef SFE_UBLOX_REDUCED_PROG_MEM
    if ((_printDebug == true) || (_printLimitedDebug == true)) // This is important. Print this if doing limited debugging
      _debugSerial->println(F("getNavigationFrequency: zero measRate!"));
#endif
    return (0); // Avoid divide-by-zero error
  }

  measurementRate = 1000 / measurementRate; // This may return an int when it's a float, but I'd rather not return 4 bytes
  return (measurementRate);
}

// Set the elapsed time between GNSS measurements in milliseconds, which defines the rate
bool SFE_UBLOX_GNSS::setMeasurementRate(uint16_t rate, uint16_t maxWait)
{
  if (rate < 25) // "Measurement rate should be greater than or equal to 25 ms."
    rate = 25;

  // Adjust the I2C polling timeout based on update rate
  if (rate >= 1000)
    i2cPollingWaitNAV = 250;
  else
    i2cPollingWaitNAV = rate / 4;                                                                 // This is the number of ms to wait between checks for new I2C data
  i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR ? i2cPollingWaitNAV : i2cPollingWaitHNR; // Set i2cPollingWait to the lower of NAV and HNR

  // Query the module
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RATE;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // This will load the payloadCfg array with current settings of the given register
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);                                                       // If command send fails then bail

  // payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[0] = rate & 0xFF; // measRate LSB
  payloadCfg[1] = rate >> 8;   // measRate MSB

  bool result = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK

  flushCFGRATE(); // Mark the polled measurement and navigation rate data as stale

  return (result);
}

// Return the elapsed time between GNSS measurements in milliseconds, which defines the rate
uint16_t SFE_UBLOX_GNSS::getMeasurementRate(uint16_t maxWait)
{
  if (packetUBXCFGRATE == NULL)
    initPacketUBXCFGRATE();     // Check that RAM has been allocated for the RATE data
  if (packetUBXCFGRATE == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXCFGRATE->moduleQueried.moduleQueried.bits.measRate == false)
    getNavigationFrequencyInternal(maxWait);
  packetUBXCFGRATE->moduleQueried.moduleQueried.bits.measRate = false; // Since we are about to give this to user, mark this data as stale
  packetUBXCFGRATE->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXCFGRATE->data.measRate);
}

// Set the ratio between the number of measurements and the number of navigation solutions. Unit is cycles. Max is 127.
bool SFE_UBLOX_GNSS::setNavigationRate(uint16_t rate, uint16_t maxWait)
{
  // Query the module
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_RATE;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // This will load the payloadCfg array with current settings of the given register
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED) // We are expecting data and an ACK
    return (false);                                                       // If command send fails then bail

  // payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[2] = rate & 0xFF; // navRate LSB
  payloadCfg[3] = rate >> 8;   // navRate MSB

  bool result = ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK

  flushCFGRATE(); // Mark the polled measurement and navigation rate data as stale

  return (result);
}

// Return the ratio between the number of measurements and the number of navigation solutions. Unit is cycles
uint16_t SFE_UBLOX_GNSS::getNavigationRate(uint16_t maxWait)
{
  if (packetUBXCFGRATE == NULL)
    initPacketUBXCFGRATE();     // Check that RAM has been allocated for the RATE data
  if (packetUBXCFGRATE == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXCFGRATE->moduleQueried.moduleQueried.bits.navRate == false)
    getNavigationFrequencyInternal(maxWait);
  packetUBXCFGRATE->moduleQueried.moduleQueried.bits.navRate = false; // Since we are about to give this to user, mark this data as stale
  packetUBXCFGRATE->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXCFGRATE->data.navRate);
}

// Mark the CFG RATE data as read/stale
void SFE_UBLOX_GNSS::flushCFGRATE()
{
  if (packetUBXCFGRATE == NULL)
    return;                                              // Bail if RAM has not been allocated (otherwise we could be writing anywhere!)
  packetUBXCFGRATE->moduleQueried.moduleQueried.all = 0; // Mark all datums as stale (read before)
}

// ***** DOP Helper Functions

uint16_t SFE_UBLOX_GNSS::getGeometricDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == NULL)
    initPacketUBXNAVDOP();     // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.gDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.gDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.gDOP);
}

uint16_t SFE_UBLOX_GNSS::getPositionDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == NULL)
    initPacketUBXNAVDOP();     // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.pDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.pDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.pDOP);
}

uint16_t SFE_UBLOX_GNSS::getTimeDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == NULL)
    initPacketUBXNAVDOP();     // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.tDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.tDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.tDOP);
}

uint16_t SFE_UBLOX_GNSS::getVerticalDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == NULL)
    initPacketUBXNAVDOP();     // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.vDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.vDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.vDOP);
}

uint16_t SFE_UBLOX_GNSS::getHorizontalDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == NULL)
    initPacketUBXNAVDOP();     // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.hDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.hDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.hDOP);
}

uint16_t SFE_UBLOX_GNSS::getNorthingDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == NULL)
    initPacketUBXNAVDOP();     // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.nDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.nDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.nDOP);
}

uint16_t SFE_UBLOX_GNSS::getEastingDOP(uint16_t maxWait)
{
  if (packetUBXNAVDOP == NULL)
    initPacketUBXNAVDOP();     // Check that RAM has been allocated for the DOP data
  if (packetUBXNAVDOP == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVDOP->moduleQueried.moduleQueried.bits.eDOP == false)
    getDOP(maxWait);
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.eDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVDOP->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVDOP->data.eDOP);
}

// ***** ATT Helper Functions

float SFE_UBLOX_GNSS::getATTroll(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXNAVATT == NULL)
    initPacketUBXNAVATT();     // Check that RAM has been allocated for the NAV ATT data
  if (packetUBXNAVATT == NULL) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXNAVATT->moduleQueried.moduleQueried.bits.roll == false)
    getNAVATT(maxWait);
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.roll = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVATT->data.roll) / 100000.0); // Convert to degrees
}

float SFE_UBLOX_GNSS::getATTpitch(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXNAVATT == NULL)
    initPacketUBXNAVATT();     // Check that RAM has been allocated for the NAV ATT data
  if (packetUBXNAVATT == NULL) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXNAVATT->moduleQueried.moduleQueried.bits.pitch == false)
    getNAVATT(maxWait);
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.pitch = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVATT->data.pitch) / 100000.0); // Convert to degrees
}

float SFE_UBLOX_GNSS::getATTheading(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXNAVATT == NULL)
    initPacketUBXNAVATT();     // Check that RAM has been allocated for the NAV ATT data
  if (packetUBXNAVATT == NULL) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXNAVATT->moduleQueried.moduleQueried.bits.heading == false)
    getNAVATT(maxWait);
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.heading = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVATT->data.heading) / 100000.0); // Convert to degrees
}

// ***** PVT Helper Functions

uint32_t SFE_UBLOX_GNSS::getTimeOfWeek(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.iTOW);
}

// Get the current year
uint16_t SFE_UBLOX_GNSS::getYear(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.year);
}

// Get the current month
uint8_t SFE_UBLOX_GNSS::getMonth(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.month);
}

// Get the current day
uint8_t SFE_UBLOX_GNSS::getDay(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.day);
}

// Get the current hour
uint8_t SFE_UBLOX_GNSS::getHour(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.hour);
}

// Get the current minute
uint8_t SFE_UBLOX_GNSS::getMinute(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.min);
}

// Get the current second
uint8_t SFE_UBLOX_GNSS::getSecond(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.sec);
}

// Get the current millisecond
uint16_t SFE_UBLOX_GNSS::getMillisecond(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.iTOW = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.iTOW % 1000);
}

// Get the current nanoseconds - includes milliseconds
int32_t SFE_UBLOX_GNSS::getNanosecond(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.nano);
}

// Get the current Unix epoch time rounded to the nearest second
uint32_t SFE_UBLOX_GNSS::getUnixEpoch(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  uint32_t t = SFE_UBLOX_DAYS_FROM_1970_TO_2020;                                                                           // Jan 1st 2020 as days from Jan 1st 1970
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_2020[packetUBXNAVPVT->data.year - 2020];                                             // Add on the number of days since 2020
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_MONTH[packetUBXNAVPVT->data.year % 4 == 0 ? 0 : 1][packetUBXNAVPVT->data.month - 1]; // Add on the number of days since Jan 1st
  t += (uint32_t)packetUBXNAVPVT->data.day - 1;                                                                            // Add on the number of days since the 1st of the month
  t *= 24;                                                                                                                 // Convert to hours
  t += (uint32_t)packetUBXNAVPVT->data.hour;                                                                               // Add on the hour
  t *= 60;                                                                                                                 // Convert to minutes
  t += (uint32_t)packetUBXNAVPVT->data.min;                                                                                // Add on the minute
  t *= 60;                                                                                                                 // Convert to seconds
  t += (uint32_t)packetUBXNAVPVT->data.sec;                                                                                // Add on the second
  return t;
}

// Get the current Unix epoch including microseconds
uint32_t SFE_UBLOX_GNSS::getUnixEpoch(uint32_t &microsecond, uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.year = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.month = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.day = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hour = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.min = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.sec = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.nano = false;
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  uint32_t t = SFE_UBLOX_DAYS_FROM_1970_TO_2020;                                                                           // Jan 1st 2020 as days from Jan 1st 1970
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_2020[packetUBXNAVPVT->data.year - 2020];                                             // Add on the number of days since 2020
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_MONTH[packetUBXNAVPVT->data.year % 4 == 0 ? 0 : 1][packetUBXNAVPVT->data.month - 1]; // Add on the number of days since Jan 1st
  t += (uint32_t)packetUBXNAVPVT->data.day - 1;                                                                            // Add on the number of days since the 1st of the month
  t *= 24;                                                                                                                 // Convert to hours
  t += (uint32_t)packetUBXNAVPVT->data.hour;                                                                               // Add on the hour
  t *= 60;                                                                                                                 // Convert to minutes
  t += (uint32_t)packetUBXNAVPVT->data.min;                                                                                // Add on the minute
  t *= 60;                                                                                                                 // Convert to seconds
  t += (uint32_t)packetUBXNAVPVT->data.sec;                                                                                // Add on the second
  int32_t us = packetUBXNAVPVT->data.nano / 1000;                                                                          // Convert nanos to micros
  microsecond = (uint32_t)us;                                                                                              // Could be -ve!
  // Adjust t if nano is negative
  if (us < 0)
  {
    microsecond = (uint32_t)(us + 1000000); // Make nano +ve
    t--;                                    // Decrement t by 1 second
  }
  return t;
}

// Get the current date validity
bool SFE_UBLOX_GNSS::getDateValid(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validDate == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validDate = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.valid.bits.validDate);
}

// Get the current time validity
bool SFE_UBLOX_GNSS::getTimeValid(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validTime == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.validTime = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.valid.bits.validTime);
}

// Check to see if the UTC time has been fully resolved
bool SFE_UBLOX_GNSS::getTimeFullyResolved(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fullyResolved == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fullyResolved = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.valid.bits.fullyResolved);
}

// Get the confirmed date validity
bool SFE_UBLOX_GNSS::getConfirmedDate(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedDate == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedDate = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.flags2.bits.confirmedDate);
}

// Get the confirmed time validity
bool SFE_UBLOX_GNSS::getConfirmedTime(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedTime == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.confirmedTime = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.flags2.bits.confirmedTime);
}

// Get the current fix type
// 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
uint8_t SFE_UBLOX_GNSS::getFixType(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fixType == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.fixType = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.fixType);
}

// Get whether we have a valid fix (i.e within DOP & accuracy masks)
bool SFE_UBLOX_GNSS::getGnssFixOk(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.gnssFixOK == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.gnssFixOK = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.flags.bits.gnssFixOK);
}

// Get whether differential corrections were applied
bool SFE_UBLOX_GNSS::getDiffSoln(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.diffSoln == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.diffSoln = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.flags.bits.diffSoln);
}

// Get whether head vehicle valid or not
bool SFE_UBLOX_GNSS::getHeadVehValid(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.headVehValid == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.headVehValid = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.flags.bits.headVehValid);
}

// Get the carrier phase range solution status
// Useful when querying module to see if it has high-precision RTK fix
// 0=No solution, 1=Float solution, 2=Fixed solution
uint8_t SFE_UBLOX_GNSS::getCarrierSolutionType(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.carrSoln == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.carrSoln = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.flags.bits.carrSoln);
}

// Get the number of satellites used in fix
uint8_t SFE_UBLOX_GNSS::getSIV(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.numSV == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.numSV = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.numSV);
}

// Get the current longitude in degrees
// Returns a long representing the number of degrees *10^-7
int32_t SFE_UBLOX_GNSS::getLongitude(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lon == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lon = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.lon);
}

// Get the current latitude in degrees
// Returns a long representing the number of degrees *10^-7
int32_t SFE_UBLOX_GNSS::getLatitude(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lat == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.lat = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.lat);
}

// Get the current altitude in mm according to ellipsoid model
int32_t SFE_UBLOX_GNSS::getAltitude(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.height == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.height = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.height);
}

// Get the current altitude in mm according to mean sea level
// Ellipsoid model: https://www.esri.com/news/arcuser/0703/geoid1of3.html
// Difference between Ellipsoid Model and Mean Sea Level: https://eos-gnss.com/elevation-for-beginners/
int32_t SFE_UBLOX_GNSS::getAltitudeMSL(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hMSL == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hMSL = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.hMSL);
}

int32_t SFE_UBLOX_GNSS::getHorizontalAccEst(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hAcc == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.hAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.hAcc);
}

int32_t SFE_UBLOX_GNSS::getVerticalAccEst(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.vAcc == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.vAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.vAcc);
}

int32_t SFE_UBLOX_GNSS::getNedNorthVel(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velN == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velN = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.velN);
}

int32_t SFE_UBLOX_GNSS::getNedEastVel(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velE == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.velE = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.velE);
}

int32_t SFE_UBLOX_GNSS::getNedDownVel(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.velD == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.velD = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.velD);
}

// Get the ground speed in mm/s
int32_t SFE_UBLOX_GNSS::getGroundSpeed(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.gSpeed == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.gSpeed = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.gSpeed);
}

// Get the heading of motion (as opposed to heading of car) in degrees * 10^-5
int32_t SFE_UBLOX_GNSS::getHeading(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headMot == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headMot = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.headMot);
}

uint32_t SFE_UBLOX_GNSS::getSpeedAccEst(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.sAcc == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.sAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.sAcc);
}

uint32_t SFE_UBLOX_GNSS::getHeadingAccEst(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headAcc == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.headAcc);
}

// Get the positional dillution of precision * 10^-2 (dimensionless)
uint16_t SFE_UBLOX_GNSS::getPDOP(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.pDOP == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.pDOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.pDOP);
}

bool SFE_UBLOX_GNSS::getInvalidLlh(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return (false);

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.invalidLlh == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.invalidLlh = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return ((bool)packetUBXNAVPVT->data.flags3.bits.invalidLlh);
}

int32_t SFE_UBLOX_GNSS::getHeadVeh(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headVeh == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.headVeh = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.headVeh);
}

int16_t SFE_UBLOX_GNSS::getMagDec(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magDec == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magDec = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.magDec);
}

uint16_t SFE_UBLOX_GNSS::getMagAcc(uint16_t maxWait)
{
  if (packetUBXNAVPVT == NULL)
    initPacketUBXNAVPVT();     // Check that RAM has been allocated for the PVT data
  if (packetUBXNAVPVT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magAcc == false)
    getPVT(maxWait);
  packetUBXNAVPVT->moduleQueried.moduleQueried2.bits.magAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVT->data.magAcc);
}

// getGeoidSeparation is currently redundant. The geoid separation seems to only be provided in NMEA GGA and GNS messages.
int32_t SFE_UBLOX_GNSS::getGeoidSeparation(uint16_t maxWait)
{
  (void)maxWait; // Do something with maxWait just to get rid of the pesky compiler warning

  return (0);
}

// ***** HPPOSECEF Helper Functions

// Get the current 3D high precision positional accuracy - a fun thing to watch
// Returns a long representing the 3D accuracy in millimeters
uint32_t SFE_UBLOX_GNSS::getPositionAccuracy(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == NULL)
    initPacketUBXNAVHPPOSECEF();     // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.pAcc == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.pAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  uint32_t tempAccuracy = packetUBXNAVHPPOSECEF->data.pAcc;

  if ((tempAccuracy % 10) >= 5)
    tempAccuracy += 5; // Round fraction of mm up to next mm if .5 or above
  tempAccuracy /= 10;  // Convert 0.1mm units to mm

  return (tempAccuracy);
}

// Get the current 3D high precision X coordinate
// Returns a long representing the coordinate in cm
int32_t SFE_UBLOX_GNSS::getHighResECEFX(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == NULL)
    initPacketUBXNAVHPPOSECEF();     // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefX == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefX = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefX);
}

// Get the current 3D high precision Y coordinate
// Returns a long representing the coordinate in cm
int32_t SFE_UBLOX_GNSS::getHighResECEFY(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == NULL)
    initPacketUBXNAVHPPOSECEF();     // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefY == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefY = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefY);
}

// Get the current 3D high precision Z coordinate
// Returns a long representing the coordinate in cm
int32_t SFE_UBLOX_GNSS::getHighResECEFZ(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == NULL)
    initPacketUBXNAVHPPOSECEF();     // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZ == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZ = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefZ);
}

// Get the high precision component of the ECEF X coordinate
// Returns a signed byte representing the component as 0.1*mm
int8_t SFE_UBLOX_GNSS::getHighResECEFXHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == NULL)
    initPacketUBXNAVHPPOSECEF();     // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefXHp == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefXHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefXHp);
}

// Get the high precision component of the ECEF Y coordinate
// Returns a signed byte representing the component as 0.1*mm
int8_t SFE_UBLOX_GNSS::getHighResECEFYHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == NULL)
    initPacketUBXNAVHPPOSECEF();     // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefYHp == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefYHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefYHp);
}

// Get the high precision component of the ECEF Z coordinate
// Returns a signed byte representing the component as 0.1*mm
int8_t SFE_UBLOX_GNSS::getHighResECEFZHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSECEF == NULL)
    initPacketUBXNAVHPPOSECEF();     // Check that RAM has been allocated for the HPPOSECEF data
  if (packetUBXNAVHPPOSECEF == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZHp == false)
    getNAVHPPOSECEF(maxWait);
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.ecefZHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSECEF->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVHPPOSECEF->data.ecefZHp);
}

// ***** HPPOSLLH Helper Functions

uint32_t SFE_UBLOX_GNSS::getTimeOfWeekFromHPPOSLLH(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.iTOW == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.iTOW = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.iTOW);
}

int32_t SFE_UBLOX_GNSS::getHighResLongitude(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lon == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lon = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.lon);
}

int32_t SFE_UBLOX_GNSS::getHighResLatitude(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lat == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lat = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.lat);
}

int32_t SFE_UBLOX_GNSS::getElipsoid(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.height == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.height = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.height);
}

int32_t SFE_UBLOX_GNSS::getMeanSeaLevel(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSL == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSL = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.hMSL);
}

int8_t SFE_UBLOX_GNSS::getHighResLongitudeHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lonHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.lonHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.lonHp);
}

int8_t SFE_UBLOX_GNSS::getHighResLatitudeHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.latHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.latHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.latHp);
}

int8_t SFE_UBLOX_GNSS::getElipsoidHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.heightHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.heightHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.heightHp);
}

int8_t SFE_UBLOX_GNSS::getMeanSeaLevelHp(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSLHp == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hMSLHp = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.hMSLHp);
}

uint32_t SFE_UBLOX_GNSS::getHorizontalAccuracy(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hAcc == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.hAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.hAcc);
}

uint32_t SFE_UBLOX_GNSS::getVerticalAccuracy(uint16_t maxWait)
{
  if (packetUBXNAVHPPOSLLH == NULL)
    initPacketUBXNAVHPPOSLLH();     // Check that RAM has been allocated for the HPPOSLLH data
  if (packetUBXNAVHPPOSLLH == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.vAcc == false)
    getHPPOSLLH(maxWait);
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.vAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVHPPOSLLH->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVHPPOSLLH->data.vAcc);
}

// ***** PVAT Helper Functions

int32_t SFE_UBLOX_GNSS::getVehicleRoll(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == NULL)
    initPacketUBXNAVPVAT();     // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehRoll == false)
    getNAVPVAT(maxWait);
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehRoll = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVAT->data.vehRoll);
}

int32_t SFE_UBLOX_GNSS::getVehiclePitch(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == NULL)
    initPacketUBXNAVPVAT();     // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehPitch == false)
    getNAVPVAT(maxWait);
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehPitch = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVAT->data.vehPitch);
}

int32_t SFE_UBLOX_GNSS::getVehicleHeading(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == NULL)
    initPacketUBXNAVPVAT();     // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehHeading == false)
    getNAVPVAT(maxWait);
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.vehHeading = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVAT->data.vehHeading);
}

int32_t SFE_UBLOX_GNSS::getMotionHeading(uint16_t maxWait)
{
  if (packetUBXNAVPVAT == NULL)
    initPacketUBXNAVPVAT();     // Check that RAM has been allocated for the PVAT data
  if (packetUBXNAVPVAT == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.motHeading == false)
    getNAVPVAT(maxWait);
  packetUBXNAVPVAT->moduleQueried.moduleQueried2.bits.motHeading = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVPVAT->moduleQueried.moduleQueried1.bits.all = false;
  return (packetUBXNAVPVAT->data.motHeading);
}

// ***** SVIN Helper Functions

bool SFE_UBLOX_GNSS::getSurveyInActive(uint16_t maxWait)
{
  if (packetUBXNAVSVIN == NULL)
    initPacketUBXNAVSVIN();     // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == NULL) // Bail if the RAM allocation failed
    return false;

  if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.active == false)
    getSurveyStatus(maxWait);
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.active = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;
  return ((bool)packetUBXNAVSVIN->data.active);
}

bool SFE_UBLOX_GNSS::getSurveyInValid(uint16_t maxWait)
{
  if (packetUBXNAVSVIN == NULL)
    initPacketUBXNAVSVIN();     // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == NULL) // Bail if the RAM allocation failed
    return false;

  if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.valid == false)
    getSurveyStatus(maxWait);
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.valid = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;
  return ((bool)packetUBXNAVSVIN->data.valid);
}

uint32_t SFE_UBLOX_GNSS::getSurveyInObservationTimeFull(uint16_t maxWait) // Return the full uint32_t
{
  if (packetUBXNAVSVIN == NULL)
    initPacketUBXNAVSVIN();     // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.dur == false)
    getSurveyStatus(maxWait);
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.dur = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;

  return (packetUBXNAVSVIN->data.dur);
}

uint16_t SFE_UBLOX_GNSS::getSurveyInObservationTime(uint16_t maxWait) // Truncated to 65535 seconds
{
  // dur (Passed survey-in observation time) is U4 (uint32_t) seconds. Here we truncate to 16 bits
  uint32_t tmpObsTime = getSurveyInObservationTimeFull(maxWait);
  if (tmpObsTime <= 0xFFFF)
  {
    return ((uint16_t)tmpObsTime);
  }
  else
  {
    return (0xFFFF);
  }
}

float SFE_UBLOX_GNSS::getSurveyInMeanAccuracy(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVSVIN == NULL)
    initPacketUBXNAVSVIN();     // Check that RAM has been allocated for the SVIN data
  if (packetUBXNAVSVIN == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.meanAcc == false)
    getSurveyStatus(maxWait);
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.meanAcc = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVSVIN->moduleQueried.moduleQueried.bits.all = false;

  // meanAcc is U4 (uint32_t) in 0.1mm. We convert this to float.
  uint32_t tempFloat = packetUBXNAVSVIN->data.meanAcc;
  return (((float)tempFloat) / 10000.0); // Convert 0.1mm to m
}

// ***** TIMELS Helper Functions

uint8_t SFE_UBLOX_GNSS::getLeapIndicator(int32_t &timeToLsEvent, uint16_t maxWait)
{
  if (packetUBXNAVTIMELS == NULL)
    initPacketUBXNAVTIMELS();     // Check that RAM has been allocated for the TIMELS data
  if (packetUBXNAVTIMELS == NULL) // Bail if the RAM allocation failed
    return 3;

  if (packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.validTimeToLsEvent == false)
    getLeapSecondEvent(maxWait);
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.validTimeToLsEvent = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.lsChange = false;
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.timeToLsEvent = false;
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.all = false;
  timeToLsEvent = packetUBXNAVTIMELS->data.timeToLsEvent;
  // returns NTP Leap Indicator
  // 0 -no warning
  // 1 -last minute of the day has 61 seconds
  // 2 -last minute of the day has 59 seconds
  // 3 -unknown (clock unsynchronized)
  return ((bool)packetUBXNAVTIMELS->data.valid.bits.validTimeToLsEvent ? (uint8_t)(packetUBXNAVTIMELS->data.lsChange == -1 ? 2 : packetUBXNAVTIMELS->data.lsChange) : 3);
}

int8_t SFE_UBLOX_GNSS::getCurrentLeapSeconds(sfe_ublox_ls_src_e &source, uint16_t maxWait)
{
  if (packetUBXNAVTIMELS == NULL)
    initPacketUBXNAVTIMELS();     // Check that RAM has been allocated for the TIMELS data
  if (packetUBXNAVTIMELS == NULL) // Bail if the RAM allocation failed
    return false;

  if (packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.validCurrLs == false)
    getLeapSecondEvent(maxWait);
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.validCurrLs = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.srcOfCurrLs = false;
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.currLs = false;
  packetUBXNAVTIMELS->moduleQueried.moduleQueried.bits.all = false;
  source = ((sfe_ublox_ls_src_e)packetUBXNAVTIMELS->data.srcOfCurrLs);
  return ((int8_t)packetUBXNAVTIMELS->data.currLs);
}

// ***** RELPOSNED Helper Functions and automatic support

float SFE_UBLOX_GNSS::getRelPosN(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == NULL)
    initPacketUBXNAVRELPOSNED();     // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosN == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosN = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.relPosN) / 100.0); // Convert to m
}

float SFE_UBLOX_GNSS::getRelPosE(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == NULL)
    initPacketUBXNAVRELPOSNED();     // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosE == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosE = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.relPosE) / 100.0); // Convert to m
}

float SFE_UBLOX_GNSS::getRelPosD(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == NULL)
    initPacketUBXNAVRELPOSNED();     // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosD == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.relPosD = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.relPosD) / 100.0); // Convert to m
}

float SFE_UBLOX_GNSS::getRelPosAccN(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == NULL)
    initPacketUBXNAVRELPOSNED();     // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accN == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accN = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.accN) / 10000.0); // Convert to m
}

float SFE_UBLOX_GNSS::getRelPosAccE(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == NULL)
    initPacketUBXNAVRELPOSNED();     // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accE == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accE = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.accE) / 10000.0); // Convert to m
}

float SFE_UBLOX_GNSS::getRelPosAccD(uint16_t maxWait) // Returned as m
{
  if (packetUBXNAVRELPOSNED == NULL)
    initPacketUBXNAVRELPOSNED();     // Check that RAM has been allocated for the RELPOSNED data
  if (packetUBXNAVRELPOSNED == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accD == false)
    getRELPOSNED(maxWait);
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.accD = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVRELPOSNED->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXNAVRELPOSNED->data.accD) / 10000.0); // Convert to m
}

// ***** AOPSTATUS Helper Functions

uint8_t SFE_UBLOX_GNSS::getAOPSTATUSuseAOP(uint16_t maxWait)
{
  if (packetUBXNAVAOPSTATUS == NULL)
    initPacketUBXNAVAOPSTATUS();     // Check that RAM has been allocated for the AOPSTATUS data
  if (packetUBXNAVAOPSTATUS == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.useAOP == false)
    getAOPSTATUS(maxWait);
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.useAOP = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVAOPSTATUS->data.aopCfg.bits.useAOP);
}

uint8_t SFE_UBLOX_GNSS::getAOPSTATUSstatus(uint16_t maxWait)
{
  if (packetUBXNAVAOPSTATUS == NULL)
    initPacketUBXNAVAOPSTATUS();     // Check that RAM has been allocated for the AOPSTATUS data
  if (packetUBXNAVAOPSTATUS == NULL) // Bail if the RAM allocation failed
    return 0;

  if (packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.status == false)
    getAOPSTATUS(maxWait);
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.status = false; // Since we are about to give this to user, mark this data as stale
  packetUBXNAVAOPSTATUS->moduleQueried.moduleQueried.bits.all = false;
  return (packetUBXNAVAOPSTATUS->data.status);
}

// ***** ESF Helper Functions

float SFE_UBLOX_GNSS::getESFroll(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXESFALG == NULL)
    initPacketUBXESFALG();     // Check that RAM has been allocated for the ESF ALG data
  if (packetUBXESFALG == NULL) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXESFALG->moduleQueried.moduleQueried.bits.roll == false)
    getESFALG(maxWait);
  packetUBXESFALG->moduleQueried.moduleQueried.bits.roll = false; // Since we are about to give this to user, mark this data as stale
  packetUBXESFALG->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXESFALG->data.roll) / 100.0); // Convert to degrees
}

float SFE_UBLOX_GNSS::getESFpitch(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXESFALG == NULL)
    initPacketUBXESFALG();     // Check that RAM has been allocated for the ESF ALG data
  if (packetUBXESFALG == NULL) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXESFALG->moduleQueried.moduleQueried.bits.pitch == false)
    getESFALG(maxWait);
  packetUBXESFALG->moduleQueried.moduleQueried.bits.pitch = false; // Since we are about to give this to user, mark this data as stale
  packetUBXESFALG->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXESFALG->data.pitch) / 100.0); // Convert to degrees
}

float SFE_UBLOX_GNSS::getESFyaw(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXESFALG == NULL)
    initPacketUBXESFALG();     // Check that RAM has been allocated for the ESF ALG data
  if (packetUBXESFALG == NULL) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXESFALG->moduleQueried.moduleQueried.bits.yaw == false)
    getESFALG(maxWait);
  packetUBXESFALG->moduleQueried.moduleQueried.bits.yaw = false; // Since we are about to give this to user, mark this data as stale
  packetUBXESFALG->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXESFALG->data.yaw) / 100.0); // Convert to degrees
}

bool SFE_UBLOX_GNSS::getSensorFusionMeasurement(UBX_ESF_MEAS_sensorData_t *sensorData, UBX_ESF_MEAS_data_t ubxDataStruct, uint8_t sensor)
{
  sensorData->data.all = ubxDataStruct.data[sensor].data.all;
  return (true);
}

bool SFE_UBLOX_GNSS::getRawSensorMeasurement(UBX_ESF_RAW_sensorData_t *sensorData, UBX_ESF_RAW_data_t ubxDataStruct, uint8_t sensor)
{
  sensorData->data.all = ubxDataStruct.data[sensor].data.all;
  sensorData->sTag = ubxDataStruct.data[sensor].sTag;
  return (true);
}

bool SFE_UBLOX_GNSS::getSensorFusionStatus(UBX_ESF_STATUS_sensorStatus_t *sensorStatus, uint8_t sensor, uint16_t maxWait)
{
  if (packetUBXESFSTATUS == NULL)
    initPacketUBXESFSTATUS();     // Check that RAM has been allocated for the ESF STATUS data
  if (packetUBXESFSTATUS == NULL) // Bail if the RAM allocation failed
    return (false);

  if ((packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.status & (1 << sensor)) == 0)
    getESFSTATUS(maxWait);
  packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.status &= ~(1 << sensor); // Since we are about to give this to user, mark this data as stale
  packetUBXESFSTATUS->moduleQueried.moduleQueried.bits.all = false;
  sensorStatus->sensStatus1.all = packetUBXESFSTATUS->data.status[sensor].sensStatus1.all;
  sensorStatus->sensStatus2.all = packetUBXESFSTATUS->data.status[sensor].sensStatus2.all;
  sensorStatus->freq = packetUBXESFSTATUS->data.status[sensor].freq;
  sensorStatus->faults.all = packetUBXESFSTATUS->data.status[sensor].faults.all;
  return (true);
}

bool SFE_UBLOX_GNSS::getSensorFusionStatus(UBX_ESF_STATUS_sensorStatus_t *sensorStatus, UBX_ESF_STATUS_data_t ubxDataStruct, uint8_t sensor)
{
  sensorStatus->sensStatus1.all = ubxDataStruct.status[sensor].sensStatus1.all;
  sensorStatus->sensStatus2.all = ubxDataStruct.status[sensor].sensStatus2.all;
  sensorStatus->freq = ubxDataStruct.status[sensor].freq;
  sensorStatus->faults.all = ubxDataStruct.status[sensor].faults.all;
  return (true);
}

// ***** HNR Helper Functions

// Set the High Navigation Rate
// Returns true if the setHNRNavigationRate is successful
bool SFE_UBLOX_GNSS::setHNRNavigationRate(uint8_t rate, uint16_t maxWait)
{
  if (rate == 0) // Return now if rate is zero
    return (false);

  if (rate > 40)
    rate = 40; // Limit rate to 40Hz so i2cPollingWait is set correctly

  // Adjust the I2C polling timeout based on update rate
  // Do this even if the sendCommand is not ACK'd
  i2cPollingWaitHNR = 1000 / (((int)rate) * 4);                                                   // This is the number of ms to wait between checks for new I2C data. Max 250. Min 6.
  i2cPollingWait = i2cPollingWaitNAV < i2cPollingWaitHNR ? i2cPollingWaitNAV : i2cPollingWaitHNR; // Set i2cPollingWait to the lower of NAV and HNR

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_HNR;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the current HNR settings. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (false);

  // Load the new navigation rate into payloadCfg
  payloadCfg[0] = rate;

  // Update the navigation rate
  sfe_ublox_status_e result = sendCommand(&packetCfg, maxWait); // We are only expecting an ACK

  return (result == SFE_UBLOX_STATUS_DATA_SENT);
}

// Get the High Navigation Rate
// Returns 0 if the getHNRNavigationRate fails
uint8_t SFE_UBLOX_GNSS::getHNRNavigationRate(uint16_t maxWait)
{
  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_HNR;
  packetCfg.len = 0;
  packetCfg.startingSpot = 0;

  // Ask module for the current HNR settings. Loads into payloadCfg.
  if (sendCommand(&packetCfg, maxWait) != SFE_UBLOX_STATUS_DATA_RECEIVED)
    return (0);

  // Return the navigation rate
  return (payloadCfg[0]);
}

float SFE_UBLOX_GNSS::getHNRroll(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXHNRATT == NULL)
    initPacketUBXHNRATT();     // Check that RAM has been allocated for the HNR ATT data
  if (packetUBXHNRATT == NULL) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXHNRATT->moduleQueried.moduleQueried.bits.roll == false)
    getHNRATT(maxWait);
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.roll = false; // Since we are about to give this to user, mark this data as stale
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXHNRATT->data.roll) / 100000.0); // Convert to degrees
}

float SFE_UBLOX_GNSS::getHNRpitch(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXHNRATT == NULL)
    initPacketUBXHNRATT();     // Check that RAM has been allocated for the HNR ATT data
  if (packetUBXHNRATT == NULL) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXHNRATT->moduleQueried.moduleQueried.bits.pitch == false)
    getHNRATT(maxWait);
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.pitch = false; // Since we are about to give this to user, mark this data as stale
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXHNRATT->data.pitch) / 100000.0); // Convert to degrees
}

float SFE_UBLOX_GNSS::getHNRheading(uint16_t maxWait) // Returned as degrees
{
  if (packetUBXHNRATT == NULL)
    initPacketUBXHNRATT();     // Check that RAM has been allocated for the HNR ATT data
  if (packetUBXHNRATT == NULL) // Bail if the RAM allocation failed
    return (0);

  if (packetUBXHNRATT->moduleQueried.moduleQueried.bits.heading == false)
    getHNRATT(maxWait);
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.heading = false; // Since we are about to give this to user, mark this data as stale
  packetUBXHNRATT->moduleQueried.moduleQueried.bits.all = false;
  return (((float)packetUBXHNRATT->data.heading) / 100000.0); // Convert to degrees
}

// Functions to extract signed and unsigned 8/16/32-bit data from a ubxPacket
// From v2.0: These are public. The user can call these to extract data from custom packets

// Given a spot in the payload array, extract eight bytes and build a uint64_t
uint64_t SFE_UBLOX_GNSS::extractLongLong(ubxPacket *msg, uint16_t spotToStart)
{
  uint64_t val = 0;
  val |= (uint64_t)msg->payload[spotToStart + 0] << 8 * 0;
  val |= (uint64_t)msg->payload[spotToStart + 1] << 8 * 1;
  val |= (uint64_t)msg->payload[spotToStart + 2] << 8 * 2;
  val |= (uint64_t)msg->payload[spotToStart + 3] << 8 * 3;
  val |= (uint64_t)msg->payload[spotToStart + 4] << 8 * 4;
  val |= (uint64_t)msg->payload[spotToStart + 5] << 8 * 5;
  val |= (uint64_t)msg->payload[spotToStart + 6] << 8 * 6;
  val |= (uint64_t)msg->payload[spotToStart + 7] << 8 * 7;
  return (val);
}

// Given a spot in the payload array, extract four bytes and build a long
uint32_t SFE_UBLOX_GNSS::extractLong(ubxPacket *msg, uint16_t spotToStart)
{
  uint32_t val = 0;
  val |= (uint32_t)msg->payload[spotToStart + 0] << 8 * 0;
  val |= (uint32_t)msg->payload[spotToStart + 1] << 8 * 1;
  val |= (uint32_t)msg->payload[spotToStart + 2] << 8 * 2;
  val |= (uint32_t)msg->payload[spotToStart + 3] << 8 * 3;
  return (val);
}

// Just so there is no ambiguity about whether a uint32_t will cast to a int32_t correctly...
int32_t SFE_UBLOX_GNSS::extractSignedLong(ubxPacket *msg, uint16_t spotToStart)
{
  union // Use a union to convert from uint32_t to int32_t
  {
    uint32_t unsignedLong;
    int32_t signedLong;
  } unsignedSigned;

  unsignedSigned.unsignedLong = extractLong(msg, spotToStart);
  return (unsignedSigned.signedLong);
}

// Given a spot in the payload array, extract two bytes and build an int
uint16_t SFE_UBLOX_GNSS::extractInt(ubxPacket *msg, uint16_t spotToStart)
{
  uint16_t val = 0;
  val |= (uint16_t)msg->payload[spotToStart + 0] << 8 * 0;
  val |= (uint16_t)msg->payload[spotToStart + 1] << 8 * 1;
  return (val);
}

// Just so there is no ambiguity about whether a uint16_t will cast to a int16_t correctly...
int16_t SFE_UBLOX_GNSS::extractSignedInt(ubxPacket *msg, uint16_t spotToStart)
{
  union // Use a union to convert from uint16_t to int16_t
  {
    uint16_t unsignedInt;
    int16_t signedInt;
  } stSignedInt;

  stSignedInt.unsignedInt = extractInt(msg, spotToStart);
  return (stSignedInt.signedInt);
}

// Given a spot, extract a byte from the payload
uint8_t SFE_UBLOX_GNSS::extractByte(ubxPacket *msg, uint16_t spotToStart)
{
  return (msg->payload[spotToStart]);
}

// Given a spot, extract a signed 8-bit value from the payload
int8_t SFE_UBLOX_GNSS::extractSignedChar(ubxPacket *msg, uint16_t spotToStart)
{
  union // Use a union to convert from uint8_t to int8_t
  {
    uint8_t unsignedByte;
    int8_t signedByte;
  } stSignedByte;

  stSignedByte.unsignedByte = extractByte(msg, spotToStart);
  return (stSignedByte.signedByte);
}
