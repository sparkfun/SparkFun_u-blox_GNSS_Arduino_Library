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

#ifndef __u_blox_config_keys_h__
#define __u_blox_config_keys_h__

// Define the maximum length of a multi-CfgValset construct
// "This message is limited to containing a maximum of 64 key-value pairs"
const uint8_t CFG_VALSET_MAX_KEYS = 64;

// The following consts are used to generate KEY values for the advanced protocol functions of VELGET/SET/DEL
const uint8_t VAL_SIZE_1 = 0x01;  // One bit
const uint8_t VAL_SIZE_8 = 0x02;  // One byte
const uint8_t VAL_SIZE_16 = 0x03; // Two bytes
const uint8_t VAL_SIZE_32 = 0x04; // Four bytes
const uint8_t VAL_SIZE_64 = 0x05; // Eight bytes

// These are the Bitfield layers definitions for the UBX-CFG-VALSET message (not to be confused with Bitfield deviceMask in UBX-CFG-CFG)
const uint8_t VAL_LAYER_RAM = (1 << 0);
const uint8_t VAL_LAYER_BBR = (1 << 1);
const uint8_t VAL_LAYER_FLASH = (1 << 2);
const uint8_t VAL_LAYER_ALL = VAL_LAYER_RAM | VAL_LAYER_BBR | VAL_LAYER_FLASH; // Not valid with getVal()

// Below are various Groups, IDs, and sizes for various settings
// These can be used to call getVal/setVal/delVal
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint8_t VAL_ID_PROT_UBX = 0x01;
const uint8_t VAL_ID_PROT_NMEA = 0x02;
const uint8_t VAL_ID_PROT_RTCM3 = 0x04;

const uint8_t VAL_GROUP_I2C = 0x51;
const uint8_t VAL_GROUP_I2COUTPROT = 0x72;
const uint8_t VAL_GROUP_UART1INPROT = 0x73;
const uint8_t VAL_GROUP_UART1OUTPROT = 0x74;
const uint8_t VAL_GROUP_UART2INPROT = 0x75;
const uint8_t VAL_GROUP_UART2OUTPROT = 0x76;
const uint8_t VAL_GROUP_USBINPROT = 0x77;
const uint8_t VAL_GROUP_USBOUTPROT = 0x78;

const uint8_t VAL_GROUP_UART_SIZE = VAL_SIZE_1; // All fields in UART group are currently 1 bit
const uint8_t VAL_GROUP_I2C_SIZE = VAL_SIZE_8;  // All fields in I2C group are currently 1 byte

const uint8_t VAL_ID_I2C_ADDRESS = 0x01;

// Below are the key values for a given configuration setting

// CFG-BDS: BeiDou system configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_BDS_USE_PRN_1_TO_5 = 0x10340014; // Use BeiDou geostationary satellites (PRN 1-5)

// CFG-GEOFENCE: Geofencing configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_GEOFENCE_CONFLVL = 0x20240011;    // Required confidence level for state evaluation
const uint32_t UBLOX_CFG_GEOFENCE_USE_PIO = 0x10240012;    // Use PIO combined fence state output
const uint32_t UBLOX_CFG_GEOFENCE_PINPOL = 0x20240013;     // PIO pin polarity
const uint32_t UBLOX_CFG_GEOFENCE_PIN = 0x20240014;        // PIO pin number
const uint32_t UBLOX_CFG_GEOFENCE_USE_FENCE1 = 0x10240020; // Use frst geofence
const uint32_t UBLOX_CFG_GEOFENCE_FENCE1_LAT = 0x40240021; // Latitude of the first geofence circle center
const uint32_t UBLOX_CFG_GEOFENCE_FENCE1_LON = 0x40240022; // Longitude of the first geofence circle center
const uint32_t UBLOX_CFG_GEOFENCE_FENCE1_RAD = 0x40240023; // Radius of the first geofence circle
const uint32_t UBLOX_CFG_GEOFENCE_USE_FENCE2 = 0x10240030; // Use second geofence
const uint32_t UBLOX_CFG_GEOFENCE_FENCE2_LAT = 0x40240031; // Latitude of the second geofence circle center
const uint32_t UBLOX_CFG_GEOFENCE_FENCE2_LON = 0x40240032; // Longitude of the second geofence circle center
const uint32_t UBLOX_CFG_GEOFENCE_FENCE2_RAD = 0x40240033; // Radius of the second geofence circle
const uint32_t UBLOX_CFG_GEOFENCE_USE_FENCE3 = 0x10240040; // Use third geofence
const uint32_t UBLOX_CFG_GEOFENCE_FENCE3_LAT = 0x40240041; // Latitude of the third geofence circle center
const uint32_t UBLOX_CFG_GEOFENCE_FENCE3_LON = 0x40240042; // Longitude of the third geofence circle center
const uint32_t UBLOX_CFG_GEOFENCE_FENCE3_RAD = 0x40240043; // Radius of the third geofence circle
const uint32_t UBLOX_CFG_GEOFENCE_USE_FENCE4 = 0x10240050; // Use fourth geofence
const uint32_t UBLOX_CFG_GEOFENCE_FENCE4_LAT = 0x40240051; // Latitude of the fourth geofence circle center
const uint32_t UBLOX_CFG_GEOFENCE_FENCE4_LON = 0x40240052; // Longitude of the fourth geofence circle center
const uint32_t UBLOX_CFG_GEOFENCE_FENCE4_RAD = 0x40240053; // Radius of the fourth geofence circle

// CFG-HW: Hardware configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_HW_ANT_CFG_VOLTCTRL = 0x10a3002e;     // Active antenna voltage control flag
const uint32_t UBLOX_CFG_HW_ANT_CFG_SHORTDET = 0x10a3002f;     // Short antenna detection flag
const uint32_t UBLOX_CFG_HW_ANT_CFG_SHORTDET_POL = 0x10a30030; // Short antenna detection polarity
const uint32_t UBLOX_CFG_HW_ANT_CFG_OPENDET = 0x10a30031;      // Open antenna detection flag
const uint32_t UBLOX_CFG_HW_ANT_CFG_OPENDET_POL = 0x10a30032;  // Open antenna detection polarity
const uint32_t UBLOX_CFG_HW_ANT_CFG_PWRDOWN = 0x10a30033;      // Power down antenna flag
const uint32_t UBLOX_CFG_HW_ANT_CFG_PWRDOWN_POL = 0x10a30034;  // Power down antenna logic polarity
const uint32_t UBLOX_CFG_HW_ANT_CFG_RECOVER = 0x10a30035;      // Automatic recovery from short state flag
const uint32_t UBLOX_CFG_HW_ANT_SUP_SWITCH_PIN = 0x20a30036;   // ANT1 PIO number
const uint32_t UBLOX_CFG_HW_ANT_SUP_SHORT_PIN = 0x20a30037;    // ANT0 PIO number
const uint32_t UBLOX_CFG_HW_ANT_SUP_OPEN_PIN = 0x20a30038;     // ANT2 PIO number
const uint32_t UBLOX_CFG_HW_ANT_SUP_ENGINE = 0x20a30054;       // Antenna supervisor engine selection
const uint32_t UBLOX_CFG_HW_ANT_SUP_SHORT_THR = 0x20a30055;    // Antenna supervisor MADC engine short detection threshold
const uint32_t UBLOX_CFG_HW_ANT_SUP_OPEN_THR = 0x20a30056;     // Antenna supervisor MADC engine open detection threshold

// CFG-I2C: Configuration of the I2C interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_I2C_ADDRESS = 0x20510001;         // I2C slave address of the receiver (7 bits)
const uint32_t UBLOX_CFG_I2C_EXTENDEDTIMEOUT = 0x10510002; // Flag to disable timeouting the interface after 1.5 s
const uint32_t UBLOX_CFG_I2C_ENABLED = 0x10510003;         // Flag to indicate if the I2C interface should be enabled

// CFG-I2CINPROT: Input protocol configuration of the I2C interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_I2CINPROT_UBX = 0x10710001;    // Flag to indicate if UBX should be an input protocol on I2C
const uint32_t UBLOX_CFG_I2CINPROT_NMEA = 0x10710002;   // Flag to indicate if NMEA should be an input protocol on I2C
const uint32_t UBLOX_CFG_I2CINPROT_RTCM3X = 0x10710004; // Flag to indicate if RTCM3X should be an input protocol on I2C
const uint32_t UBLOX_CFG_I2CINPROT_SPARTN = 0x10710005; // Flag to indicate if SPARTN should be an input protocol on I2C

// CFG-I2COUTPROT: Output protocol configuration of the I2C interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_I2COUTPROT_UBX = 0x10720001;    // Flag to indicate if UBX should be an output protocol on I2C
const uint32_t UBLOX_CFG_I2COUTPROT_NMEA = 0x10720002;   // Flag to indicate if NMEA should be an output protocol on I2C
const uint32_t UBLOX_CFG_I2COUTPROT_RTCM3X = 0x10720004; // Flag to indicate if RTCM3X should be an output protocol on I2C

// CFG-INFMSG: Information message configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_INFMSG_UBX_I2C = 0x20920001;    // Information message enable flags for the UBX protocol on the I2C interface
const uint32_t UBLOX_CFG_INFMSG_UBX_UART1 = 0x20920002;  // Information message enable flags for the UBX protocol on the UART1 interface
const uint32_t UBLOX_CFG_INFMSG_UBX_UART2 = 0x20920003;  // Information message enable flags for the UBX protocol on the UART2 interface
const uint32_t UBLOX_CFG_INFMSG_UBX_USB = 0x20920004;    // Information message enable flags for the UBX protocol on the USB interface
const uint32_t UBLOX_CFG_INFMSG_UBX_SPI = 0x20920005;    // Information message enable flags for the UBX protocol on the SPI interface
const uint32_t UBLOX_CFG_INFMSG_NMEA_I2C = 0x20920006;   // Information message enable flags for the NMEA protocol on the I2C interface
const uint32_t UBLOX_CFG_INFMSG_NMEA_UART1 = 0x20920007; // Information message enable flags for the NMEA protocol on the UART1 interface
const uint32_t UBLOX_CFG_INFMSG_NMEA_UART2 = 0x20920008; // Information message enable flags for the NMEA protocol on the UART2 interface
const uint32_t UBLOX_CFG_INFMSG_NMEA_USB = 0x20920009;   // Information message enable flags for the NMEA protocol on the USB interface
const uint32_t UBLOX_CFG_INFMSG_NMEA_SPI = 0x2092000a;   // Information message enable flags for the NMEA protocol on the SPI interface

// CFG-ITFM: Jamming and interference monitor configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_ITFM_BBTHRESHOLD = 0x20410001; // Broadband jamming detection threshold
const uint32_t UBLOX_CFG_ITFM_CWTHRESHOLD = 0x20410002; // CW jamming detection threshold
const uint32_t UBLOX_CFG_ITFM_ENABLE = 0x1041000d;      // Enable interference detection
const uint32_t UBLOX_CFG_ITFM_ANTSETTING = 0x20410010;  // Antenna setting
const uint32_t UBLOX_CFG_ITFM_ENABLE_AUX = 0x10410013;  // Scan auxiliary bands

// CFG-LOGFILTER: Data logger configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_LOGFILTER_RECORD_ENA = 0x10de0002;           // Recording enabled
const uint32_t UBLOX_CFG_LOGFILTER_ONCE_PER_WAKE_UP_ENA = 0x10de0003; // Once per wake up
const uint32_t UBLOX_CFG_LOGFILTER_APPLY_ALL_FILTERS = 0x10de0004;    // Apply all filter settings
const uint32_t UBLOX_CFG_LOGFILTER_MIN_INTERVAL = 0x30de0005;         // Minimum time interval between loggedpositions
const uint32_t UBLOX_CFG_LOGFILTER_TIME_THRS = 0x30de0006;            // Time threshold
const uint32_t UBLOX_CFG_LOGFILTER_SPEED_THRS = 0x30de0007;           // Speed threshold
const uint32_t UBLOX_CFG_LOGFILTER_POSITION_THRS = 0x40de0008;        // Position threshold

// CFG-MOT: Motion detector configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_MOT_GNSSSPEED_THRS = 0x20250038; // GNSS speed threshold below which platform is considered as stationary (a.k.a. static hold threshold)
const uint32_t UBLOX_CFG_MOT_GNSSDIST_THRS = 0x3025003b;  // Distance above which GNSS-based stationary motion is exit (a.k.a. static hold distance threshold)

// CFG-MSGOUT: Message output configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// For each message and port a separate output rate (per second, per epoch) can be configured.
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_DTM_I2C = 0x209100a6;          // Output rate of the NMEA-GX-DTM message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_DTM_SPI = 0x209100aa;          // Output rate of the NMEA-GX-DTM message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_DTM_UART1 = 0x209100a7;        // Output rate of the NMEA-GX-DTM message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_DTM_UART2 = 0x209100a8;        // Output rate of the NMEA-GX-DTM message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_DTM_USB = 0x209100a9;          // Output rate of the NMEA-GX-DTM message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GBS_I2C = 0x209100dd;          // Output rate of the NMEA-GX-GBS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GBS_SPI = 0x209100e1;          // Output rate of the NMEA-GX-GBS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GBS_UART1 = 0x209100de;        // Output rate of the NMEA-GX-GBS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GBS_UART2 = 0x209100df;        // Output rate of the NMEA-GX-GBS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GBS_USB = 0x209100e0;          // Output rate of the NMEA-GX-GBS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C = 0x209100ba;          // Output rate of the NMEA-GX-GGA message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GGA_SPI = 0x209100be;          // Output rate of the NMEA-GX-GGA message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1 = 0x209100bb;        // Output rate of the NMEA-GX-GGA message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART2 = 0x209100bc;        // Output rate of the NMEA-GX-GGA message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GGA_USB = 0x209100bd;          // Output rate of the NMEA-GX-GGA message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GLL_I2C = 0x209100c9;          // Output rate of the NMEA-GX-GLL message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GLL_SPI = 0x209100cd;          // Output rate of the NMEA-GX-GLL message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1 = 0x209100ca;        // Output rate of the NMEA-GX-GLL message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART2 = 0x209100cb;        // Output rate of the NMEA-GX-GLL message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GLL_USB = 0x209100cc;          // Output rate of the NMEA-GX-GLL message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GNS_I2C = 0x209100b5;          // Output rate of the NMEA-GX-GNS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GNS_SPI = 0x209100b9;          // Output rate of the NMEA-GX-GNS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GNS_UART1 = 0x209100b6;        // Output rate of the NMEA-GX-GNS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GNS_UART2 = 0x209100b7;        // Output rate of the NMEA-GX-GNS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GNS_USB = 0x209100b8;          // Output rate of the NMEA-GX-GNS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GRS_I2C = 0x209100ce;          // Output rate of the NMEA-GX-GRS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GRS_SPI = 0x209100d2;          // Output rate of the NMEA-GX-GRS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GRS_UART1 = 0x209100cf;        // Output rate of the NMEA-GX-GRS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GRS_UART2 = 0x209100d0;        // Output rate of the NMEA-GX-GRS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GRS_USB = 0x209100d1;          // Output rate of the NMEA-GX-GRS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GSA_I2C = 0x209100bf;          // Output rate of the NMEA-GX-GSA message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GSA_SPI = 0x209100c3;          // Output rate of the NMEA-GX-GSA message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1 = 0x209100c0;        // Output rate of the NMEA-GX-GSA message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART2 = 0x209100c1;        // Output rate of the NMEA-GX-GSA message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GSA_USB = 0x209100c2;          // Output rate of the NMEA-GX-GSA message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GST_I2C = 0x209100d3;          // Output rate of the NMEA-GX-GST message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GST_SPI = 0x209100d7;          // Output rate of the NMEA-GX-GST message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GST_UART1 = 0x209100d4;        // Output rate of the NMEA-GX-GST message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GST_UART2 = 0x209100d5;        // Output rate of the NMEA-GX-GST message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GST_USB = 0x209100d6;          // Output rate of the NMEA-GX-GST message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GSV_I2C = 0x209100c4;          // Output rate of the NMEA-GX-GSV message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GSV_SPI = 0x209100c8;          // Output rate of the NMEA-GX-GSV message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART1 = 0x209100c5;        // Output rate of the NMEA-GX-GSV message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART2 = 0x209100c6;        // Output rate of the NMEA-GX-GSV message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_GSV_USB = 0x209100c7;          // Output rate of the NMEA-GX-GSV message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_RLM_I2C = 0x20910400;          // Output rate of the NMEA-GX-RLM message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_RLM_SPI = 0x20910404;          // Output rate of the NMEA-GX-RLM message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_RLM_UART1 = 0x20910401;        // Output rate of the NMEA-GX-RLM message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_RLM_UART2 = 0x20910402;        // Output rate of the NMEA-GX-RLM message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_RLM_USB = 0x20910403;          // Output rate of the NMEA-GX-RLM message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_RMC_I2C = 0x209100ab;          // Output rate of the NMEA-GX-RMC message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_RMC_SPI = 0x209100af;          // Output rate of the NMEA-GX-RMC message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1 = 0x209100ac;        // Output rate of the NMEA-GX-RMC message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART2 = 0x209100ad;        // Output rate of the NMEA-GX-RMC message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_RMC_USB = 0x209100ae;          // Output rate of the NMEA-GX-RMC message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_VLW_I2C = 0x209100e7;          // Output rate of the NMEA-GX-VLW message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_VLW_SPI = 0x209100eb;          // Output rate of the NMEA-GX-VLW message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_VLW_UART1 = 0x209100e8;        // Output rate of the NMEA-GX-VLW message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_VLW_UART2 = 0x209100e9;        // Output rate of the NMEA-GX-VLW message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_VLW_USB = 0x209100ea;          // Output rate of the NMEA-GX-VLW message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_VTG_I2C = 0x209100b0;          // Output rate of the NMEA-GX-VTG message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_VTG_SPI = 0x209100b4;          // Output rate of the NMEA-GX-VTG message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1 = 0x209100b1;        // Output rate of the NMEA-GX-VTG message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART2 = 0x209100b2;        // Output rate of the NMEA-GX-VTG message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_VTG_USB = 0x209100b3;          // Output rate of the NMEA-GX-VTG message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_I2C = 0x209100d8;          // Output rate of the NMEA-GX-ZDA message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_SPI = 0x209100dc;          // Output rate of the NMEA-GX-ZDA message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_UART1 = 0x209100d9;        // Output rate of the NMEA-GX-ZDA message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_UART2 = 0x209100da;        // Output rate of the NMEA-GX-ZDA message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_USB = 0x209100db;          // Output rate of the NMEA-GX-ZDA message on port USB
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYP_I2C = 0x209100ec;        // Output rate of the NMEA-GX-PUBX00 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYP_SPI = 0x209100f0;        // Output rate of the NMEA-GX-PUBX00 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYP_UART1 = 0x209100ed;      // Output rate of the NMEA-GX-PUBX00 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYP_UART2 = 0x209100ee;      // Output rate of the NMEA-GX-PUBX00 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYP_USB = 0x209100ef;        // Output rate of the NMEA-GX-PUBX00 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYS_I2C = 0x209100f1;        // Output rate of the NMEA-GX-PUBX03 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYS_SPI = 0x209100f5;        // Output rate of the NMEA-GX-PUBX03 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYS_UART1 = 0x209100f2;      // Output rate of the NMEA-GX-PUBX03 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYS_UART2 = 0x209100f3;      // Output rate of the NMEA-GX-PUBX03 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYS_USB = 0x209100f4;        // Output rate of the NMEA-GX-PUBX03 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYT_I2C = 0x209100f6;        // Output rate of the NMEA-GX-PUBX04 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYT_SPI = 0x209100fa;        // Output rate of the NMEA-GX-PUBX04 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYT_UART1 = 0x209100f7;      // Output rate of the NMEA-GX-PUBX04 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYT_UART2 = 0x209100f8;      // Output rate of the NMEA-GX-PUBX04 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_PUBX_ID_POLYT_USB = 0x209100f9;        // Output rate of the NMEA-GX-PUBX04 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_I2C = 0x209102bd;     // Output rate of the RTCM-3X-TYPE1005 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_SPI = 0x209102c1;     // Output rate of the RTCM-3X-TYPE1005 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART1 = 0x209102be;   // Output rate of the RTCM-3X-TYPE1005 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART2 = 0x209102bf;   // Output rate of the RTCM-3X-TYPE1005 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_USB = 0x209102c0;     // Output rate of the RTCM-3X-TYPE1005 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_I2C = 0x2091035e;     // Output rate of the RTCM-3X-TYPE1074 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_SPI = 0x20910362;     // Output rate of the RTCM-3X-TYPE1074 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_UART1 = 0x2091035f;   // Output rate of the RTCM-3X-TYPE1074 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_UART2 = 0x20910360;   // Output rate of the RTCM-3X-TYPE1074 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1074_USB = 0x20910361;     // Output rate of the RTCM-3X-TYPE1074 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_I2C = 0x209102cc;     // Output rate of the RTCM-3X-TYPE1077 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_SPI = 0x209102d0;     // Output rate of the RTCM-3X-TYPE1077 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART1 = 0x209102cd;   // Output rate of the RTCM-3X-TYPE1077 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART2 = 0x209102ce;   // Output rate of the RTCM-3X-TYPE1077 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_USB = 0x209102cf;     // Output rate of the RTCM-3X-TYPE1077 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_I2C = 0x20910363;     // Output rate of the RTCM-3X-TYPE1084 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_SPI = 0x20910367;     // Output rate of the RTCM-3X-TYPE1084 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_UART1 = 0x20910364;   // Output rate of the RTCM-3X-TYPE1084 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_UART2 = 0x20910365;   // Output rate of the RTCM-3X-TYPE1084 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1084_USB = 0x20910366;     // Output rate of the RTCM-3X-TYPE1084 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_I2C = 0x209102d1;     // Output rate of the RTCM-3X-TYPE1087 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_SPI = 0x209102d5;     // Output rate of the RTCM-3X-TYPE1087 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART1 = 0x209102d2;   // Output rate of the RTCM-3X-TYPE1087 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART2 = 0x209102d3;   // Output rate of the RTCM-3X-TYPE1087 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_USB = 0x209102d4;     // Output rate of the RTCM-3X-TYPE1087 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_I2C = 0x20910368;     // Output rate of the RTCM-3X-TYPE1094 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_SPI = 0x2091036c;     // Output rate of the RTCM-3X-TYPE1094 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_UART1 = 0x20910369;   // Output rate of the RTCM-3X-TYPE1094 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_UART2 = 0x2091036a;   // Output rate of the RTCM-3X-TYPE1094 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1094_USB = 0x2091036b;     // Output rate of the RTCM-3X-TYPE1094 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_I2C = 0x20910318;     // Output rate of the RTCM-3X-TYPE1097 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_SPI = 0x2091031c;     // Output rate of the RTCM-3X-TYPE1097 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART1 = 0x20910319;   // Output rate of the RTCM-3X-TYPE1097 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART2 = 0x2091031a;   // Output rate of the RTCM-3X-TYPE1097 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_USB = 0x2091031b;     // Output rate of the RTCM-3X-TYPE1097 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_I2C = 0x2091036d;     // Output rate of the RTCM-3X-TYPE1124 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_SPI = 0x20910371;     // Output rate of the RTCM-3X-TYPE1124 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_UART1 = 0x2091036e;   // Output rate of the RTCM-3X-TYPE1124 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_UART2 = 0x2091036f;   // Output rate of the RTCM-3X-TYPE1124 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1124_USB = 0x20910370;     // Output rate of the RTCM-3X-TYPE1124 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_I2C = 0x209102d6;     // Output rate of the RTCM-3X-TYPE1127 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_SPI = 0x209102da;     // Output rate of the RTCM-3X-TYPE1127 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART1 = 0x209102d7;   // Output rate of the RTCM-3X-TYPE1127 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART2 = 0x209102d8;   // Output rate of the RTCM-3X-TYPE1127 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_USB = 0x209102d9;     // Output rate of the RTCM-3X-TYPE1127 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_I2C = 0x20910303;     // Output rate of the RTCM-3X-TYPE1230 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_SPI = 0x20910307;     // Output rate of the RTCM-3X-TYPE1230 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART1 = 0x20910304;   // Output rate of the RTCM-3X-TYPE1230 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART2 = 0x20910305;   // Output rate of the RTCM-3X-TYPE1230 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_USB = 0x20910306;     // Output rate of the RTCM-3X-TYPE1230 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_I2C = 0x209102fe;   // Output rate of the RTCM-3X-TYPE4072_0 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_SPI = 0x20910302;   // Output rate of the RTCM-3X-TYPE4072_0 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART1 = 0x209102ff; // Output rate of the RTCM-3X-TYPE4072_0 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_UART2 = 0x20910300; // Output rate of the RTCM-3X-TYPE4072_0 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_USB = 0x20910301;   // Output rate of the RTCM-3X-TYPE4072_0 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_1_I2C = 0x20910381;   // Output rate of the RTCM-3X-TYPE4072_1 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_1_SPI = 0x20910385;   // Output rate of the RTCM-3X-TYPE4072_1 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART1 = 0x20910382; // Output rate of the RTCM-3X-TYPE4072_1 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_1_UART2 = 0x20910383; // Output rate of the RTCM-3X-TYPE4072_1 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_1_USB = 0x20910384;   // Output rate of the RTCM-3X-TYPE4072_1 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_LOG_INFO_I2C = 0x20910259;         // Output rate of the UBX-LOG-INFO message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_LOG_INFO_SPI = 0x2091025d;         // Output rate of the UBX-LOG-INFO message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_LOG_INFO_UART1 = 0x2091025a;       // Output rate of the UBX-LOG-INFO message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_LOG_INFO_UART2 = 0x2091025b;       // Output rate of the UBX-LOG-INFO message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_LOG_INFO_USB = 0x2091025c;         // Output rate of the UBX-LOG-INFO message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_COMMS_I2C = 0x2091034f;        // Output rate of the UBX-MON-COMMS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_COMMS_SPI = 0x20910353;        // Output rate of the UBX-MON-COMMS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_COMMS_UART1 = 0x20910350;      // Output rate of the UBX-MON-COMMS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_COMMS_UART2 = 0x20910351;      // Output rate of the UBX-MON-COMMS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_COMMS_USB = 0x20910352;        // Output rate of the UBX-MON-COMMS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW2_I2C = 0x209101b9;          // Output rate of the UBX-MON-HW2 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW2_SPI = 0x209101bd;          // Output rate of the UBX-MON-HW2 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW2_UART1 = 0x209101ba;        // Output rate of the UBX-MON-HW2 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW2_UART2 = 0x209101bb;        // Output rate of the UBX-MON-HW2 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW2_USB = 0x209101bc;          // Output rate of the UBX-MON-HW2 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW3_I2C = 0x20910354;          // Output rate of the UBX-MON-HW3 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW3_SPI = 0x20910358;          // Output rate of the UBX-MON-HW3 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW3_UART1 = 0x20910355;        // Output rate of the UBX-MON-HW3 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW3_UART2 = 0x20910356;        // Output rate of the UBX-MON-HW3 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW3_USB = 0x20910357;          // Output rate of the UBX-MON-HW3 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW_I2C = 0x209101b4;           // Output rate of the UBX-MON-HW message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW_SPI = 0x209101b8;           // Output rate of the UBX-MON-HW message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW_UART1 = 0x209101b5;         // Output rate of the UBX-MON-HW message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW_UART2 = 0x209101b6;         // Output rate of the UBX-MON-HW message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_HW_USB = 0x209101b7;           // Output rate of the UBX-MON-HW message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_IO_I2C = 0x209101a5;           // Output rate of the UBX-MON-IO message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_IO_SPI = 0x209101a9;           // Output rate of the UBX-MON-IO message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_IO_UART1 = 0x209101a6;         // Output rate of the UBX-MON-IO message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_IO_UART2 = 0x209101a7;         // Output rate of the UBX-MON-IO message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_IO_USB = 0x209101a8;           // Output rate of the UBX-MON-IO message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_MSGPP_I2C = 0x20910196;        // Output rate of the UBX-MON-MSGPP message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_MSGPP_SPI = 0x2091019a;        // Output rate of the UBX-MON-MSGPP message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_MSGPP_UART1 = 0x20910197;      // Output rate of the UBX-MON-MSGPP message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_MSGPP_UART2 = 0x20910198;      // Output rate of the UBX-MON-MSGPP message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_MSGPP_USB = 0x20910199;        // Output rate of the UBX-MON-MSGPP message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RF_I2C = 0x20910359;           // Output rate of the UBX-MON-RF message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RF_SPI = 0x2091035d;           // Output rate of the UBX-MON-RF message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RF_UART1 = 0x2091035a;         // Output rate of the UBX-MON-RF message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RF_UART2 = 0x2091035b;         // Output rate of the UBX-MON-RF message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RF_USB = 0x2091035c;           // Output rate of the UBX-MON-RF message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RXBUF_I2C = 0x209101a0;        // Output rate of the UBX-MON-RXBUF message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RXBUF_SPI = 0x209101a4;        // Output rate of the UBX-MON-RXBUF message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RXBUF_UART1 = 0x209101a1;      // Output rate of the UBX-MON-RXBUF message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RXBUF_UART2 = 0x209101a2;      // Output rate of the UBX-MON-RXBUF message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RXBUF_USB = 0x209101a3;        // Output rate of the UBX-MON-RXBUF message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RXR_I2C = 0x20910187;          // Output rate of the UBX-MON-RXR message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RXR_SPI = 0x2091018b;          // Output rate of the UBX-MON-RXR message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RXR_UART1 = 0x20910188;        // Output rate of the UBX-MON-RXR message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RXR_UART2 = 0x20910189;        // Output rate of the UBX-MON-RXR message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_RXR_USB = 0x2091018a;          // Output rate of the UBX-MON-RXR message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_SPAN_I2C = 0x2091038b;         // Output rate of the UBX-MON-SPAN message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_SPAN_SPI = 0x2091038f;         // Output rate of the UBX-MON-SPAN message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_SPAN_UART1 = 0x2091038c;       // Output rate of the UBX-MON-SPAN message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_SPAN_UART2 = 0x2091038d;       // Output rate of the UBX-MON-SPAN message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_SPAN_USB = 0x2091038e;         // Output rate of the UBX-MON-SPAN message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_SYS_I2C = 0x2091069d;          // Output rate of the UBX-MON-SYS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_SYS_SPI = 0x209106a1;          // Output rate of the UBX-MON-SYS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_SYS_UART1 = 0x2091069e;        // Output rate of the UBX-MON-SYS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_SYS_UART2 = 0x2091069f;        // Output rate of the UBX-MON-SYS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_SYS_USB = 0x209106a0;          // Output rate of the UBX-MON-SYS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_TXBUF_I2C = 0x2091019b;        // Output rate of the UBX-MON-TXBUF message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_TXBUF_SPI = 0x2091019f;        // Output rate of the UBX-MON-TXBUF message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_TXBUF_UART1 = 0x2091019c;      // Output rate of the UBX-MON-TXBUF message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_TXBUF_UART2 = 0x2091019d;      // Output rate of the UBX-MON-TXBUF message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_TXBUF_USB = 0x2091019e;        // Output rate of the UBX-MON-TXBUF message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ATT_I2C = 0x2091001f;          // Output rate of the UBX_NAV_ATT message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ATT_SPI = 0x20910023;          // Output rate of the UBX_NAV_ATT message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ATT_UART1 = 0x20910020;        // Output rate of the UBX_NAV_ATT message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ATT_UART2 = 0x20910021;        // Output rate of the UBX_NAV_ATT message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ATT_USB = 0x20910022;          // Output rate of the UBX_NAV_ATT message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_I2C = 0x20910065;        // Output rate of the UBX-NAV-CLOCK message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_SPI = 0x20910069;        // Output rate of the UBX-NAV-CLOCK message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_UART1 = 0x20910066;      // Output rate of the UBX-NAV-CLOCK message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_UART2 = 0x20910067;      // Output rate of the UBX-NAV-CLOCK message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_USB = 0x20910068;        // Output rate of the UBX-NAV-CLOCK message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_DOP_I2C = 0x20910038;          // Output rate of the UBX-NAV-DOP message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_DOP_SPI = 0x2091003c;          // Output rate of the UBX-NAV-DOP message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_DOP_UART1 = 0x20910039;        // Output rate of the UBX-NAV-DOP message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_DOP_UART2 = 0x2091003a;        // Output rate of the UBX-NAV-DOP message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_DOP_USB = 0x2091003b;          // Output rate of the UBX-NAV-DOP message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_EOE_I2C = 0x2091015f;          // Output rate of the UBX-NAV-EOE message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_EOE_SPI = 0x20910163;          // Output rate of the UBX-NAV-EOE message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_EOE_UART1 = 0x20910160;        // Output rate of the UBX-NAV-EOE message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_EOE_UART2 = 0x20910161;        // Output rate of the UBX-NAV-EOE message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_EOE_USB = 0x20910162;          // Output rate of the UBX-NAV-EOE message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_GEOFENCE_I2C = 0x209100a1;     // Output rate of the UBX-NAV-GEOFENCE message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_GEOFENCE_SPI = 0x209100a5;     // Output rate of the UBX-NAV-GEOFENCE message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_GEOFENCE_UART1 = 0x209100a2;   // Output rate of the UBX-NAV-GEOFENCE message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_GEOFENCE_UART2 = 0x209100a3;   // Output rate of the UBX-NAV-GEOFENCE message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_GEOFENCE_USB = 0x209100a4;     // Output rate of the UBX-NAV-GEOFENCE message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_I2C = 0x2091002e;    // Output rate of the UBX-NAV-HPPOSECEF message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_SPI = 0x20910032;    // Output rate of the UBX-NAV-HPPOSECEF message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART1 = 0x2091002f;  // Output rate of the UBX-NAV-HPPOSECEF message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_UART2 = 0x20910030;  // Output rate of the UBX-NAV-HPPOSECEF message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB = 0x20910031;    // Output rate of the UBX-NAV-HPPOSECEF message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_I2C = 0x20910033;     // Output rate of the UBX-NAV-HPPOSLLH message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_SPI = 0x20910037;     // Output rate of the UBX-NAV-HPPOSLLH message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART1 = 0x20910034;   // Output rate of the UBX-NAV-HPPOSLLH message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_UART2 = 0x20910035;   // Output rate of the UBX-NAV-HPPOSLLH message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB = 0x20910036;     // Output rate of the UBX-NAV-HPPOSLLH message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ODO_I2C = 0x2091007e;          // Output rate of the UBX-NAV-ODO message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ODO_SPI = 0x20910082;          // Output rate of the UBX-NAV-ODO message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ODO_UART1 = 0x2091007f;        // Output rate of the UBX-NAV-ODO message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ODO_UART2 = 0x20910080;        // Output rate of the UBX-NAV-ODO message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ODO_USB = 0x20910081;          // Output rate of the UBX-NAV-ODO message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ORB_I2C = 0x20910010;          // Output rate of the UBX-NAV-ORB message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ORB_SPI = 0x20910014;          // Output rate of the UBX-NAV-ORB message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ORB_UART1 = 0x20910011;        // Output rate of the UBX-NAV-ORB message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ORB_UART2 = 0x20910012;        // Output rate of the UBX-NAV-ORB message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_ORB_USB = 0x20910013;          // Output rate of the UBX-NAV-ORB message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PL_I2C = 0x20910415;           // Output rate of the UBX-NAV-PL message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PL_SPI = 0x20910419;           // Output rate of the UBX-NAV-PL message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PL_UART1 = 0x20910416;         // Output rate of the UBX-NAV-PL message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PL_UART2 = 0x20910417;         // Output rate of the UBX-NAV-PL message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PL_USB = 0x20910418;           // Output rate of the UBX-NAV-PL message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_POSECEF_I2C = 0x20910024;      // Output rate of the UBX-NAV-POSECEF message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_POSECEF_SPI = 0x20910028;      // Output rate of the UBX-NAV-POSECEF message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_POSECEF_UART1 = 0x20910025;    // Output rate of the UBX-NAV-POSECEF message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_POSECEF_UART2 = 0x20910026;    // Output rate of the UBX-NAV-POSECEF message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_POSECEF_USB = 0x20910027;      // Output rate of the UBX-NAV-POSECEF message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_POSLLH_I2C = 0x20910029;       // Output rate of the UBX-NAV-POSLLH message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_POSLLH_SPI = 0x2091002d;       // Output rate of the UBX-NAV-POSLLH message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_POSLLH_UART1 = 0x2091002a;     // Output rate of the UBX-NAV-POSLLH message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_POSLLH_UART2 = 0x2091002b;     // Output rate of the UBX-NAV-POSLLH message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_POSLLH_USB = 0x2091002c;       // Output rate of the UBX-NAV-POSLLH message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVT_I2C = 0x20910006;          // Output rate of the UBX-NAV-PVT message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVT_SPI = 0x2091000a;          // Output rate of the UBX-NAV-PVT message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1 = 0x20910007;        // Output rate of the UBX-NAV-PVT message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART2 = 0x20910008;        // Output rate of the UBX-NAV-PVT message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVT_USB = 0x20910009;          // Output rate of the UBX-NAV-PVT message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_I2C = 0x2091008d;    // Output rate of the UBX-NAV-RELPOSNED message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_SPI = 0x20910091;    // Output rate of the UBX-NAV-RELPOSNED message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_UART1 = 0x2091008e;  // Output rate of the UBX-NAV-RELPOSNED message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_UART2 = 0x2091008f;  // Output rate of the UBX-NAV-RELPOSNED message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_RELPOSNED_USB = 0x20910090;    // Output rate of the UBX-NAV-RELPOSNED message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SAT_I2C = 0x20910015;          // Output rate of the UBX-NAV-SAT message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SAT_SPI = 0x20910019;          // Output rate of the UBX-NAV-SAT message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SAT_UART1 = 0x20910016;        // Output rate of the UBX-NAV-SAT message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SAT_UART2 = 0x20910017;        // Output rate of the UBX-NAV-SAT message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SAT_USB = 0x20910018;          // Output rate of the UBX-NAV-SAT message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SBAS_I2C = 0x2091006a;         // Output rate of the UBX-NAV-SBAS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SBAS_SPI = 0x2091006e;         // Output rate of the UBX-NAV-SBAS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SBAS_UART1 = 0x2091006b;       // Output rate of the UBX-NAV-SBAS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SBAS_UART2 = 0x2091006c;       // Output rate of the UBX-NAV-SBAS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SBAS_USB = 0x2091006d;         // Output rate of the UBX-NAV-SBAS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SIG_I2C = 0x20910345;          // Output rate of the UBX-NAV-SIG message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SIG_SPI = 0x20910349;          // Output rate of the UBX-NAV-SIG message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SIG_UART1 = 0x20910346;        // Output rate of the UBX-NAV-SIG message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SIG_UART2 = 0x20910347;        // Output rate of the UBX-NAV-SIG message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SIG_USB = 0x20910348;          // Output rate of the UBX-NAV-SIG message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SLAS_I2C = 0x20910336;         // Output rate of the UBX-NAV-SLAS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SLAS_SPI = 0x2091033a;         // Output rate of the UBX-NAV-SLAS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SLAS_UART1 = 0x20910337;       // Output rate of the UBX-NAV-SLAS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SLAS_UART2 = 0x20910338;       // Output rate of the UBX-NAV-SLAS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SLAS_USB = 0x20910339;         // Output rate of the UBX-NAV-SLAS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_STATUS_I2C = 0x2091001a;       // Output rate of the UBX-NAV-STATUS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_STATUS_SPI = 0x2091001e;       // Output rate of the UBX-NAV-STATUS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_STATUS_UART1 = 0x2091001b;     // Output rate of the UBX-NAV-STATUS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_STATUS_UART2 = 0x2091001c;     // Output rate of the UBX-NAV-STATUS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_STATUS_USB = 0x2091001d;       // Output rate of the UBX-NAV-STATUS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_I2C = 0x20910088;         // Output rate of the UBX-NAV-SVIN message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_SPI = 0x2091008c;         // Output rate of the UBX-NAV-SVIN message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_UART1 = 0x20910089;       // Output rate of the UBX-NAV-SVIN message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_UART2 = 0x2091008a;       // Output rate of the UBX-NAV-SVIN message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_USB = 0x2091008b;         // Output rate of the UBX-NAV-SVIN message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEBDS_I2C = 0x20910051;      // Output rate of the UBX-NAV-TIMEBDS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEBDS_SPI = 0x20910055;      // Output rate of the UBX-NAV-TIMEBDS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEBDS_UART1 = 0x20910052;    // Output rate of the UBX-NAV-TIMEBDS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEBDS_UART2 = 0x20910053;    // Output rate of the UBX-NAV-TIMEBDS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEBDS_USB = 0x20910054;      // Output rate of the UBX-NAV-TIMEBDS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGAL_I2C = 0x20910056;      // Output rate of the UBX-NAV-TIMEGAL message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGAL_SPI = 0x2091005a;      // Output rate of the UBX-NAV-TIMEGAL message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGAL_UART1 = 0x20910057;    // Output rate of the UBX-NAV-TIMEGAL message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGAL_UART2 = 0x20910058;    // Output rate of the UBX-NAV-TIMEGAL message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGAL_USB = 0x20910059;      // Output rate of the UBX-NAV-TIMEGAL message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGLO_I2C = 0x2091004c;      // Output rate of the UBX-NAV-TIMEGLO message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGLO_SPI = 0x20910050;      // Output rate of the UBX-NAV-TIMEGLO message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGLO_UART1 = 0x2091004d;    // Output rate of the UBX-NAV-TIMEGLO message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGLO_UART2 = 0x2091004e;    // Output rate of the UBX-NAV-TIMEGLO message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGLO_USB = 0x2091004f;      // Output rate of the UBX-NAV-TIMEGLO message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGPS_I2C = 0x20910047;      // Output rate of the UBX-NAV-TIMEGPS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGPS_SPI = 0x2091004b;      // Output rate of the UBX-NAV-TIMEGPS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1 = 0x20910048;    // Output rate of the UBX-NAV-TIMEGPS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGPS_UART2 = 0x20910049;    // Output rate of the UBX-NAV-TIMEGPS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGPS_USB = 0x2091004a;      // Output rate of the UBX-NAV-TIMEGPS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMELS_I2C = 0x20910060;       // Output rate of the UBX-NAV-TIMELS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMELS_SPI = 0x20910064;       // Output rate of the UBX-NAV-TIMELS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMELS_UART1 = 0x20910061;     // Output rate of the UBX-NAV-TIMELS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMELS_UART2 = 0x20910062;     // Output rate of the UBX-NAV-TIMELS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMELS_USB = 0x20910063;       // Output rate of the UBX-NAV-TIMELS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_I2C = 0x20910386;     // Output rate of the UBX-NAV-TIMEQZSSmessage on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_SPI = 0x2091038a;     // Output rate of the UBX-NAV-TIMEQZSSmessage on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_UART1 = 0x20910387;   // Output rate of the UBX-NAV-TIMEQZSSmessage on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_UART2 = 0x20910388;   // Output rate of the UBX-NAV-TIMEQZSSmessage on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_USB = 0x20910389;     // Output rate of the UBX-NAV-TIMEQZSSmessage on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_I2C = 0x2091005b;      // Output rate of the UBX-NAV-TIMEUTC message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_SPI = 0x2091005f;      // Output rate of the UBX-NAV-TIMEUTC message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART1 = 0x2091005c;    // Output rate of the UBX-NAV-TIMEUTC message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART2 = 0x2091005d;    // Output rate of the UBX-NAV-TIMEUTC message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_USB = 0x2091005e;      // Output rate of the UBX-NAV-TIMEUTC message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_VELECEF_I2C = 0x2091003d;      // Output rate of the UBX-NAV-VELECEF message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_VELECEF_SPI = 0x20910041;      // Output rate of the UBX-NAV-VELECEF message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_VELECEF_UART1 = 0x2091003e;    // Output rate of the UBX-NAV-VELECEF message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_VELECEF_UART2 = 0x2091003f;    // Output rate of the UBX-NAV-VELECEF message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_VELECEF_USB = 0x20910040;      // Output rate of the UBX-NAV-VELECEF message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_VELNED_I2C = 0x20910042;       // Output rate of the UBX-NAV-VELNED message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_VELNED_SPI = 0x20910046;       // Output rate of the UBX-NAV-VELNED message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_VELNED_UART1 = 0x20910043;     // Output rate of the UBX-NAV-VELNED message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_VELNED_UART2 = 0x20910044;     // Output rate of the UBX-NAV-VELNED message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_VELNED_USB = 0x20910045;       // Output rate of the UBX-NAV-VELNED message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_COR_I2C = 0x209106b6;          // Output rate of the UBX-RXM-COR message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_COR_SPI = 0x209106ba;          // Output rate of the UBX-RXM-COR message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_COR_UART1 = 0x209106b7;        // Output rate of the UBX-RXM-COR message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_COR_UART2 = 0x209106b8;        // Output rate of the UBX-RXM-COR message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_COR_USB = 0x209106b9;          // Output rate of the UBX-RXM-COR message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_I2C = 0x20910204;        // Output rate of the UBX-RXM-MEASX message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_SPI = 0x20910208;        // Output rate of the UBX-RXM-MEASX message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_UART1 = 0x20910205;      // Output rate of the UBX-RXM-MEASX message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_UART2 = 0x20910206;      // Output rate of the UBX-RXM-MEASX message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_MEASX_USB = 0x20910207;        // Output rate of the UBX-RXM-MEASX message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_I2C = 0x209102a4;         // Output rate of the UBX-RXM-RAWX message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_SPI = 0x209102a8;         // Output rate of the UBX-RXM-RAWX message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_UART1 = 0x209102a5;       // Output rate of the UBX-RXM-RAWX message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_UART2 = 0x209102a6;       // Output rate of the UBX-RXM-RAWX message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_USB = 0x209102a7;         // Output rate of the UBX-RXM-RAWX message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RLM_I2C = 0x2091025e;          // Output rate of the UBX-RXM-RLM message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RLM_SPI = 0x20910262;          // Output rate of the UBX-RXM-RLM message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RLM_UART1 = 0x2091025f;        // Output rate of the UBX-RXM-RLM message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RLM_UART2 = 0x20910260;        // Output rate of the UBX-RXM-RLM message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RLM_USB = 0x20910261;          // Output rate of the UBX-RXM-RLM message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RTCM_I2C = 0x20910268;         // Output rate of the UBX-RXM-RTCM message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RTCM_SPI = 0x2091026c;         // Output rate of the UBX-RXM-RTCM message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RTCM_UART1 = 0x20910269;       // Output rate of the UBX-RXM-RTCM message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RTCM_UART2 = 0x2091026a;       // Output rate of the UBX-RXM-RTCM message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_RTCM_USB = 0x2091026b;         // Output rate of the UBX-RXM-RTCM message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_I2C = 0x20910231;        // Output rate of the UBX-RXM-SFRBX message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_SPI = 0x20910235;        // Output rate of the UBX-RXM-SFRBX message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_UART1 = 0x20910232;      // Output rate of the UBX-RXM-SFRBX message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_UART2 = 0x20910233;      // Output rate of the UBX-RXM-SFRBX message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_USB = 0x20910234;        // Output rate of the UBX-RXM-SFRBX message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_SPARTN_I2C = 0x20910605;       // Output rate of the UBX-RXM-SPARTN message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_SPARTN_UART1 = 0x20910606;     // Output rate of the UBX-RXM-SPARTN message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_SPARTN_UART2 = 0x20910607;     // Output rate of the UBX-RXM-SPARTN message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_SPARTN_USB = 0x20910608;       // Output rate of the UBX-RXM-SPARTN message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_SPARTN_SPI = 0x20910609;       // Output rate of the UBX-RXM-SPARTN message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_SEC_SIG_I2C = 0x20910634;          // Output rate of the UBX-SEC-SIG message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_SEC_SIG_SPI = 0x20910638;          // Output rate of the UBX-SEC-SIG message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_SEC_SIG_UART1 = 0x20910635;        // Output rate of the UBX-SEC-SIG message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_SEC_SIG_UART2 = 0x20910636;        // Output rate of the UBX-SEC-SIG message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_SEC_SIG_USB = 0x20910637;          // Output rate of the UBX-SEC-SIG message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_TM2_I2C = 0x20910178;          // Output rate of the UBX-TIM-TM2 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_TM2_SPI = 0x2091017c;          // Output rate of the UBX-TIM-TM2 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_TM2_UART1 = 0x20910179;        // Output rate of the UBX-TIM-TM2 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_TM2_UART2 = 0x2091017a;        // Output rate of the UBX-TIM-TM2 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_TM2_USB = 0x2091017b;          // Output rate of the UBX-TIM-TM2 message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_TP_I2C = 0x2091017d;           // Output rate of the UBX-TIM-TP message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_TP_SPI = 0x20910181;           // Output rate of the UBX-TIM-TP message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_TP_UART1 = 0x2091017e;         // Output rate of the UBX-TIM-TP message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_TP_UART2 = 0x2091017f;         // Output rate of the UBX-TIM-TP message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_TP_USB = 0x20910180;           // Output rate of the UBX-TIM-TP message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_VRFY_I2C = 0x20910092;         // Output rate of the UBX-TIM-VRFY message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_VRFY_SPI = 0x20910096;         // Output rate of the UBX-TIM-VRFY message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_VRFY_UART1 = 0x20910093;       // Output rate of the UBX-TIM-VRFY message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_VRFY_UART2 = 0x20910094;       // Output rate of the UBX-TIM-VRFY message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_VRFY_USB = 0x20910095;         // Output rate of the UBX-TIM-VRFY message on port USB

// Additional CFG_MSGOUT keys for the ZED-F9R HPS121
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_COV_I2C = 0x20910083;      // Output rate of the UBX-NAV-COV message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_COV_UART1 = 0x20910084;    // Output rate of the UBX-NAV-COV message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_COV_UART2 = 0x20910085;    // Output rate of the UBX-NAV-COV message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_COV_USB = 0x20910086;      // Output rate of the UBX-NAV-COV message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_COV_SPI = 0x20910087;      // Output rate of the UBX-NAV-COV message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_THS_I2C = 0x209100e2;      // Output rate of the NMEA-GX-THS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_THS_UART1 = 0x209100e3;    // Output rate of the NMEA-GX-THS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_THS_UART2 = 0x209100e4;    // Output rate of the NMEA-GX-THS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_THS_USB = 0x209100e5;      // Output rate of the NMEA-GX-THS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_ID_THS_SPI = 0x209100e6;      // Output rate of the NMEA-GX-THS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_STATUS_I2C = 0x20910105;   // Output rate of the UBX-ESF-STATUS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_STATUS_UART1 = 0x20910106; // Output rate of the UBX-ESF-STATUS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_STATUS_UART2 = 0x20910107; // Output rate of the UBX-ESF-STATUS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_STATUS_USB = 0x20910108;   // Output rate of the UBX-ESF-STATUS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_STATUS_SPI = 0x20910109;   // Output rate of the UBX-ESF-STATUS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_ALG_I2C = 0x2091010f;      // Output rate of the UBX-ESF-ALG message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_ALG_UART1 = 0x20910110;    // Output rate of the UBX-ESF-ALG message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_ALG_UART2 = 0x20910111;    // Output rate of the UBX-ESF-ALG message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_ALG_USB = 0x20910112;      // Output rate of the UBX-ESF-ALG message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_ALG_SPI = 0x20910113;      // Output rate of the UBX-ESF-ALG message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_INS_I2C = 0x20910114;      // Output rate of the UBX-ESF-INS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_INS_UART1 = 0x20910115;    // Output rate of the UBX-ESF-INS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_INS_UART2 = 0x20910116;    // Output rate of the UBX-ESF-INS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_INS_USB = 0x20910117;      // Output rate of the UBX-ESF-INS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_INS_SPI = 0x20910118;      // Output rate of the UBX-ESF-INS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_MEAS_I2C = 0x20910277;     // Output rate of the UBX-ESF-MEAS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_MEAS_UART1 = 0x20910278;   // Output rate of the UBX-ESF-MEAS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_MEAS_UART2 = 0x20910279;   // Output rate of the UBX-ESF-MEAS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_MEAS_USB = 0x2091027a;     // Output rate of the UBX-ESF-MEAS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_MEAS_SPI = 0x2091027b;     // Output rate of the UBX-ESF-MEAS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_RAW_I2C = 0x2091029f;      // Output rate of the UBX-ESF-RAW message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_RAW_UART1 = 0x209102a0;    // Output rate of the UBX-ESF-RAW message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_RAW_UART2 = 0x209102a1;    // Output rate of the UBX-ESF-RAW message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_RAW_USB = 0x209102a2;      // Output rate of the UBX-ESF-RAW message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_ESF_RAW_SPI = 0x209102a3;      // Output rate of the UBX-ESF-RAW message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_EELL_I2C = 0x20910313;     // Output rate of the UBX-NAV-EELL message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_EELL_UART1 = 0x20910314;   // Output rate of the UBX-NAV-EELL message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_EELL_UART2 = 0x20910315;   // Output rate of the UBX-NAV-EELL message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_EELL_USB = 0x20910316;     // Output rate of the UBX-NAV-EELL message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_EELL_SPI = 0x20910317;     // Output rate of the UBX-NAV-EELL message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_I2C = 0x2091062a;     // Output rate of the UBX-NAV-PVAT message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_UART1 = 0x2091062b;   // Output rate of the UBX-NAV-PVAT message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_UART2 = 0x2091062c;   // Output rate of the UBX-NAV-PVAT message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_USB = 0x2091062d;     // Output rate of the UBX-NAV-PVAT message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_PVAT_SPI = 0x2091062e;     // Output rate of the UBX-NAV-PVAT message on port SPI

// Additional CFG_MSGOUT keys for the ZED-F9T
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GGA_I2C = 0x20910661;    // Output rate of the NMEA-NAV2-GX-GGA message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GGA_SPI = 0x20910665;    // Output rate of the NMEA-NAV2-GX-GGA message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GGA_UART1 = 0x20910662;  // Output rate of the NMEA-NAV2-GX-GGA message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GGA_UART2 = 0x20910663;  // Output rate of the NMEA-NAV2-GX-GGA message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GGA_USB = 0x20910664;    // Output rate of the NMEA-NAV2-GX-GGA message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GLL_I2C = 0x20910670;    // Output rate of the NMEA-NAV2-GX-GLL message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GLL_SPI = 0x20910674;    // Output rate of the NMEA-NAV2-GX-GLL message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GLL_UART1 = 0x20910671;  // Output rate of the NMEA-NAV2-GX-GLL message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GLL_UART2 = 0x20910672;  // Output rate of the NMEA-NAV2-GX-GLL message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GLL_USB = 0x20910673;    // Output rate of the NMEA-NAV2-GX-GLL message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GNS_I2C = 0x2091065c;    // Output rate of the NMEA-NAV2-GX-GNS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GNS_SPI = 0x20910660;    // Output rate of the NMEA-NAV2-GX-GNS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GNS_UART1 = 0x2091065d;  // Output rate of the NMEA-NAV2-GX-GNS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GNS_UART2 = 0x2091065e;  // Output rate of the NMEA-NAV2-GX-GNS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GNS_USB = 0x2091065f;    // Output rate of the NMEA-NAV2-GX-GNS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GSA_I2C = 0x20910666;    // Output rate of the NMEA-NAV2-GX-GSA message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GSA_SPI = 0x2091066a;    // Output rate of the NMEA-NAV2-GX-GSA message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GSA_UART1 = 0x20910667;  // Output rate of the NMEA-NAV2-GX-GSA message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GSA_UART2 = 0x20910668;  // Output rate of the NMEA-NAV2-GX-GSA message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_GSA_USB = 0x20910669;    // Output rate of the NMEA-NAV2-GX-GSA message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_RMC_I2C = 0x20910652;    // Output rate of the NMEA-NAV2-GX-RMC message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_RMC_SPI = 0x20910656;    // Output rate of the NMEA-NAV2-GX-RMC message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_RMC_UART1 = 0x20910653;  // Output rate of the NMEA-NAV2-GX-RMC message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_RMC_UART2 = 0x20910654;  // Output rate of the NMEA-NAV2-GX-RMC message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_RMC_USB = 0x20910655;    // Output rate of the NMEA-NAV2-GX-RMC message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_VTG_I2C = 0x20910657;    // Output rate of the NMEA-NAV2-GX-VTG message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_VTG_SPI = 0x2091065b;    // Output rate of the NMEA-NAV2-GX-VTG message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_VTG_UART1 = 0x20910658;  // Output rate of the NMEA-NAV2-GX-VTG message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_VTG_UART2 = 0x20910659;  // Output rate of the NMEA-NAV2-GX-VTG message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_VTG_USB = 0x2091065a;    // Output rate of the NMEA-NAV2-GX-VTG message on port USB
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_ZDA_I2C = 0x2091067f;    // Output rate of the NMEA-NAV2-GX-ZDA message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_ZDA_SPI = 0x20910683;    // Output rate of the NMEA-NAV2-GX-ZDA message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_ZDA_UART1 = 0x20910680;  // Output rate of the NMEA-NAV2-GX-ZDA message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_ZDA_UART2 = 0x20910681;  // Output rate of the NMEA-NAV2-GX-ZDA message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_NMEA_NAV2_ID_ZDA_USB = 0x20910682;    // Output rate of the NMEA-NAV2-GX-ZDA message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_CLOCK_I2C = 0x20910430;      // Output rate of the UBX-NAV2-CLOCK message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_CLOCK_SPI = 0x20910434;      // Output rate of the UBX-NAV2-CLOCK message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_CLOCK_UART1 = 0x20910431;    // Output rate of the UBX-NAV2-CLOCK message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_CLOCK_UART2 = 0x20910432;    // Output rate of the UBX-NAV2-CLOCK message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_CLOCK_USB = 0x20910433;      // Output rate of the UBX-NAV2-CLOCK message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_COV_I2C = 0x20910435;        // Output rate of the UBX-NAV2-COV message onport I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_COV_SPI = 0x20910439;        // Output rate of the UBX-NAV2-COV message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_COV_UART1 = 0x20910436;      // Output rate of the UBX-NAV2-COV message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_COV_UART2 = 0x20910437;      // Output rate of the UBX-NAV2-COV message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_COV_USB = 0x20910438;        // Output rate of the UBX-NAV2-COV message onport USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_DOP_I2C = 0x20910465;        // Output rate of the UBX-NAV2-DOP message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_DOP_SPI = 0x20910469;        // Output rate of the UBX-NAV2-DOP message onport SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_DOP_UART1 = 0x20910466;      // Output rate of the UBX-NAV2-DOP message onport UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_DOP_UART2 = 0x20910467;      // Output rate of the UBX-NAV2-DOP message onport UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_DOP_USB = 0x20910468;        // Output rate of the UBX-NAV2-DOP message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_EOE_I2C = 0x20910565;        // Output rate of the UBX-NAV2-EOE message onport I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_EOE_SPI = 0x20910569;        // Output rate of the UBX-NAV2-EOE message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_EOE_UART1 = 0x20910566;      // Output rate of the UBX-NAV2-EOE message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_EOE_UART2 = 0x20910567;      // Output rate of the UBX-NAV2-EOE message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_EOE_USB = 0x20910568;        // Output rate of the UBX-NAV2-EOE message onport USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_ODO_I2C = 0x20910475;        // Output rate of the UBX-NAV2-ODO message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_ODO_SPI = 0x20910479;        // Output rate of the UBX-NAV2-ODO message onport SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_ODO_UART1 = 0x20910476;      // Output rate of the UBX-NAV2-ODO message onport UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_ODO_UART2 = 0x20910477;      // Output rate of the UBX-NAV2-ODO message onport UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_ODO_USB = 0x20910478;        // Output rate of the UBX-NAV2-ODO message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_POSECEF_I2C = 0x20910480;    // Output rate of the UBX-NAV2-POSECEF message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_POSECEF_SPI = 0x20910484;    // Output rate of the UBX-NAV2-POSECEF message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_POSECEF_UART1 = 0x20910481;  // Output rate of the UBX-NAV2-POSECEF message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_POSECEF_UART2 = 0x20910482;  // Output rate of the UBX-NAV2-POSECEF message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_POSECEF_USB = 0x20910483;    // Output rate of the UBX-NAV2-POSECEF message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_POSLLH_I2C = 0x20910485;     // Output rate of the UBX-NAV2-POSLLH message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_POSLLH_SPI = 0x20910489;     // Output rate of the UBX-NAV2-POSLLH message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_POSLLH_UART1 = 0x20910486;   // Output rate of the UBX-NAV2-POSLLH message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_POSLLH_UART2 = 0x20910487;   // Output rate of the UBX-NAV2-POSLLH message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_POSLLH_USB = 0x20910488;     // Output rate of the UBX-NAV2-POSLLH message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_PVT_I2C = 0x20910490;        // Output rate of the UBX-NAV2-PVT message onport I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_PVT_SPI = 0x20910494;        // Output rate of the UBX-NAV2-PVT message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_PVT_UART1 = 0x20910491;      // Output rate of the UBX-NAV2-PVT message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_PVT_UART2 = 0x20910492;      // Output rate of the UBX-NAV2-PVT message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_PVT_USB = 0x20910493;        // Output rate of the UBX-NAV2-PVT message onport USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SAT_I2C = 0x20910495;        // Output rate of the UBX-NAV2-SAT message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SAT_SPI = 0x20910499;        // Output rate of the UBX-NAV2-SAT message onport SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SAT_UART1 = 0x20910496;      // Output rate of the UBX-NAV2-SAT message onport UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SAT_UART2 = 0x20910497;      // Output rate of the UBX-NAV2-SAT message onport UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SAT_USB = 0x20910498;        // Output rate of the UBX-NAV2-SAT message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SBAS_I2C = 0x20910500;       // Output rate of the UBX-NAV2-SBAS messageon port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SBAS_SPI = 0x20910504;       // Output rate of the UBX-NAV2-SBAS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SBAS_UART1 = 0x20910501;     // Output rate of the UBX-NAV2-SBAS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SBAS_UART2 = 0x20910502;     // Output rate of the UBX-NAV2-SBAS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SBAS_USB = 0x20910503;       // Output rate of the UBX-NAV2-SBAS messageon port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SIG_I2C = 0x20910505;        // Output rate of the UBX-NAV2-SIG message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SIG_SPI = 0x20910509;        // Output rate of the UBX-NAV2-SIG message onport SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SIG_UART1 = 0x20910506;      // Output rate of the UBX-NAV2-SIG message onport UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SIG_UART2 = 0x20910507;      // Output rate of the UBX-NAV2-SIG message onport UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SIG_USB = 0x20910508;        // Output rate of the UBX-NAV2-SIG message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SLAS_I2C = 0x20910510;       // Output rate of the UBX-NAV2-SLAS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SLAS_SPI = 0x20910514;       // Output rate of the UBX-NAV2-SLAS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SLAS_UART1 = 0x20910511;     // Output rate of the UBX-NAV2-SLAS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SLAS_UART2 = 0x20910512;     // Output rate of the UBX-NAV2-SLAS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SLAS_USB = 0x20910513;       // Output rate of the UBX-NAV2-SLAS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_STATUS_I2C = 0x20910515;     // Output rate of the UBX-NAV2-STATUS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_STATUS_SPI = 0x20910519;     // Output rate of the UBX-NAV2-STATUS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_STATUS_UART1 = 0x20910516;   // Output rate of the UBX-NAV2-STATUS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_STATUS_UART2 = 0x20910517;   // Output rate of the UBX-NAV2-STATUS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_STATUS_USB = 0x20910518;     // Output rate of the UBX-NAV2-STATUS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SVIN_I2C = 0x20910520;       // Output rate of the UBX-NAV2-SVIN message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SVIN_SPI = 0x20910524;       // Output rate of the UBX-NAV2-SVIN message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SVIN_UART1 = 0x20910521;     // Output rate of the UBX-NAV2-SVIN message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SVIN_UART2 = 0x20910522;     // Output rate of the UBX-NAV2-SVIN message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_SVIN_USB = 0x20910523;       // Output rate of the UBX-NAV2-SVIN message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEBDS_I2C = 0x20910525;    // Output rate of the UBX-NAV2-TIMEBDS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEBDS_SPI = 0x20910529;    // Output rate of the UBX-NAV2-TIMEBDS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEBDS_UART1 = 0x20910526;  // Output rate of the UBX-NAV2-TIMEBDS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEBDS_UART2 = 0x20910527;  // Output rate of the UBX-NAV2-TIMEBDS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEBDS_USB = 0x20910528;    // Output rate of the UBX-NAV2-TIMEBDS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGAL_I2C = 0x20910530;    // Output rate of the UBX-NAV2-TIMEGAL message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGAL_SPI = 0x20910534;    // Output rate of the UBX-NAV2-TIMEGAL message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGAL_UART1 = 0x20910531;  // Output rate of the UBX-NAV2-TIMEGAL message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGAL_UART2 = 0x20910532;  // Output rate of the UBX-NAV2-TIMEGAL message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGAL_USB = 0x20910533;    // Output rate of the UBX-NAV2-TIMEGAL message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGLO_I2C = 0x20910535;    // Output rate of the UBX-NAV2-TIMEGLO message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGLO_SPI = 0x20910539;    // Output rate of the UBX-NAV2-TIMEGLO message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGLO_UART1 = 0x20910536;  // Output rate of the UBX-NAV2-TIMEGLO message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGLO_UART2 = 0x20910537;  // Output rate of the UBX-NAV2-TIMEGLO message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGLO_USB = 0x20910538;    // Output rate of the UBX-NAV2-TIMEGLO message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGPS_I2C = 0x20910540;    // Output rate of the UBX-NAV2-TIMEGPS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGPS_SPI = 0x20910544;    // Output rate of the UBX-NAV2-TIMEGPS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGPS_UART1 = 0x20910541;  // Output rate of the UBX-NAV2-TIMEGPS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGPS_UART2 = 0x20910542;  // Output rate of the UBX-NAV2-TIMEGPS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEGPS_USB = 0x20910543;    // Output rate of the UBX-NAV2-TIMEGPS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMELS_I2C = 0x20910545;     // Output rate of the UBX-NAV2-TIMELS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMELS_SPI = 0x20910549;     // Output rate of the UBX-NAV2-TIMELS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMELS_UART1 = 0x20910546;   // Output rate of the UBX-NAV2-TIMELS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMELS_UART2 = 0x20910547;   // Output rate of the UBX-NAV2-TIMELS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMELS_USB = 0x20910548;     // Output rate of the UBX-NAV2-TIMELS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEQZSS_I2C = 0x20910575;   // Output rate of the UBX-NAV2-TIMEQZSS message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEQZSS_SPI = 0x20910579;   // Output rate of the UBX-NAV2-TIMEQZSS message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEQZSS_UART1 = 0x20910576; // Output rate of the UBX-NAV2-TIMEQZSS message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEQZSS_UART2 = 0x20910577; // Output rate of the UBX-NAV2-TIMEQZSS message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEQZSS_USB = 0x20910578;   // Output rate of the UBX-NAV2-TIMEQZSS message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEUTC_I2C = 0x20910550;    // Output rate of the UBX-NAV2-TIMEUTC message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEUTC_SPI = 0x20910554;    // Output rate of the UBX-NAV2-TIMEUTC message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEUTC_UART1 = 0x20910551;  // Output rate of the UBX-NAV2-TIMEUTC message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEUTC_UART2 = 0x20910552;  // Output rate of the UBX-NAV2-TIMEUTC message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_TIMEUTC_USB = 0x20910553;    // Output rate of the UBX-NAV2-TIMEUTC message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_VELECEF_I2C = 0x20910555;    // Output rate of the UBX-NAV2-VELECEF message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_VELECEF_SPI = 0x20910559;    // Output rate of the UBX-NAV2-VELECEF message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_VELECEF_UART1 = 0x20910556;  // Output rate of the UBX-NAV2-VELECEF message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_VELECEF_UART2 = 0x20910557;  // Output rate of the UBX-NAV2-VELECEF message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_VELECEF_USB = 0x20910558;    // Output rate of the UBX-NAV2-VELECEF message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_VELNED_I2C = 0x20910560;     // Output rate of the UBX-NAV2-VELNED message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_VELNED_SPI = 0x20910564;     // Output rate of the UBX-NAV2-VELNED message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_VELNED_UART1 = 0x20910561;   // Output rate of the UBX-NAV2-VELNED message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_VELNED_UART2 = 0x20910562;   // Output rate of the UBX-NAV2-VELNED message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV2_VELNED_USB = 0x20910563;     // Output rate of the UBX-NAV2-VELNED message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_NMI_I2C = 0x20910590;         // Output rate of the UBX-NAV-NMI message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_NMI_SPI = 0x20910594;         // Output rate of the UBX-NAV-NMI message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_NMI_UART1 = 0x20910591;       // Output rate of the UBX-NAV-NMI message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_NMI_UART2 = 0x20910592;       // Output rate of the UBX-NAV-NMI message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_NAV_NMI_USB = 0x20910593;         // Output rate of the UBX-NAV-NMI message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_TM_I2C = 0x20910610;          // Output rate of the UBX-RXM-TM message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_TM_SPI = 0x20910614;          // Output rate of the UBX-RXM-TM message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_TM_UART1 = 0x20910611;        // Output rate of the UBX-RXM-TM message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_TM_UART2 = 0x20910612;        // Output rate of the UBX-RXM-TM message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_TM_USB = 0x20910613;          // Output rate of the UBX-RXM-TM message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_SEC_SIGLOG_I2C = 0x20910689;      // Output rate of the UBX-SEC-SIGLOG message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_SEC_SIGLOG_SPI = 0x2091068d;      // Output rate of the UBX-SEC-SIGLOG message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_SEC_SIGLOG_UART1 = 0x2091068a;    // Output rate of the UBX-SEC-SIGLOG message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_SEC_SIGLOG_UART2 = 0x2091068b;    // Output rate of the UBX-SEC-SIGLOG message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_SEC_SIGLOG_USB = 0x2091068c;      // Output rate of the UBX-SEC-SIGLOG message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_SVIN_I2C = 0x20910097;        // Output rate of the UBX-TIM-SVIN message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_SVIN_SPI = 0x2091009b;        // Output rate of the UBX-TIM-SVIN message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_SVIN_UART1 = 0x20910098;      // Output rate of the UBX-TIM-SVIN message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_SVIN_UART2 = 0x20910099;      // Output rate of the UBX-TIM-SVIN message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_TIM_SVIN_USB = 0x2091009a;        // Output rate of the UBX-TIM-SVIN message on port USB

// Additional CFG_MSGOUT keys for the NEO-D9S
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C = 0x2091031d;   // Output rate of the UBX_RXM_PMP message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_PMP_SPI = 0x20910321;   // Output rate of the UBX_RXM_PMP message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART1 = 0x2091031e; // Output rate of the UBX_RXM_PMP message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART2 = 0x2091031f; // Output rate of the UBX_RXM_PMP message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_PMP_USB = 0x20910320;   // Output rate of the UBX_RXM_PMP message on port USB
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_PMP_I2C = 0x20910322;   // Output rate of the UBX_MON_PMP message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_PMP_SPI = 0x20910326;   // Output rate of the UBX_MON_PMP message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_PMP_UART1 = 0x20910323; // Output rate of the UBX_MON_PMP message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_PMP_UART2 = 0x20910324; // Output rate of the UBX_MON_PMP message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_MON_PMP_USB = 0x20910325;   // Output rate of the UBX_MON_PMP message on port USB

// Additional CFG_MSGOUT keys for the NEO-D9S
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_QZSSL6_I2C = 0x2091033f;   // Output rate of the UBX_RXM_QZSSL6 message on port I2C
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_QZSSL6_SPI = 0x2091033e;   // Output rate of the UBX_RXM_QZSSL6 message on port SPI
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_QZSSL6_UART1 = 0x2091033b; // Output rate of the UBX_RXM_QZSSL6 message on port UART1
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_QZSSL6_UART2 = 0x2091033c; // Output rate of the UBX_RXM_QZSSL6 message on port UART2
const uint32_t UBLOX_CFG_MSGOUT_UBX_RXM_QZSSL6_USB = 0x2091033d;   // Output rate of the UBX_RXM_QZSSL6 message on port USB

// CFG-NAV2: Secondary output configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_NAV2_OUT_ENABLED = 0x10170001;        // Enable secondary (NAV2) output
const uint32_t UBLOX_CFG_NAV2_SBAS_USE_INTEGRITY = 0x10170002; // Use SBAS integrity information in the secondary output

// CFG-NAVHPG: High precision navigation configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_NAVHPG_DGNSSMODE = 0x20140011; // Diﬀerential corrections mode

// CFG-NAVSPG: Standard precision navigation configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_NAVSPG_FIXMODE = 0x20110011;        // Position fix mode
const uint32_t UBLOX_CFG_NAVSPG_INIFIX3D = 0x10110013;       // Initial fix must be a 3D fix
const uint32_t UBLOX_CFG_NAVSPG_WKNROLLOVER = 0x30110017;    // GPS week rollover number
const uint32_t UBLOX_CFG_NAVSPG_USE_PPP = 0x10110019;        // Use precise point positioning (PPP)
const uint32_t UBLOX_CFG_NAVSPG_UTCSTANDARD = 0x2011001c;    // UTC standard to be used
const uint32_t UBLOX_CFG_NAVSPG_DYNMODEL = 0x20110021;       // Dynamic platform model
const uint32_t UBLOX_CFG_NAVSPG_ACKAIDING = 0x10110025;      // Acknowledge assistance input messages
const uint32_t UBLOX_CFG_NAVSPG_USE_USRDAT = 0x10110061;     // Use user geodetic datum parameters
const uint32_t UBLOX_CFG_NAVSPG_USRDAT_MAJA = 0x50110062;    // Geodetic datum semi-major axis
const uint32_t UBLOX_CFG_NAVSPG_USRDAT_FLAT = 0x50110063;    // Geodetic datum 1.0 flattening
const uint32_t UBLOX_CFG_NAVSPG_USRDAT_DX = 0x40110064;      // Geodetic datum X axis shift at the origin
const uint32_t UBLOX_CFG_NAVSPG_USRDAT_DY = 0x40110065;      // Geodetic datum Y axis shift at the origin
const uint32_t UBLOX_CFG_NAVSPG_USRDAT_DZ = 0x40110066;      // Geodetic datum Z axis shift at the origin
const uint32_t UBLOX_CFG_NAVSPG_USRDAT_ROTX = 0x40110067;    // arcsec Geodetic datum rotation about the X axis
const uint32_t UBLOX_CFG_NAVSPG_USRDAT_ROTY = 0x40110068;    // arcsec Geodetic datum rotation about the Y axis
const uint32_t UBLOX_CFG_NAVSPG_USRDAT_ROTZ = 0x40110069;    // arcsec Geodetic datum rotation about the Z axis
const uint32_t UBLOX_CFG_NAVSPG_USRDAT_SCALE = 0x4011006a;   // ppm Geodetic datum scale factor
const uint32_t UBLOX_CFG_NAVSPG_INFIL_MINSVS = 0x201100a1;   // Minimum number of satellites for navigation
const uint32_t UBLOX_CFG_NAVSPG_INFIL_MAXSVS = 0x201100a2;   // Maximum number of satellites for navigation
const uint32_t UBLOX_CFG_NAVSPG_INFIL_MINCNO = 0x201100a3;   // Minimum satellite signal level for navigation
const uint32_t UBLOX_CFG_NAVSPG_INFIL_MINELEV = 0x201100a4;  // Minimum elevation for a GNSS satellite to be used in navigation
const uint32_t UBLOX_CFG_NAVSPG_INFIL_NCNOTHRS = 0x201100aa; // Number of satellites required to have C/N0 above const uint32_t UBLOX_CFG_NAVSPG-INFIL_CNOTHRS for a fix to be attempted
const uint32_t UBLOX_CFG_NAVSPG_INFIL_CNOTHRS = 0x201100ab;  // C/N0 threshold for deciding whether to attempt a fix
const uint32_t UBLOX_CFG_NAVSPG_OUTFIL_PDOP = 0x301100b1;    // Output filter position DOP mask (threshold)
const uint32_t UBLOX_CFG_NAVSPG_OUTFIL_TDOP = 0x301100b2;    // Output filter time DOP mask (threshold)
const uint32_t UBLOX_CFG_NAVSPG_OUTFIL_PACC = 0x301100b3;    // Output filter position accuracy mask (threshold)
const uint32_t UBLOX_CFG_NAVSPG_OUTFIL_TACC = 0x301100b4;    // Output filter time accuracy mask (threshold)
const uint32_t UBLOX_CFG_NAVSPG_OUTFIL_FACC = 0x301100b5;    // Output filter frequency accuracy mask (threshold)
const uint32_t UBLOX_CFG_NAVSPG_CONSTR_ALT = 0x401100c1;     // Fixed altitude (mean sea level) for 2D fix mode
const uint32_t UBLOX_CFG_NAVSPG_CONSTR_ALTVAR = 0x401100c2;  // Fixed altitude variance for 2D mode
const uint32_t UBLOX_CFG_NAVSPG_CONSTR_DGNSSTO = 0x201100c4; // DGNSS timeout
const uint32_t UBLOX_CFG_NAVSPG_SIGATTCOMP = 0x201100d6;     // Permanently attenuated signal compensation mode
const uint32_t UBLOX_CFG_NAVSPG_PL_ENA = 0x101100d7;         // Enable Protection level. If enabled, protection level computing will be on.

// CFG-NMEA: NMEA protocol configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_NMEA_PROTVER = 0x20930001;       // NMEA protocol version
const uint32_t UBLOX_CFG_NMEA_MAXSVS = 0x20930002;        // Maximum number of SVs to report per Talker ID
const uint32_t UBLOX_CFG_NMEA_COMPAT = 0x10930003;        // Enable compatibility mode
const uint32_t UBLOX_CFG_NMEA_CONSIDER = 0x10930004;      // Enable considering mode
const uint32_t UBLOX_CFG_NMEA_LIMIT82 = 0x10930005;       // Enable strict limit to 82 characters maximum NMEA message length
const uint32_t UBLOX_CFG_NMEA_HIGHPREC = 0x10930006;      // Enable high precision mode
const uint32_t UBLOX_CFG_NMEA_SVNUMBERING = 0x20930007;   // Display configuration for SVs that do not have value defined in NMEA
const uint32_t UBLOX_CFG_NMEA_FILT_GPS = 0x10930011;      // Disable reporting of GPS satellites
const uint32_t UBLOX_CFG_NMEA_FILT_SBAS = 0x10930012;     // Disable reporting of SBAS satellites
const uint32_t UBLOX_CFG_NMEA_FILT_GAL = 0x10930013;      // Disable reporting of Galileo satellites
const uint32_t UBLOX_CFG_NMEA_FILT_QZSS = 0x10930015;     // Disable reporting of QZSS satellites
const uint32_t UBLOX_CFG_NMEA_FILT_GLO = 0x10930016;      // Disable reporting of GLONASS satellites
const uint32_t UBLOX_CFG_NMEA_FILT_BDS = 0x10930017;      // Disable reporting of BeiDou satellites
const uint32_t UBLOX_CFG_NMEA_OUT_INVFIX = 0x10930021;    // Enable position output for failed or invalid fixes
const uint32_t UBLOX_CFG_NMEA_OUT_MSKFIX = 0x10930022;    // Enable position output for invalid fixes
const uint32_t UBLOX_CFG_NMEA_OUT_INVTIME = 0x10930023;   // Enable time output for invalid times
const uint32_t UBLOX_CFG_NMEA_OUT_INVDATE = 0x10930024;   // Enable date output for invalid dates
const uint32_t UBLOX_CFG_NMEA_OUT_ONLYGPS = 0x10930025;   // Restrict output to GPS satellites only
const uint32_t UBLOX_CFG_NMEA_OUT_FROZENCOG = 0x10930026; // Enable course over ground output even if it is frozen
const uint32_t UBLOX_CFG_NMEA_MAINTALKERID = 0x20930031;  // Main Talker ID
const uint32_t UBLOX_CFG_NMEA_GSVTALKERID = 0x20930032;   // Talker ID for GSV NMEA messages
const uint32_t UBLOX_CFG_NMEA_BDSTALKERID = 0x30930033;   // BeiDou Talker ID

// CFG-ODO: Odometer and low-speed course over ground filter
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_ODO_USE_ODO = 0x10220001;      // Use odometer
const uint32_t UBLOX_CFG_ODO_USE_COG = 0x10220002;      // Use low-speed course over ground filter
const uint32_t UBLOX_CFG_ODO_OUTLPVEL = 0x10220003;     // Output low-pass filtered velocity
const uint32_t UBLOX_CFG_ODO_OUTLPCOG = 0x10220004;     // Output low-pass filtered course over ground (heading)
const uint32_t UBLOX_CFG_ODO_PROFILE = 0x20220005;      // Odometer profile configuration
const uint32_t UBLOX_CFG_ODO_COGMAXSPEED = 0x20220021;  // Upper speed limit for low-speed course over ground filter
const uint32_t UBLOX_CFG_ODO_COGMAXPOSACC = 0x20220022; // Maximum acceptable position accuracy for computing low-speed filtered course over ground
const uint32_t UBLOX_CFG_ODO_VELLPGAIN = 0x20220031;    // Velocity low-pass filter level
const uint32_t UBLOX_CFG_ODO_COGLPGAIN = 0x20220032;    // Course over ground low-pass filter level (at speed < 8 m/s)

// CFG-PM: Configuration for receiver power management (NEO-D9S)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_PM_EXTINTSEL = 0x20d0000b;        // EXTINT pin select
const uint32_t UBLOX_CFG_PM_EXTINTWAKE = 0x10d0000c;       // EXTINT pin control (Wake). Enable to keep receiver awake as long as selected EXTINT pin is "high".
const uint32_t UBLOX_CFG_PM_EXTINTBACKUP = 0x10d0000d;     // EXTINT pin control (Backup). Enable to force receiver into BACKUP mode when selected EXTINT pin is "low".
const uint32_t UBLOX_CFG_PM_EXTINTINACTIVE = 0x10d0000e;   // EXTINT pin control (Inactive). Enable to force backup in case EXTINT Pin is inactive for time longer than CFG-PM-EXTINTINACTIVITY.
const uint32_t UBLOX_CFG_PM_EXTINTINACTIVITY = 0x40d0000f; // Inactivity time out on EXTINT pin if enabled

// CFG-PMP: Point to multipoint (PMP) configuration (NEO-D9S)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_PMP_CENTER_FREQUENCY = 0x40b10011;  // Center frequency. The center frequency for the receiver can be set from 1525000000 to 1559000000 Hz.
const uint32_t UBLOX_CFG_PMP_SEARCH_WINDOW = 0x30b10012;     // Search window. Search window can be set from 0 to 65535 Hz. It is +/- this value from the center frequency set by CENTER_FREQUENCY.
const uint32_t UBLOX_CFG_PMP_USE_SERVICE_ID = 0x10b10016;    // Use service ID. Enable/disable service ID check to confirm the correct service is received.
const uint32_t UBLOX_CFG_PMP_SERVICE_ID = 0x30b10017;        // Service identifier. Defines the expected service ID.
const uint32_t UBLOX_CFG_PMP_DATA_RATE = 0x30b10013;         // bps Data rate. The data rate of the received data.
const uint32_t UBLOX_CFG_PMP_USE_DESCRAMBLER = 0x10b10014;   // Use descrambler. Enables/disables the descrambler.
const uint32_t UBLOX_CFG_PMP_DESCRAMBLER_INIT = 0x30b10015;  // Descrambler initialization. Set the intialisation value for the descrambler.
const uint32_t UBLOX_CFG_PMP_USE_PRESCRAMBLING = 0x10b10019; // Use prescrambling. Enables/disables the prescrambling.
const uint32_t UBLOX_CFG_PMP_UNIQUE_WORD = 0x50b1001a;       // Unique word. Defines value of unique word.

// CFG-QZSS-L6: QZSS system configuration configuration (NEO-D9C)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_QZSSL6_SVIDA = 0x20370020;          // QZSS L6 SV Id to be decoded by channel A
const uint32_t UBLOX_CFG_QZSSL6_SVIDB = 0x20370030;          // QZSS L6 SV Id to be decoded by channel B
const uint32_t UBLOX_CFG_QZSSL6_MSGA = 0x20370050;           // QZSS L6 messages to be decoded by channel A
const uint32_t UBLOX_CFG_QZSSL6_MSGB = 0x20370060;           // QZSS L6 messages to be decoded by channel B
const uint32_t UBLOX_CFG_QZSSL6_RSDECODER = 0x20370080;      // QZSS L6 message Reed-Solomon decoder mode

// CFG-QZSS: QZSS system configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_QZSS_USE_SLAS_DGNSS = 0x10370005;       // Apply QZSS SLAS DGNSS corrections
const uint32_t UBLOX_CFG_QZSS_USE_SLAS_TESTMODE = 0x10370006;    // Use QZSS SLAS data when it is in test mode (SLAS msg 0)
const uint32_t UBLOX_CFG_QZSS_USE_SLAS_RAIM_UNCORR = 0x10370007; // Raim out measurements that are not corrected by QZSS SLAS, if at least 5 measurements are corrected
const uint32_t UBLOX_CFG_QZSS_SLAS_MAX_BASELINE = 0x30370008;    // Maximum baseline distance to closest Ground Monitoring Station: km

// CFG-RATE: Navigation and measurement rate configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_RATE_MEAS = 0x30210001;     // Nominal time between GNSS measurements
const uint32_t UBLOX_CFG_RATE_NAV = 0x30210002;      // Ratio of number of measurements to number of navigation solutions
const uint32_t UBLOX_CFG_RATE_TIMEREF = 0x20210003;  // Time system to which measurements are aligned
const uint32_t UBLOX_CFG_RATE_NAV_PRIO = 0x20210004; // Output rate of priority navigation mode messages

// CFG-RINV: Remote inventory
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_RINV_DUMP = 0x10c70001;      // Dump data at startup
const uint32_t UBLOX_CFG_RINV_BINARY = 0x10c70002;    // Data is binary
const uint32_t UBLOX_CFG_RINV_DATA_SIZE = 0x20c70003; // Size of data
const uint32_t UBLOX_CFG_RINV_CHUNK0 = 0x50c70004;    // Data bytes 1-8 (LSB)
const uint32_t UBLOX_CFG_RINV_CHUNK1 = 0x50c70005;    // Data bytes 9-16
const uint32_t UBLOX_CFG_RINV_CHUNK2 = 0x50c70006;    // Data bytes 17-240x44434241.
const uint32_t UBLOX_CFG_RINV_CHUNK3 = 0x50c70007;    // Data bytes 25-30 (MSB)

// CFG-RTCM: RTCM protocol configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_RTCM_DF003_OUT = 0x30090001;       // RTCM DF003 (Reference station ID) output value
const uint32_t UBLOX_CFG_RTCM_DF003_IN = 0x30090008;        // RTCM DF003 (Reference station ID) input value
const uint32_t UBLOX_CFG_RTCM_DF003_IN_FILTER = 0x20090009; // RTCM input filter configuration based on RTCM DF003 (Reference station ID) value

// CFG-SBAS: SBAS configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_SBAS_USE_TESTMODE = 0x10360002;  // Use SBAS data when it is in test mode (SBAS msg 0)
const uint32_t UBLOX_CFG_SBAS_USE_RANGING = 0x10360003;   // Use SBAS GEOs as a ranging source (for navigation)
const uint32_t UBLOX_CFG_SBAS_USE_DIFFCORR = 0x10360004;  // Use SBAS diﬀerential corrections
const uint32_t UBLOX_CFG_SBAS_USE_INTEGRITY = 0x10360005; // Use SBAS integrity information
const uint32_t UBLOX_CFG_SBAS_PRNSCANMASK = 0x50360006;   // SBAS PRN search configuration

// CFG-SEC: Security configuration (ZED-F9R)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_SEC_CFG_LOCK = 0x10f60009;            // Configuration lockdown
const uint32_t UBLOX_CFG_SEC_CFG_LOCK_UNLOCKGRP1 = 0x30f6000a; // Configuration lockdown exempted group 1
const uint32_t UBLOX_CFG_SEC_CFG_LOCK_UNLOCKGRP2 = 0x30f6000b; // Configuration lockdown exempted group 2

// CFG-SFCORE: Sensor fusion (SF) core configuration (ZED-F9R)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_SFCORE_USE_SF = 0x10080001; // Use ADR/UDR sensor fusion

// CFG-SFIMU: Sensor fusion (SF) inertial measurement unit (IMU) configuration (ZED-F9R)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_SFIMU_GYRO_TC_UPDATE_PERIOD = 0x30060007; // Time period between each update for the saved temperature-dependent gyroscope bias table
const uint32_t UBLOX_CFG_SFIMU_GYRO_RMSTHDL = 0x20060008;          // Gyroscope sensor RMS threshold
const uint32_t UBLOX_CFG_SFIMU_GYRO_FREQUENCY = 0x20060009;        // Nominal gyroscope sensor data sampling frequency
const uint32_t UBLOX_CFG_SFIMU_GYRO_LATENCY = 0x3006000a;          // Gyroscope sensor data latency due to e.g. CAN bus
const uint32_t UBLOX_CFG_SFIMU_GYRO_ACCURACY = 0x3006000b;         // Gyroscope sensor data accuracy
const uint32_t UBLOX_CFG_SFIMU_ACCEL_RMSTHDL = 0x20060015;         // Accelerometer RMS threshold
const uint32_t UBLOX_CFG_SFIMU_ACCEL_FREQUENCY = 0x20060016;       // Nominal accelerometer sensor data sampling frequency
const uint32_t UBLOX_CFG_SFIMU_ACCEL_LATENCY = 0x30060017;         // Accelerometer sensor data latency due to e.g. CAN bus
const uint32_t UBLOX_CFG_SFIMU_ACCEL_ACCURACY = 0x30060018;        // Accelerometer sensor data accuracy
const uint32_t UBLOX_CFG_SFIMU_IMU_EN = 0x1006001d;                // IMU enabled
const uint32_t UBLOX_CFG_SFIMU_IMU_I2C_SCL_PIO = 0x2006001e;       // SCL PIO of the IMU I2C
const uint32_t UBLOX_CFG_SFIMU_IMU_I2C_SDA_PIO = 0x2006001f;       // SDA PIO of the IMU I2C
const uint32_t UBLOX_CFG_SFIMU_AUTO_MNTALG_ENA = 0x10060027;       // Enable automatic IMU-mount alignment
const uint32_t UBLOX_CFG_SFIMU_IMU_MNTALG_YAW = 0x4006002d;        // User-defined IMU-mount yaw angle [0, 360]
const uint32_t UBLOX_CFG_SFIMU_IMU_MNTALG_PITCH = 0x3006002e;      // User-defined IMU-mount pitch angle [-90, 90]
const uint32_t UBLOX_CFG_SFIMU_IMU_MNTALG_ROLL = 0x3006002f;       // User-defined IMU-mount roll angle [-180, 180]

// CFG-SFODO: Sensor fusion (SF) odometer configuration (ZED-F9R)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_SFODO_COMBINE_TICKS = 0x10070001;     // Use combined rear wheel ticks instead of the single tick
const uint32_t UBLOX_CFG_SFODO_USE_SPEED = 0x10070003;         // Use speed measurements
const uint32_t UBLOX_CFG_SFODO_DIS_AUTOCOUNTMAX = 0x10070004;  // Disable automatic estimation of maximum absolute wheel tick counter
const uint32_t UBLOX_CFG_SFODO_DIS_AUTODIRPINPOL = 0x10070005; // Disable automatic wheel tick direction pin polarity detection
const uint32_t UBLOX_CFG_SFODO_DIS_AUTOSPEED = 0x10070006;     // Disable automatic receiver reconfiguration for processing speed data
const uint32_t UBLOX_CFG_SFODO_FACTOR = 0x40070007;            // Wheel tick scale factor
const uint32_t UBLOX_CFG_SFODO_QUANT_ERROR = 0x40070008;       // Wheel tick quantization
const uint32_t UBLOX_CFG_SFODO_COUNT_MAX = 0x40070009;         // Wheel tick counter maximum value
const uint32_t UBLOX_CFG_SFODO_LATENCY = 0x3007000a;           // Wheel tick data latency due to e.g. CAN bus
const uint32_t UBLOX_CFG_SFODO_FREQUENCY = 0x2007000b;         // Nominal wheel tick data frequency (0 = not set)
const uint32_t UBLOX_CFG_SFODO_CNT_BOTH_EDGES = 0x1007000d;    // Count both rising and falling edges on wheel tick signal
const uint32_t UBLOX_CFG_SFODO_SPEED_BAND = 0x3007000e;        // Speed sensor dead band (0 = not set)
const uint32_t UBLOX_CFG_SFODO_USE_WT_PIN = 0x1007000f;        // Wheel tick signal enabled
const uint32_t UBLOX_CFG_SFODO_DIR_PINPOL = 0x10070010;        // Wheel tick direction pin polarity
const uint32_t UBLOX_CFG_SFODO_DIS_AUTOSW = 0x10070011;        // Disable automatic use of wheel tick or speed data received over the software interface

// CFG-SIGNAL: Satellite systems (GNSS) signal configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_SIGNAL_GPS_ENA = 0x1031001f;       // GPS enable
const uint32_t UBLOX_CFG_SIGNAL_GPS_L1CA_ENA = 0x10310001;  // GPS L1C/A
const uint32_t UBLOX_CFG_SIGNAL_GPS_L5_ENA = 0x10310004;    // GPS L5
const uint32_t UBLOX_CFG_SIGNAL_GPS_L2C_ENA = 0x10310003;   // GPS L2C (only on u-blox F9 platform products)
const uint32_t UBLOX_CFG_SIGNAL_SBAS_ENA = 0x10310020;      // SBAS enable
const uint32_t UBLOX_CFG_SIGNAL_SBAS_L1CA_ENA = 0x10310005; // SBAS L1C/A
const uint32_t UBLOX_CFG_SIGNAL_GAL_ENA = 0x10310021;       // Galileo enable
const uint32_t UBLOX_CFG_SIGNAL_GAL_E1_ENA = 0x10310007;    // Galileo E1
const uint32_t UBLOX_CFG_SIGNAL_GAL_E5A_ENA = 0x10310009;   // Galileo E5a
const uint32_t UBLOX_CFG_SIGNAL_GAL_E5B_ENA = 0x1031000a;   // Galileo E5b (only on u-blox F9 platform products)
const uint32_t UBLOX_CFG_SIGNAL_BDS_ENA = 0x10310022;       // BeiDou Enable
const uint32_t UBLOX_CFG_SIGNAL_BDS_B1_ENA = 0x1031000d;    // BeiDou B1I
const uint32_t UBLOX_CFG_SIGNAL_BDS_B1C_ENA = 0x1031000f;   // BeiDou B1C
const uint32_t UBLOX_CFG_SIGNAL_BDS_B2A_ENA = 0x10310028;   // BeiDou B2a
const uint32_t UBLOX_CFG_SIGNAL_BDS_B2_ENA = 0x1031000e;    // BeiDou B2I (only on u-blox F9 platform products)
const uint32_t UBLOX_CFG_SIGNAL_QZSS_ENA = 0x10310024;      // QZSS enable
const uint32_t UBLOX_CFG_SIGNAL_QZSS_L1CA_ENA = 0x10310012; // QZSS L1C/A
const uint32_t UBLOX_CFG_SIGNAL_QZSS_L5_ENA = 0x10310017;   // QZSS L5
const uint32_t UBLOX_CFG_SIGNAL_QZSS_L1S_ENA = 0x10310014;  // QZSS L1S
const uint32_t UBLOX_CFG_SIGNAL_QZSS_L2C_ENA = 0x10310015;  // QZSS L2C (only on u-blox F9 platform products)
const uint32_t UBLOX_CFG_SIGNAL_GLO_ENA = 0x10310025;       // GLONASS enable
const uint32_t UBLOX_CFG_SIGNAL_GLO_L1_ENA = 0x10310018;    // GLONASS L1
const uint32_t UBLOX_CFG_SIGNAL_GLO_L2_ENA = 0x1031001a;    // GLONASS L2 (only on u-blox F9 platform products)

// CFG-SPARTN: Configuration of the SPARTN interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_SPARTN_USE_SOURCE = 0x20a70001;

// CFG-SPI: Configuration of the SPI interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_SPI_MAXFF = 0x20640001;           // Number of bytes containing 0xFF to receive before switching oﬀ reception. Range: 0 (mechanism oﬀ) - 63
const uint32_t UBLOX_CFG_SPI_CPOLARITY = 0x10640002;       // Clock polarity select: 0: Active Hight Clock, SCLK idles low, 1: Active Low Clock, SCLK idles high
const uint32_t UBLOX_CFG_SPI_CPHASE = 0x10640003;          // Clock phase select: 0: Data captured on first edge of SCLK, 1: Data captured on second edge of SCLK
const uint32_t UBLOX_CFG_SPI_EXTENDEDTIMEOUT = 0x10640005; // Flag to disable timeouting the interface after 1.5s
const uint32_t UBLOX_CFG_SPI_ENABLED = 0x10640006;         // Flag to indicate if the SPI interface should be enabled

// CFG-SPIINPROT: Input protocol configuration of the SPI interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_SPIINPROT_UBX = 0x10790001;    // Flag to indicate if UBX should be an input protocol on SPI
const uint32_t UBLOX_CFG_SPIINPROT_NMEA = 0x10790002;   // Flag to indicate if NMEA should be an input protocol on SPI
const uint32_t UBLOX_CFG_SPIINPROT_RTCM3X = 0x10790004; // Flag to indicate if RTCM3X should be an input protocol on SPI
const uint32_t UBLOX_CFG_SPIINPROT_SPARTN = 0x10790005; // Flag to indicate if SPARTN should be an input protocol on SPI

// CFG-SPIOUTPROT: Output protocol configuration of the SPI interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_SPIOUTPROT_UBX = 0x107a0001;    // Flag to indicate if UBX should be an output protocol on SPI
const uint32_t UBLOX_CFG_SPIOUTPROT_NMEA = 0x107a0002;   // Flag to indicate if NMEA should be an output protocol on SPI
const uint32_t UBLOX_CFG_SPIOUTPROT_RTCM3X = 0x107a0004; // Flag to indicate if RTCM3X should be an output protocol on SPI

// CFG-TMODE: Time mode configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_TMODE_MODE = 0x20030001;           // Receiver mode
const uint32_t UBLOX_CFG_TMODE_POS_TYPE = 0x20030002;       // Determines whether the ARP position is given in ECEF or LAT/LON/HEIGHT?
const uint32_t UBLOX_CFG_TMODE_ECEF_X = 0x40030003;         // ECEF X coordinate of the ARP position.
const uint32_t UBLOX_CFG_TMODE_ECEF_Y = 0x40030004;         // ECEF Y coordinate of the ARP position.
const uint32_t UBLOX_CFG_TMODE_ECEF_Z = 0x40030005;         // ECEF Z coordinate of the ARP position.
const uint32_t UBLOX_CFG_TMODE_ECEF_X_HP = 0x20030006;      // High-precision ECEF X coordinate of the ARP position.
const uint32_t UBLOX_CFG_TMODE_ECEF_Y_HP = 0x20030007;      // High-precision ECEF Y coordinate of the ARP position.
const uint32_t UBLOX_CFG_TMODE_ECEF_Z_HP = 0x20030008;      // High-precision ECEF Z coordinate of the ARP position.
const uint32_t UBLOX_CFG_TMODE_LAT = 0x40030009;            // Latitude of the ARP position.
const uint32_t UBLOX_CFG_TMODE_LON = 0x4003000a;            // Longitude of the ARP position.
const uint32_t UBLOX_CFG_TMODE_HEIGHT = 0x4003000b;         // Height of the ARP position.
const uint32_t UBLOX_CFG_TMODE_LAT_HP = 0x2003000c;         // High-precision latitude of the ARP position
const uint32_t UBLOX_CFG_TMODE_LON_HP = 0x2003000d;         // High-precision longitude of the ARP position.
const uint32_t UBLOX_CFG_TMODE_HEIGHT_HP = 0x2003000e;      // High-precision height of the ARP position.
const uint32_t UBLOX_CFG_TMODE_FIXED_POS_ACC = 0x4003000f;  // Fixed position 3D accuracy
const uint32_t UBLOX_CFG_TMODE_SVIN_MIN_DUR = 0x40030010;   // Survey-in minimum duration
const uint32_t UBLOX_CFG_TMODE_SVIN_ACC_LIMIT = 0x40030011; // Survey-in position accuracy limit

// CFG-TP: Timepulse configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_TP_PULSE_DEF = 0x20050023;        // Determines whether the time pulse is interpreted as frequency or period
const uint32_t UBLOX_CFG_TP_PULSE_LENGTH_DEF = 0x20050030; // Determines whether the time pulse length is interpreted as length[us] or pulse ratio[%]
const uint32_t UBLOX_CFG_TP_ANT_CABLEDELAY = 0x30050001;   // Antenna cable delay
const uint32_t UBLOX_CFG_TP_PERIOD_TP1 = 0x40050002;       // Time pulse period (TP1)
const uint32_t UBLOX_CFG_TP_PERIOD_LOCK_TP1 = 0x40050003;  // Time pulse period when locked to GNSS time (TP1)
const uint32_t UBLOX_CFG_TP_FREQ_TP1 = 0x40050024;         // Time pulse frequency (TP1)
const uint32_t UBLOX_CFG_TP_FREQ_LOCK_TP1 = 0x40050025;    // Time pulse frequency when locked to GNSS time (TP1)
const uint32_t UBLOX_CFG_TP_LEN_TP1 = 0x40050004;          // Time pulse length (TP1)
const uint32_t UBLOX_CFG_TP_LEN_LOCK_TP1 = 0x40050005;     // Time pulse length when locked to GNSS time (TP1)
const uint32_t UBLOX_CFG_TP_DUTY_TP1 = 0x5005002a;         // Time pulse duty cycle (TP1)
const uint32_t UBLOX_CFG_TP_DUTY_LOCK_TP1 = 0x5005002b;    // Time pulse duty cycle when locked to GNSS time (TP1)
const uint32_t UBLOX_CFG_TP_USER_DELAY_TP1 = 0x40050006;   // User-configurable time pulse delay (TP1)
const uint32_t UBLOX_CFG_TP_TP1_ENA = 0x10050007;          // Enable the first timepulse
const uint32_t UBLOX_CFG_TP_SYNC_GNSS_TP1 = 0x10050008;    // Sync time pulse to GNSS time or local clock (TP1)
const uint32_t UBLOX_CFG_TP_USE_LOCKED_TP1 = 0x10050009;   // Use locked parameters when possible (TP1)
const uint32_t UBLOX_CFG_TP_ALIGN_TO_TOW_TP1 = 0x1005000a; // Align time pulse to top of second (TP1)
const uint32_t UBLOX_CFG_TP_POL_TP1 = 0x1005000b;          // Set time pulse polarity (TP1)
const uint32_t UBLOX_CFG_TP_TIMEGRID_TP1 = 0x2005000c;     // Time grid to use (TP1)
const uint32_t UBLOX_CFG_TP_PERIOD_TP2 = 0x4005000d;       // Time pulse period (TP2)
const uint32_t UBLOX_CFG_TP_PERIOD_LOCK_TP2 = 0x4005000e;  // Time pulse period when locked to GNSS time
const uint32_t UBLOX_CFG_TP_FREQ_TP2 = 0x40050026;         // Time pulse frequency (TP2)
const uint32_t UBLOX_CFG_TP_FREQ_LOCK_TP2 = 0x40050027;    // Time pulse frequency when locked to GNSS time
const uint32_t UBLOX_CFG_TP_LEN_TP2 = 0x4005000f;          // Time pulse length (TP2)
const uint32_t UBLOX_CFG_TP_LEN_LOCK_TP2 = 0x40050010;     // Time pulse length when locked to GNSS time
const uint32_t UBLOX_CFG_TP_DUTY_TP2 = 0x5005002c;         // Time pulse duty cycle (TP2)
const uint32_t UBLOX_CFG_TP_DUTY_LOCK_TP2 = 0x5005002d;    // Time pulse duty cycle when locked to GNSS time
const uint32_t UBLOX_CFG_TP_USER_DELAY_TP2 = 0x40050011;   // User-configurable time pulse delay (TP2)
const uint32_t UBLOX_CFG_TP_TP2_ENA = 0x10050012;          // Enable the second timepulse
const uint32_t UBLOX_CFG_TP_SYNC_GNSS_TP2 = 0x10050013;    // Sync time pulse to GNSS time or local clock
const uint32_t UBLOX_CFG_TP_USE_LOCKED_TP2 = 0x10050014;   // Use locked parameters when possible (TP2)
const uint32_t UBLOX_CFG_TP_ALIGN_TO_TOW_TP2 = 0x10050015; // Align time pulse to top of second (TP2)
const uint32_t UBLOX_CFG_TP_POL_TP2 = 0x10050016;          // Set time pulse polarity (TP2)
const uint32_t UBLOX_CFG_TP_TIMEGRID_TP2 = 0x20050017;     // Time grid to use (TP2)
const uint32_t UBLOX_CFG_TP_DRSTR_TP1 = 0x20050035;        // Set drive strength of TP1
const uint32_t UBLOX_CFG_TP_DRSTR_TP2 = 0x20050036;        // Set drive strength of TP2

// CFG-TXREADY: TX ready configuration
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_TXREADY_ENABLED = 0x10a20001;   // Flag to indicate if TX ready pin mechanism should be enabled
const uint32_t UBLOX_CFG_TXREADY_POLARITY = 0x10a20002;  // The polarity of the TX ready pin: false:high- active, true:low-active
const uint32_t UBLOX_CFG_TXREADY_PIN = 0x20a20003;       // Pin number to use for the TX ready functionality
const uint32_t UBLOX_CFG_TXREADY_THRESHOLD = 0x30a20004; // Amount of data that should be ready on the interface before triggering the TX ready pin
const uint32_t UBLOX_CFG_TXREADY_INTERFACE = 0x20a20005; // Interface where the TX ready feature should be linked to

// CFG-UART1: Configuration of the UART1 interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_UART1_BAUDRATE = 0x40520001; // The baud rate that should be configured on the UART1
const uint32_t UBLOX_CFG_UART1_STOPBITS = 0x20520002; // Number of stopbits that should be used on UART1
const uint32_t UBLOX_CFG_UART1_DATABITS = 0x20520003; // Number of databits that should be used on UART1
const uint32_t UBLOX_CFG_UART1_PARITY = 0x20520004;   // Parity mode that should be used on UART1
const uint32_t UBLOX_CFG_UART1_ENABLED = 0x10520005;  // Flag to indicate if the UART1 should be enabled

// CFG-UART1INPROT: Input protocol configuration of the UART1 interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_UART1INPROT_UBX = 0x10730001;    // Flag to indicate if UBX should be an input protocol on UART1
const uint32_t UBLOX_CFG_UART1INPROT_NMEA = 0x10730002;   // Flag to indicate if NMEA should be an input protocol on UART1
const uint32_t UBLOX_CFG_UART1INPROT_RTCM3X = 0x10730004; // Flag to indicate if RTCM3X should be an input protocol on UART1
const uint32_t UBLOX_CFG_UART1INPROT_SPARTN = 0x10730005; // Flag to indicate if SPARTN should be an input protocol on UART1

// CFG-UART1OUTPROT: Output protocol configuration of the UART1 interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_UART1OUTPROT_UBX = 0x10740001;    // Flag to indicate if UBX should be an output protocol on UART1
const uint32_t UBLOX_CFG_UART1OUTPROT_NMEA = 0x10740002;   // Flag to indicate if NMEA should be an output protocol on UART1
const uint32_t UBLOX_CFG_UART1OUTPROT_RTCM3X = 0x10740004; // Flag to indicate if RTCM3X should be an output protocol on UART1

// CFG-UART2: Configuration of the UART2 interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_UART2_BAUDRATE = 0x40530001; // The baud rate that should be configured on the UART2
const uint32_t UBLOX_CFG_UART2_STOPBITS = 0x20530002; // Number of stopbits that should be used on UART2
const uint32_t UBLOX_CFG_UART2_DATABITS = 0x20530003; // Number of databits that should be used on UART2
const uint32_t UBLOX_CFG_UART2_PARITY = 0x20530004;   // Parity mode that should be used on UART2
const uint32_t UBLOX_CFG_UART2_ENABLED = 0x10530005;  // Flag to indicate if the UART2 should be enabled
const uint32_t UBLOX_CFG_UART2_REMAP = 0x10530006;    // UART2 Remapping

// CFG-UART2INPROT: Input protocol configuration of the UART2 interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_UART2INPROT_UBX = 0x10750001;    // Flag to indicate if UBX should be an input protocol on UART2
const uint32_t UBLOX_CFG_UART2INPROT_NMEA = 0x10750002;   // Flag to indicate if NMEA should be an input protocol on UART2
const uint32_t UBLOX_CFG_UART2INPROT_RTCM3X = 0x10750004; // Flag to indicate if RTCM3X should be an input protocol on UART2
const uint32_t UBLOX_CFG_UART2INPROT_SPARTN = 0x10750005; // Flag to indicate if SPARTN should be an input protocol on UART2

// CFG-UART2OUTPROT: Output protocol configuration of the UART2 interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_UART2OUTPROT_UBX = 0x10760001;    // Flag to indicate if UBX should be an output protocol on UART2
const uint32_t UBLOX_CFG_UART2OUTPROT_NMEA = 0x10760002;   // Flag to indicate if NMEA should be an output protocol on UART2
const uint32_t UBLOX_CFG_UART2OUTPROT_RTCM3X = 0x10760004; // Flag to indicate if RTCM3X should be an output protocol on UART2

// CFG-USB: Configuration of the USB interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_USB_ENABLED = 0x10650001;        // Flag to indicate if the USB interface should be enabled
const uint32_t UBLOX_CFG_USB_SELFPOW = 0x10650002;        // Self-powered device
const uint32_t UBLOX_CFG_USB_VENDOR_ID = 0x3065000a;      // Vendor ID
const uint32_t UBLOX_CFG_USB_PRODUCT_ID = 0x3065000b;     // Vendor ID
const uint32_t UBLOX_CFG_USB_POWER = 0x3065000c;          // Power consumption
const uint32_t UBLOX_CFG_USB_VENDOR_STR0 = 0x5065000d;    // Vendor string characters 0-7
const uint32_t UBLOX_CFG_USB_VENDOR_STR1 = 0x5065000e;    // Vendor string characters 8-15
const uint32_t UBLOX_CFG_USB_VENDOR_STR2 = 0x5065000f;    // Vendor string characters 16-23
const uint32_t UBLOX_CFG_USB_VENDOR_STR3 = 0x50650010;    // Vendor string characters 24-31
const uint32_t UBLOX_CFG_USB_PRODUCT_STR0 = 0x50650011;   // Product string characters 0-7
const uint32_t UBLOX_CFG_USB_PRODUCT_STR1 = 0x50650012;   // Product string characters 8-15
const uint32_t UBLOX_CFG_USB_PRODUCT_STR2 = 0x50650013;   // Product string characters 16-23
const uint32_t UBLOX_CFG_USB_PRODUCT_STR3 = 0x50650014;   // Product string characters 24-31
const uint32_t UBLOX_CFG_USB_SERIAL_NO_STR0 = 0x50650015; // Serial number string characters 0-7
const uint32_t UBLOX_CFG_USB_SERIAL_NO_STR1 = 0x50650016; // Serial number string characters 8-15
const uint32_t UBLOX_CFG_USB_SERIAL_NO_STR2 = 0x50650017; // Serial number string characters 16-23
const uint32_t UBLOX_CFG_USB_SERIAL_NO_STR3 = 0x50650018; // Serial number string characters 24-31

// CFG-USBINPROT: Input protocol configuration of the USB interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_USBINPROT_UBX = 0x10770001;    // Flag to indicate if UBX should be an input protocol on USB
const uint32_t UBLOX_CFG_USBINPROT_NMEA = 0x10770002;   // Flag to indicate if NMEA should be an input protocol on USB
const uint32_t UBLOX_CFG_USBINPROT_RTCM3X = 0x10770004; // Flag to indicate if RTCM3X should be an input protocol on USB
const uint32_t UBLOX_CFG_USBINPROT_SPARTN = 0x10770005; // Flag to indicate if SPARTN should be an input protocol on USB

// CFG-USBOUTPROT: Output protocol configuration of the USB interface
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
const uint32_t UBLOX_CFG_USBOUTPROT_UBX = 0x10780001;    // Flag to indicate if UBX should be an output protocol on USB
const uint32_t UBLOX_CFG_USBOUTPROT_NMEA = 0x10780002;   // Flag to indicate if NMEA should be an output protocol on USB
const uint32_t UBLOX_CFG_USBOUTPROT_RTCM3X = 0x10780004; // Flag to indicate if RTCM3X should be an output protocol on USB

#endif
