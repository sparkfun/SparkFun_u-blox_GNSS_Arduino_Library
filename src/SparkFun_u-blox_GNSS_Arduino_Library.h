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

#ifndef SPARKFUN_UBLOX_ARDUINO_LIBRARY_H
#define SPARKFUN_UBLOX_ARDUINO_LIBRARY_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#include <SPI.h>

#include "u-blox_config_keys.h"
#include "u-blox_structs.h"

// Uncomment the next line (or add SFE_UBLOX_REDUCED_PROG_MEM as a compiler directive) to reduce the amount of program memory used by the library
//#define SFE_UBLOX_REDUCED_PROG_MEM // Uncommenting this line will delete the minor debug messages to save memory

// Uncomment the next line (or add SFE_UBLOX_DISABLE_AUTO_NMEA as a compiler directive) to reduce the amount of program memory used by the library
//#define SFE_UBLOX_DISABLE_AUTO_NMEA // Uncommenting this line will disable auto-NMEA support to save memory

// The code exceeds the program memory on the ATmega328P (Arduino Uno), so let's delete the minor debug messages and disable auto-NMEA support anyway
// However, the ATmega2560 and ATmega1280 _do_ have enough memory, so let's exclude those
#if !defined(SFE_UBLOX_REDUCED_PROG_MEM) && defined(ARDUINO_ARCH_AVR) && !defined(ARDUINO_AVR_MEGA2560) && !defined(ARDUINO_AVR_MEGA) && !defined(ARDUINO_AVR_ADK)
#define SFE_UBLOX_REDUCED_PROG_MEM
#endif
#if !defined(SFE_UBLOX_DISABLE_AUTO_NMEA) && defined(ARDUINO_ARCH_AVR) && !defined(ARDUINO_AVR_MEGA2560) && !defined(ARDUINO_AVR_MEGA) && !defined(ARDUINO_AVR_ADK)
#define SFE_UBLOX_DISABLE_AUTO_NMEA
#endif

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Define a digital pin to aid debugging
// Leave set to -1 if not needed
const int debugPin = -1;

// Global Status Returns
typedef enum
{
  SFE_UBLOX_STATUS_SUCCESS,
  SFE_UBLOX_STATUS_FAIL,
  SFE_UBLOX_STATUS_CRC_FAIL,
  SFE_UBLOX_STATUS_TIMEOUT,
  SFE_UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognised, invalid or that the module is too busy to respond
  SFE_UBLOX_STATUS_OUT_OF_RANGE,
  SFE_UBLOX_STATUS_INVALID_ARG,
  SFE_UBLOX_STATUS_INVALID_OPERATION,
  SFE_UBLOX_STATUS_MEM_ERR,
  SFE_UBLOX_STATUS_HW_ERR,
  SFE_UBLOX_STATUS_DATA_SENT,     // This indicates that a 'set' was successful
  SFE_UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
  SFE_UBLOX_STATUS_I2C_COMM_FAILURE,
  SFE_UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
} sfe_ublox_status_e;

// ubxPacket validity
typedef enum
{
  SFE_UBLOX_PACKET_VALIDITY_NOT_VALID,
  SFE_UBLOX_PACKET_VALIDITY_VALID,
  SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
  SFE_UBLOX_PACKET_NOTACKNOWLEDGED // This indicates that we received a NACK
} sfe_ublox_packet_validity_e;

// Identify which packet buffer is in use:
// packetCfg (or a custom packet), packetAck or packetBuf
// packetAuto is used to store expected "automatic" messages
typedef enum
{
  SFE_UBLOX_PACKET_PACKETCFG,
  SFE_UBLOX_PACKET_PACKETACK,
  SFE_UBLOX_PACKET_PACKETBUF,
  SFE_UBLOX_PACKET_PACKETAUTO
} sfe_ublox_packet_buffer_e;

// Define a struct to allow selective logging / processing of NMEA messages
// Set the individual bits to pass the NMEA messages to the file buffer and/or processNMEA
// Setting bits.all will pass all messages to the file buffer and processNMEA
typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;
      uint32_t UBX_NMEA_DTM : 1;
      uint32_t UBX_NMEA_GAQ : 1;
      uint32_t UBX_NMEA_GBQ : 1;
      uint32_t UBX_NMEA_GBS : 1;
      uint32_t UBX_NMEA_GGA : 1;
      uint32_t UBX_NMEA_GLL : 1;
      uint32_t UBX_NMEA_GLQ : 1;
      uint32_t UBX_NMEA_GNQ : 1;
      uint32_t UBX_NMEA_GNS : 1;
      uint32_t UBX_NMEA_GPQ : 1;
      uint32_t UBX_NMEA_GQQ : 1;
      uint32_t UBX_NMEA_GRS : 1;
      uint32_t UBX_NMEA_GSA : 1;
      uint32_t UBX_NMEA_GST : 1;
      uint32_t UBX_NMEA_GSV : 1;
      uint32_t UBX_NMEA_RLM : 1;
      uint32_t UBX_NMEA_RMC : 1;
      uint32_t UBX_NMEA_TXT : 1;
      uint32_t UBX_NMEA_VLW : 1;
      uint32_t UBX_NMEA_VTG : 1;
      uint32_t UBX_NMEA_ZDA : 1;
    } bits;
  };
} sfe_ublox_nmea_filtering_t;

// Define an enum to make it easy to enable/disable selected NMEA messages for logging / processing
typedef enum
{
  SFE_UBLOX_FILTER_NMEA_ALL = 0x00000001,
  SFE_UBLOX_FILTER_NMEA_DTM = 0x00000002,
  SFE_UBLOX_FILTER_NMEA_GAQ = 0x00000004,
  SFE_UBLOX_FILTER_NMEA_GBQ = 0x00000008,
  SFE_UBLOX_FILTER_NMEA_GBS = 0x00000010,
  SFE_UBLOX_FILTER_NMEA_GGA = 0x00000020,
  SFE_UBLOX_FILTER_NMEA_GLL = 0x00000040,
  SFE_UBLOX_FILTER_NMEA_GLQ = 0x00000080,
  SFE_UBLOX_FILTER_NMEA_GNQ = 0x00000100,
  SFE_UBLOX_FILTER_NMEA_GNS = 0x00000200,
  SFE_UBLOX_FILTER_NMEA_GPQ = 0x00000400,
  SFE_UBLOX_FILTER_NMEA_GQQ = 0x00000800,
  SFE_UBLOX_FILTER_NMEA_GRS = 0x00001000,
  SFE_UBLOX_FILTER_NMEA_GSA = 0x00002000,
  SFE_UBLOX_FILTER_NMEA_GST = 0x00004000,
  SFE_UBLOX_FILTER_NMEA_GSV = 0x00008000,
  SFE_UBLOX_FILTER_NMEA_RLM = 0x00010000,
  SFE_UBLOX_FILTER_NMEA_RMC = 0x00020000,
  SFE_UBLOX_FILTER_NMEA_TXT = 0x00040000,
  SFE_UBLOX_FILTER_NMEA_VLW = 0x00080000,
  SFE_UBLOX_FILTER_NMEA_VTG = 0x00100000,
  SFE_UBLOX_FILTER_NMEA_ZDA = 0x00200000
} sfe_ublox_nmea_filtering_e;

// Registers
const uint8_t UBX_SYNCH_1 = 0xB5;
const uint8_t UBX_SYNCH_2 = 0x62;

// The following are UBX Class IDs. Descriptions taken from ZED-F9P Interface Description Document page 32, NEO-M8P Interface Description page 145
const uint8_t UBX_CLASS_NAV = 0x01;  // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
const uint8_t UBX_CLASS_RXM = 0x02;  // Receiver Manager Messages: Satellite Status, RTC Status
const uint8_t UBX_CLASS_INF = 0x04;  // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
const uint8_t UBX_CLASS_ACK = 0x05;  // Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
const uint8_t UBX_CLASS_CFG = 0x06;  // Configuration Input Messages: Configure the receiver.
const uint8_t UBX_CLASS_UPD = 0x09;  // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
const uint8_t UBX_CLASS_MON = 0x0A;  // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
const uint8_t UBX_CLASS_AID = 0x0B;  //(NEO-M8P ONLY!!!) AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
const uint8_t UBX_CLASS_TIM = 0x0D;  // Timing Messages: Time Pulse Output, Time Mark Results
const uint8_t UBX_CLASS_ESF = 0x10;  //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information
const uint8_t UBX_CLASS_MGA = 0x13;  // Multiple GNSS Assistance Messages: Assistance data for various GNSS
const uint8_t UBX_CLASS_LOG = 0x21;  // Logging Messages: Log creation, deletion, info and retrieval
const uint8_t UBX_CLASS_SEC = 0x27;  // Security Feature Messages
const uint8_t UBX_CLASS_HNR = 0x28;  //(NEO-M8P ONLY!!!) High Rate Navigation Results Messages: High rate time, position speed, heading
const uint8_t UBX_CLASS_NMEA = 0xF0; // NMEA Strings: standard NMEA strings
const uint8_t UBX_CLASS_PUBX = 0xF1; // Proprietary NMEA-format messages defined by u-blox

// Class: CFG
// The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34 and NEO-M9N Interface Description pg 47-48
const uint8_t UBX_CFG_ANT = 0x13;       // Antenna Control Settings. Used to configure the antenna control settings
const uint8_t UBX_CFG_BATCH = 0x93;     // Get/set data batching configuration.
const uint8_t UBX_CFG_CFG = 0x09;       // Clear, Save, and Load Configurations. Used to save current configuration
const uint8_t UBX_CFG_DAT = 0x06;       // Set User-defined Datum or The currently defined Datum
const uint8_t UBX_CFG_DGNSS = 0x70;     // DGNSS configuration
const uint8_t UBX_CFG_ESFALG = 0x56;    // ESF alignment
const uint8_t UBX_CFG_ESFA = 0x4C;      // ESF accelerometer
const uint8_t UBX_CFG_ESFG = 0x4D;      // ESF gyro
const uint8_t UBX_CFG_GEOFENCE = 0x69;  // Geofencing configuration. Used to configure a geofence
const uint8_t UBX_CFG_GNSS = 0x3E;      // GNSS system configuration
const uint8_t UBX_CFG_HNR = 0x5C;       // High Navigation Rate
const uint8_t UBX_CFG_INF = 0x02;       // Depending on packet length, either: poll configuration for one protocol, or information message configuration
const uint8_t UBX_CFG_ITFM = 0x39;      // Jamming/Interference Monitor configuration
const uint8_t UBX_CFG_LOGFILTER = 0x47; // Data Logger Configuration
const uint8_t UBX_CFG_MSG = 0x01;       // Poll a message configuration, or Set Message Rate(s), or Set Message Rate
const uint8_t UBX_CFG_NAV5 = 0x24;      // Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
const uint8_t UBX_CFG_NAVX5 = 0x23;     // Navigation Engine Expert Settings
const uint8_t UBX_CFG_NMEA = 0x17;      // Extended NMEA protocol configuration V1
const uint8_t UBX_CFG_ODO = 0x1E;       // Odometer, Low-speed COG Engine Settings
const uint8_t UBX_CFG_PM2 = 0x3B;       // Extended power management configuration
const uint8_t UBX_CFG_PMS = 0x86;       // Power mode setup
const uint8_t UBX_CFG_PRT = 0x00;       // Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
const uint8_t UBX_CFG_PWR = 0x57;       // Put receiver in a defined power state
const uint8_t UBX_CFG_RATE = 0x08;      // Navigation/Measurement Rate Settings. Used to set port baud rates.
const uint8_t UBX_CFG_RINV = 0x34;      // Contents of Remote Inventory
const uint8_t UBX_CFG_RST = 0x04;       // Reset Receiver / Clear Backup Data Structures. Used to reset device.
const uint8_t UBX_CFG_RXM = 0x11;       // RXM configuration
const uint8_t UBX_CFG_SBAS = 0x16;      // SBAS configuration
const uint8_t UBX_CFG_TMODE3 = 0x71;    // Time Mode Settings 3. Used to enable Survey In Mode
const uint8_t UBX_CFG_TP5 = 0x31;       // Time Pulse Parameters
const uint8_t UBX_CFG_USB = 0x1B;       // USB Configuration
const uint8_t UBX_CFG_VALDEL = 0x8C;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
const uint8_t UBX_CFG_VALGET = 0x8B;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Configuration Items
const uint8_t UBX_CFG_VALSET = 0x8A;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.

// Class: NMEA
// The following are used to enable NMEA messages. Descriptions come from the NMEA messages overview in the ZED-F9P Interface Description
const uint8_t UBX_NMEA_MSB = 0xF0; // All NMEA enable commands have 0xF0 as MSB. Equal to UBX_CLASS_NMEA
const uint8_t UBX_NMEA_DTM = 0x0A; // GxDTM (datum reference)
const uint8_t UBX_NMEA_GAQ = 0x45; // GxGAQ (poll a standard message (if the current talker ID is GA))
const uint8_t UBX_NMEA_GBQ = 0x44; // GxGBQ (poll a standard message (if the current Talker ID is GB))
const uint8_t UBX_NMEA_GBS = 0x09; // GxGBS (GNSS satellite fault detection)
const uint8_t UBX_NMEA_GGA = 0x00; // GxGGA (Global positioning system fix data)
const uint8_t UBX_NMEA_GLL = 0x01; // GxGLL (latitude and long, whith time of position fix and status)
const uint8_t UBX_NMEA_GLQ = 0x43; // GxGLQ (poll a standard message (if the current Talker ID is GL))
const uint8_t UBX_NMEA_GNQ = 0x42; // GxGNQ (poll a standard message (if the current Talker ID is GN))
const uint8_t UBX_NMEA_GNS = 0x0D; // GxGNS (GNSS fix data)
const uint8_t UBX_NMEA_GPQ = 0x40; // GxGPQ (poll a standard message (if the current Talker ID is GP))
const uint8_t UBX_NMEA_GQQ = 0x47; // GxGQQ (poll a standard message (if the current Talker ID is GQ))
const uint8_t UBX_NMEA_GRS = 0x06; // GxGRS (GNSS range residuals)
const uint8_t UBX_NMEA_GSA = 0x02; // GxGSA (GNSS DOP and Active satellites)
const uint8_t UBX_NMEA_GST = 0x07; // GxGST (GNSS Pseudo Range Error Statistics)
const uint8_t UBX_NMEA_GSV = 0x03; // GxGSV (GNSS satellites in view)
const uint8_t UBX_NMEA_RLM = 0x0B; // GxRMC (Return link message (RLM))
const uint8_t UBX_NMEA_RMC = 0x04; // GxRMC (Recommended minimum data)
const uint8_t UBX_NMEA_TXT = 0x41; // GxTXT (text transmission)
const uint8_t UBX_NMEA_VLW = 0x0F; // GxVLW (dual ground/water distance)
const uint8_t UBX_NMEA_VTG = 0x05; // GxVTG (course over ground and Ground speed)
const uint8_t UBX_NMEA_ZDA = 0x08; // GxZDA (Time and Date)

// The following are used to configure the NMEA protocol main talker ID and GSV talker ID
const uint8_t UBX_NMEA_MAINTALKERID_NOTOVERRIDDEN = 0x00; // main talker ID is system dependent
const uint8_t UBX_NMEA_MAINTALKERID_GP = 0x01;            // main talker ID is GPS
const uint8_t UBX_NMEA_MAINTALKERID_GL = 0x02;            // main talker ID is GLONASS
const uint8_t UBX_NMEA_MAINTALKERID_GN = 0x03;            // main talker ID is combined receiver
const uint8_t UBX_NMEA_MAINTALKERID_GA = 0x04;            // main talker ID is Galileo
const uint8_t UBX_NMEA_MAINTALKERID_GB = 0x05;            // main talker ID is BeiDou
const uint8_t UBX_NMEA_GSVTALKERID_GNSS = 0x00;           // GNSS specific Talker ID (as defined by NMEA)
const uint8_t UBX_NMEA_GSVTALKERID_MAIN = 0x01;           // use the main Talker ID

// Class: PUBX
// The following are used to enable PUBX messages with configureMessage
// See the M8 receiver description & protocol specification for more details
const uint8_t UBX_PUBX_CONFIG = 0x41;   // Set protocols and baud rate
const uint8_t UBX_PUBX_POSITION = 0x00; // Lat/Long position data
const uint8_t UBX_PUBX_RATE = 0x40;     // Set/get NMEA message output rate
const uint8_t UBX_PUBX_SVSTATUS = 0x03; // Satellite status
const uint8_t UBX_PUBX_TIME = 0x04;     // Time of day and clock information

// Class: HNR
// The following are used to configure the HNR message rates
const uint8_t UBX_HNR_ATT = 0x01; // HNR Attitude
const uint8_t UBX_HNR_INS = 0x02; // HNR Vehicle Dynamics
const uint8_t UBX_HNR_PVT = 0x00; // HNR PVT

// Class: INF
// The following are used to configure INF UBX messages (information messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_INF_CLASS = 0x04;   // All INF messages have 0x04 as the class
const uint8_t UBX_INF_DEBUG = 0x04;   // ASCII output with debug contents
const uint8_t UBX_INF_ERROR = 0x00;   // ASCII output with error contents
const uint8_t UBX_INF_NOTICE = 0x02;  // ASCII output with informational contents
const uint8_t UBX_INF_TEST = 0x03;    // ASCII output with test contents
const uint8_t UBX_INF_WARNING = 0x01; // ASCII output with warning contents

// Class: LOG
// The following are used to configure LOG UBX messages (loggings messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_LOG_CREATE = 0x07;           // Create Log File
const uint8_t UBX_LOG_ERASE = 0x03;            // Erase Logged Data
const uint8_t UBX_LOG_FINDTIME = 0x0E;         // Find index of a log entry based on a given time, or response to FINDTIME requested
const uint8_t UBX_LOG_INFO = 0x08;             // Poll for log information, or Log information
const uint8_t UBX_LOG_RETRIEVEPOSEXTRA = 0x0F; // Odometer log entry
const uint8_t UBX_LOG_RETRIEVEPOS = 0x0B;      // Position fix log entry
const uint8_t UBX_LOG_RETRIEVESTRING = 0x0D;   // Byte string log entry
const uint8_t UBX_LOG_RETRIEVE = 0x09;         // Request log data
const uint8_t UBX_LOG_STRING = 0x04;           // Store arbitrary string on on-board flash

// Class: MGA
// The following are used to configure MGA UBX messages (Multiple GNSS Assistance Messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_MGA_ACK_DATA0 = 0x60;      // Multiple GNSS Acknowledge message
const uint8_t UBX_MGA_ANO = 0x20;            // Multiple GNSS AssistNow Offline assistance - NOT SUPPORTED BY THE ZED-F9P! "The ZED-F9P supports AssistNow Online only."
const uint8_t UBX_MGA_BDS_EPH = 0x03;        // BDS Ephemeris Assistance
const uint8_t UBX_MGA_BDS_ALM = 0x03;        // BDS Almanac Assistance
const uint8_t UBX_MGA_BDS_HEALTH = 0x03;     // BDS Health Assistance
const uint8_t UBX_MGA_BDS_UTC = 0x03;        // BDS UTC Assistance
const uint8_t UBX_MGA_BDS_IONO = 0x03;       // BDS Ionospheric Assistance
const uint8_t UBX_MGA_DBD = 0x80;            // Either: Poll the Navigation Database, or Navigation Database Dump Entry
const uint8_t UBX_MGA_GAL_EPH = 0x02;        // Galileo Ephemeris Assistance
const uint8_t UBX_MGA_GAL_ALM = 0x02;        // Galileo Almanac Assitance
const uint8_t UBX_MGA_GAL_TIMOFFSET = 0x02;  // Galileo GPS time offset assistance
const uint8_t UBX_MGA_GAL_UTC = 0x02;        // Galileo UTC Assistance
const uint8_t UBX_MGA_GLO_EPH = 0x06;        // GLONASS Ephemeris Assistance
const uint8_t UBX_MGA_GLO_ALM = 0x06;        // GLONASS Almanac Assistance
const uint8_t UBX_MGA_GLO_TIMEOFFSET = 0x06; // GLONASS Auxiliary Time Offset Assistance
const uint8_t UBX_MGA_GPS_EPH = 0x00;        // GPS Ephemeris Assistance
const uint8_t UBX_MGA_GPS_ALM = 0x00;        // GPS Almanac Assistance
const uint8_t UBX_MGA_GPS_HEALTH = 0x00;     // GPS Health Assistance
const uint8_t UBX_MGA_GPS_UTC = 0x00;        // GPS UTC Assistance
const uint8_t UBX_MGA_GPS_IONO = 0x00;       // GPS Ionosphere Assistance
const uint8_t UBX_MGA_INI_POS_XYZ = 0x40;    // Initial Position Assistance
const uint8_t UBX_MGA_INI_POS_LLH = 0x40;    // Initial Position Assitance
const uint8_t UBX_MGA_INI_TIME_UTC = 0x40;   // Initial Time Assistance
const uint8_t UBX_MGA_INI_TIME_GNSS = 0x40;  // Initial Time Assistance
const uint8_t UBX_MGA_INI_CLKD = 0x40;       // Initial Clock Drift Assitance
const uint8_t UBX_MGA_INI_FREQ = 0x40;       // Initial Frequency Assistance
const uint8_t UBX_MGA_INI_EOP = 0x40;        // Earth Orientation Parameters Assistance
const uint8_t UBX_MGA_QZSS_EPH = 0x05;       // QZSS Ephemeris Assistance
const uint8_t UBX_MGA_QZSS_ALM = 0x05;       // QZSS Almanac Assistance
const uint8_t UBX_MGA_QZAA_HEALTH = 0x05;    // QZSS Health Assistance

// Class: MON
// The following are used to configure the MON UBX messages (monitoring messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35)
const uint8_t UBX_MON_COMMS = 0x36; // Comm port information
const uint8_t UBX_MON_GNSS = 0x28;  // Information message major GNSS selection
const uint8_t UBX_MON_HW2 = 0x0B;   // Extended Hardware Status
const uint8_t UBX_MON_HW3 = 0x37;   // HW I/O pin information
const uint8_t UBX_MON_HW = 0x09;    // Hardware Status
const uint8_t UBX_MON_IO = 0x02;    // I/O Subsystem Status
const uint8_t UBX_MON_MSGPP = 0x06; // Message Parse and Process Status
const uint8_t UBX_MON_PATCH = 0x27; // Output information about installed patches
const uint8_t UBX_MON_RF = 0x38;    // RF information
const uint8_t UBX_MON_RXBUF = 0x07; // Receiver Buffer Status
const uint8_t UBX_MON_RXR = 0x21;   // Receiver Status Information
const uint8_t UBX_MON_SPAN = 0x31;  // Signal characteristics
const uint8_t UBX_MON_SYS = 0x39;   // Current system performance information
const uint8_t UBX_MON_TXBUF = 0x08; // Transmitter Buffer Status. Used for query tx buffer size/state.
const uint8_t UBX_MON_VER = 0x04;   // Receiver/Software Version. Used for obtaining Protocol Version.

// Class: NAV
// The following are used to configure the NAV UBX messages (navigation results messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35-36)
const uint8_t UBX_NAV_ATT = 0x05;       // Vehicle "Attitude" Solution
const uint8_t UBX_NAV_CLOCK = 0x22;     // Clock Solution
const uint8_t UBX_NAV_DOP = 0x04;       // Dilution of precision
const uint8_t UBX_NAV_EOE = 0x61;       // End of Epoch
const uint8_t UBX_NAV_GEOFENCE = 0x39;  // Geofencing status. Used to poll the geofence status
const uint8_t UBX_NAV_HPPOSECEF = 0x13; // High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
const uint8_t UBX_NAV_HPPOSLLH = 0x14;  // High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision
const uint8_t UBX_NAV_ODO = 0x09;       // Odometer Solution
const uint8_t UBX_NAV_ORB = 0x34;       // GNSS Orbit Database Info
const uint8_t UBX_NAV_PL = 0x62;        // Protection Level Information
const uint8_t UBX_NAV_POSECEF = 0x01;   // Position Solution in ECEF
const uint8_t UBX_NAV_POSLLH = 0x02;    // Geodetic Position Solution
const uint8_t UBX_NAV_PVT = 0x07;       // All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
const uint8_t UBX_NAV_PVAT = 0x17;      // Navigation position velocity attitude time solution (ZED-F9R only)
const uint8_t UBX_NAV_RELPOSNED = 0x3C; // Relative Positioning Information in NED frame
const uint8_t UBX_NAV_RESETODO = 0x10;  // Reset odometer
const uint8_t UBX_NAV_SAT = 0x35;       // Satellite Information
const uint8_t UBX_NAV_SIG = 0x43;       // Signal Information
const uint8_t UBX_NAV_STATUS = 0x03;    // Receiver Navigation Status
const uint8_t UBX_NAV_SVIN = 0x3B;      // Survey-in data. Used for checking Survey In status
const uint8_t UBX_NAV_TIMEBDS = 0x24;   // BDS Time Solution
const uint8_t UBX_NAV_TIMEGAL = 0x25;   // Galileo Time Solution
const uint8_t UBX_NAV_TIMEGLO = 0x23;   // GLO Time Solution
const uint8_t UBX_NAV_TIMEGPS = 0x20;   // GPS Time Solution
const uint8_t UBX_NAV_TIMELS = 0x26;    // Leap second event information
const uint8_t UBX_NAV_TIMEUTC = 0x21;   // UTC Time Solution
const uint8_t UBX_NAV_VELECEF = 0x11;   // Velocity Solution in ECEF
const uint8_t UBX_NAV_VELNED = 0x12;    // Velocity Solution in NED
const uint8_t UBX_NAV_AOPSTATUS = 0x60; // AssistNow Autonomous status

// Class: RXM
// The following are used to configure the RXM UBX messages (receiver manager messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_RXM_COR = 0x34;       // Differential correction input status
const uint8_t UBX_RXM_MEASX = 0x14;     // Satellite Measurements for RRLP
const uint8_t UBX_RXM_PMP = 0x72;       // PMP raw data (NEO-D9S) (two different versions) (packet size for version 0x01 is variable)
const uint8_t UBX_RXM_QZSSL6 = 0x73;    // QZSSL6 data (NEO-D9C)
const uint8_t UBX_RXM_PMREQ = 0x41;     // Requests a Power Management task (two different packet sizes)
const uint8_t UBX_RXM_RAWX = 0x15;      // Multi-GNSS Raw Measurement Data
const uint8_t UBX_RXM_RLM = 0x59;       // Galileo SAR Short-RLM report (two different packet sizes)
const uint8_t UBX_RXM_RTCM = 0x32;      // RTCM input status
const uint8_t UBX_RXM_SFRBX = 0x13;     // Broadcast Navigation Data Subframe
const uint8_t UBX_RXM_SPARTN = 0x33;    // SPARTN input status
const uint8_t UBX_RXM_SPARTNKEY = 0x36; // Poll/transfer dynamic SPARTN keys

// Class: SEC
// The following are used to configure the SEC UBX messages (security feature messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_SEC_UNIQID = 0x03; // Unique chip ID

// Class: TIM
// The following are used to configure the TIM UBX messages (timing messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_TIM_TM2 = 0x03;  // Time mark data
const uint8_t UBX_TIM_TP = 0x01;   // Time Pulse Timedata
const uint8_t UBX_TIM_VRFY = 0x06; // Sourced Time Verification

// Class: UPD
// The following are used to configure the UPD UBX messages (firmware update messages). Descriptions from UBX messages overview (ZED-F9P Interface Description Document page 36)
const uint8_t UBX_UPD_SOS = 0x14; // Poll Backup Fil Restore Status, Create Backup File in Flash, Clear Backup File in Flash, Backup File Creation Acknowledge, System Restored from Backup

// The following are used to enable RTCM messages
const uint8_t UBX_RTCM_MSB = 0xF5;    // All RTCM enable commands have 0xF5 as MSB
const uint8_t UBX_RTCM_1005 = 0x05;   // Stationary RTK reference ARP
const uint8_t UBX_RTCM_1074 = 0x4A;   // GPS MSM4
const uint8_t UBX_RTCM_1077 = 0x4D;   // GPS MSM7
const uint8_t UBX_RTCM_1084 = 0x54;   // GLONASS MSM4
const uint8_t UBX_RTCM_1087 = 0x57;   // GLONASS MSM7
const uint8_t UBX_RTCM_1094 = 0x5E;   // Galileo MSM4
const uint8_t UBX_RTCM_1097 = 0x61;   // Galileo MSM7
const uint8_t UBX_RTCM_1124 = 0x7C;   // BeiDou MSM4
const uint8_t UBX_RTCM_1127 = 0x7F;   // BeiDou MSM7
const uint8_t UBX_RTCM_1230 = 0xE6;   // GLONASS code-phase biases, set to once every 10 seconds
const uint8_t UBX_RTCM_4072_0 = 0xFE; // Reference station PVT (ublox proprietary RTCM message)
const uint8_t UBX_RTCM_4072_1 = 0xFD; // Additional reference station information (ublox proprietary RTCM message)

// Class: ACK
const uint8_t UBX_ACK_NACK = 0x00;
const uint8_t UBX_ACK_ACK = 0x01;
const uint8_t UBX_ACK_NONE = 0x02; // Not a real value

// Class: ESF
//  The following constants are used to get External Sensor Measurements and Status
//  Information.
const uint8_t UBX_ESF_MEAS = 0x02;
const uint8_t UBX_ESF_RAW = 0x03;
const uint8_t UBX_ESF_STATUS = 0x10;
const uint8_t UBX_ESF_RESETALG = 0x13;
const uint8_t UBX_ESF_ALG = 0x14;
const uint8_t UBX_ESF_INS = 0x15; // 36 bytes

const uint8_t SVIN_MODE_DISABLE = 0x00;
const uint8_t SVIN_MODE_ENABLE = 0x01;

// The following consts are used to configure the various ports and streams for those ports. See -CFG-PRT.
const uint8_t COM_PORT_I2C = 0;
const uint8_t COM_PORT_UART1 = 1;
const uint8_t COM_PORT_UART2 = 2;
const uint8_t COM_PORT_USB = 3;
const uint8_t COM_PORT_SPI = 4;

const uint8_t COM_TYPE_UBX = (1 << 0);
const uint8_t COM_TYPE_NMEA = (1 << 1);
const uint8_t COM_TYPE_RTCM3 = (1 << 5);
const uint8_t COM_TYPE_SPARTN = (1 << 6);

// Odometer configuration - flags
const uint8_t UBX_CFG_ODO_USE_ODO = (1 << 0);
const uint8_t UBX_CFG_ODO_USE_COG = (1 << 1);
const uint8_t UBX_CFG_ODO_OUT_LP_VEL = (1 << 2);
const uint8_t UBX_CFG_ODO_OUT_LP_COG = (1 << 3);

// Odometer configuration - odoCfg
enum odoCfg_e
{
  UBX_CFG_ODO_RUN = 0,
  UBX_CFG_ODO_CYCLE,
  UBX_CFG_ODO_SWIM,
  UBX_CFG_ODO_CAR,
  UBX_CFG_ODO_CUSTOM,
};

// Configuration Sub-Section mask definitions for saveConfigSelective (UBX-CFG-CFG)
const uint32_t VAL_CFG_SUBSEC_IOPORT = 0x00000001;   // ioPort - communications port settings (causes IO system reset!)
const uint32_t VAL_CFG_SUBSEC_MSGCONF = 0x00000002;  // msgConf - message configuration
const uint32_t VAL_CFG_SUBSEC_INFMSG = 0x00000004;   // infMsg - INF message configuration
const uint32_t VAL_CFG_SUBSEC_NAVCONF = 0x00000008;  // navConf - navigation configuration
const uint32_t VAL_CFG_SUBSEC_RXMCONF = 0x00000010;  // rxmConf - receiver manager configuration
const uint32_t VAL_CFG_SUBSEC_SENCONF = 0x00000100;  // senConf - sensor interface configuration (requires protocol 19+)
const uint32_t VAL_CFG_SUBSEC_RINVCONF = 0x00000200; // rinvConf - remove inventory configuration
const uint32_t VAL_CFG_SUBSEC_ANTCONF = 0x00000400;  // antConf - antenna configuration
const uint32_t VAL_CFG_SUBSEC_LOGCONF = 0x00000800;  // logConf - logging configuration
const uint32_t VAL_CFG_SUBSEC_FTSCONF = 0x00001000;  // ftsConf - FTS configuration (FTS products only)

// Bitfield wakeupSources for UBX_RXM_PMREQ
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX = 0x00000008;  // uartrx
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0 = 0x00000020; // extint0
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT1 = 0x00000040; // extint1
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_SPICS = 0x00000080;   // spics

enum dynModel // Possible values for the dynamic platform model, which provide more accuract position output for the situation. Description extracted from ZED-F9P Integration Manual
{
  DYN_MODEL_PORTABLE = 0, // Applications with low acceleration, e.g. portable devices. Suitable for most situations.
  // 1 is not defined
  DYN_MODEL_STATIONARY = 2, // Used in timing applications (antenna must be stationary) or other stationary applications. Velocity restricted to 0 m/s. Zero dynamics assumed.
  DYN_MODEL_PEDESTRIAN,     // Applications with low acceleration and speed, e.g. how a pedestrian would move. Low acceleration assumed.
  DYN_MODEL_AUTOMOTIVE,     // Used for applications with equivalent dynamics to those of a passenger car. Low vertical acceleration assumed
  DYN_MODEL_SEA,            // Recommended for applications at sea, with zero vertical velocity. Zero vertical velocity assumed. Sea level assumed.
  DYN_MODEL_AIRBORNE1g,     // Airborne <1g acceleration. Used for applications with a higher dynamic range and greater vertical acceleration than a passenger car. No 2D position fixes supported.
  DYN_MODEL_AIRBORNE2g,     // Airborne <2g acceleration. Recommended for typical airborne environments. No 2D position fixes supported.
  DYN_MODEL_AIRBORNE4g,     // Airborne <4g acceleration. Only recommended for extremely dynamic environments. No 2D position fixes supported.
  DYN_MODEL_WRIST,          // Not supported in protocol versions less than 18. Only recommended for wrist worn applications. Receiver will filter out arm motion.
  DYN_MODEL_BIKE,           // Supported in protocol versions 19.2. (not available in all products)
  DYN_MODEL_MOWER,          // Added in HPS 1.21 (not available in all products)
  DYN_MODEL_ESCOOTER,       // Added in HPS 1.21 (not available in all products)
  DYN_MODEL_UNKNOWN = 255   // getDynamicModel will return 255 if sendCommand fails
};

// The GNSS identifiers - used by UBX-CFG-GNSS (0x06 0x3E) GNSS system configuration
enum sfe_ublox_gnss_ids_e
{
  SFE_UBLOX_GNSS_ID_GPS,
  SFE_UBLOX_GNSS_ID_SBAS,
  SFE_UBLOX_GNSS_ID_GALILEO,
  SFE_UBLOX_GNSS_ID_BEIDOU,
  SFE_UBLOX_GNSS_ID_IMES,
  SFE_UBLOX_GNSS_ID_QZSS,
  SFE_UBLOX_GNSS_ID_GLONASS
};

// The GNSS identifiers of leap second event info source - used by UBX-NAV-TIMELS
enum sfe_ublox_ls_src_e
{
  SFE_UBLOX_LS_SRC_DEFAULT,
  SFE_UBLOX_LS_SRC_GLONASS,
  SFE_UBLOX_LS_SRC_GPS,
  SFE_UBLOX_LS_SRC_SBAS,
  SFE_UBLOX_LS_SRC_BEIDOU,
  SFE_UBLOX_LS_SRC_GALILEO,
  SFE_UBLOX_LS_SRC_AIDED,
  SFE_UBLOX_LS_SRC_CONFIGURED,
  SFE_UBLOX_LS_SRC_UNKNOWN = 255
};

typedef enum
{
  SFE_UBLOX_MGA_ASSIST_ACK_NO,     // Do not expect UBX-MGA-ACK's. If the module outputs them, they will be ignored
  SFE_UBLOX_MGA_ASSIST_ACK_YES,    // Expect and check for UBX-MGA-ACK's
  SFE_UBLOX_MGA_ASSIST_ACK_ENQUIRE // Check UBX-CFG-NAVX5 ackAiding to determine if UBX-MGA-ACK's are expected
} sfe_ublox_mga_assist_ack_e;

// The infoCode byte included in UBX-MGA-ACK-DATA0
enum sfe_ublox_mga_ack_infocode_e
{
  SFE_UBLOX_MGA_ACK_INFOCODE_ACCEPTED,
  SFE_UBLOX_MGA_ACK_INFOCODE_NO_TIME,
  SFE_UBLOX_MGA_ACK_INFOCODE_NOT_SUPPORTED,
  SFE_UBLOX_MGA_ACK_INFOCODE_SIZE_MISMATCH,
  SFE_UBLOX_MGA_ACK_INFOCODE_NOT_STORED,
  SFE_UBLOX_MGA_ACK_INFOCODE_NOT_READY,
  SFE_UBLOX_MGA_ACK_INFOCODE_TYPE_UNKNOWN
};

// The mainTalkerId, set by UBX-CFG-NMEA setMainTalkerID
enum sfe_ublox_talker_ids_e
{
  SFE_UBLOX_MAIN_TALKER_ID_DEFAULT,
  SFE_UBLOX_MAIN_TALKER_ID_GP,
  SFE_UBLOX_MAIN_TALKER_ID_GL,
  SFE_UBLOX_MAIN_TALKER_ID_GN,
  SFE_UBLOX_MAIN_TALKER_ID_GA,
  SFE_UBLOX_MAIN_TALKER_ID_GB,
  SFE_UBLOX_MAIN_TALKER_ID_GQ
};

// The DGNSS differential mode
enum sfe_ublox_dgnss_mode_e
{
  SFE_UBLOX_DGNSS_MODE_FLOAT = 2, // No attempts are made to fix ambiguities
  SFE_UBLOX_DGNSS_MODE_FIXED      // Ambiguities are fixed whenever possible
};

// Values for UBX-CFG-PMS
enum sfe_ublox_pms_mode_e
{
  SFE_UBLOX_PMS_MODE_FULLPOWER = 0,
  SFE_UBLOX_PMS_MODE_BALANCED,
  SFE_UBLOX_PMS_MODE_INTERVAL,
  SFE_UBLOX_PMS_MODE_AGGRESSIVE_1HZ,
  SFE_UBLOX_PMS_MODE_AGGRESSIVE_2HZ,
  SFE_UBLOX_PMS_MODE_AGGRESSIVE_4HZ,
  SFE_UBLOX_PMS_MODE_INVALID = 0xff
};

//Values for UBX-CFG-RXM
enum sfe_ublox_rxm_mode_e
{
  SFE_UBLOX_CFG_RXM_CONTINUOUS = 0,
  SFE_UBLOX_CFG_RXM_POWERSAVE = 1
};

//-=-=-=-=-

#ifndef MAX_PAYLOAD_SIZE
// v2.0: keep this for backwards-compatibility, but this is largely superseded by setPacketCfgPayloadSize
#define MAX_PAYLOAD_SIZE 256 // We need ~220 bytes for getProtocolVersion on most ublox modules
//#define MAX_PAYLOAD_SIZE 768 //Worst case: UBX_CFG_VALSET packet with 64 keyIDs each with 64 bit values
#endif

// For storing SPI bytes received during sendSpiCommand
#define SFE_UBLOX_SPI_BUFFER_SIZE 128

// Default maximum NMEA byte count
// maxNMEAByteCount was set to 82: https://en.wikipedia.org/wiki/NMEA_0183#Message_structure
// but the u-blox HP (RTK) GGA messages are 88 bytes long
// The user can adjust maxNMEAByteCount by calling setMaxNMEAByteCount
#define SFE_UBLOX_MAX_NMEA_BYTE_COUNT 88

//-=-=-=-=- UBX binary specific variables
struct ubxPacket
{
  uint8_t cls;
  uint8_t id;
  uint16_t len;          // Length of the payload. Does not include cls, id, or checksum bytes
  uint16_t counter;      // Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
  uint16_t startingSpot; // The counter value needed to go past before we begin recording into payload array
  uint8_t *payload;      // We will allocate RAM for the payload if/when needed.
  uint8_t checksumA;     // Given to us from module. Checked against the rolling calculated A/B checksums.
  uint8_t checksumB;
  sfe_ublox_packet_validity_e valid;           // Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
  sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};

// Struct to hold the results returned by getGeofenceState (returned by UBX-NAV-GEOFENCE)
typedef struct
{
  uint8_t status;    // Geofencing status: 0 - Geofencing not available or not reliable; 1 - Geofencing active
  uint8_t numFences; // Number of geofences
  uint8_t combState; // Combined (logical OR) state of all geofences: 0 - Unknown; 1 - Inside; 2 - Outside
  uint8_t states[4]; // Geofence states: 0 - Unknown; 1 - Inside; 2 - Outside
} geofenceState;

// Struct to hold the current geofence parameters
typedef struct
{
  uint8_t numFences; // Number of active geofences
  int32_t lats[4];   // Latitudes of geofences (in degrees * 10^-7)
  int32_t longs[4];  // Longitudes of geofences (in degrees * 10^-7)
  uint32_t rads[4];  // Radii of geofences (in m * 10^-2)
} geofenceParams_t;

// Struct to hold the module software version
typedef struct
{
  uint8_t versionLow; // Loaded from getProtocolVersion().
  uint8_t versionHigh;
  bool moduleQueried;
} moduleSWVersion_t;

const uint32_t SFE_UBLOX_DAYS_FROM_1970_TO_2020 = 18262; // Jan 1st 2020 Epoch = 1577836800 seconds
const uint16_t SFE_UBLOX_DAYS_SINCE_2020[80] =
    {
        0, 366, 731, 1096, 1461, 1827, 2192, 2557, 2922, 3288,
        3653, 4018, 4383, 4749, 5114, 5479, 5844, 6210, 6575, 6940,
        7305, 7671, 8036, 8401, 8766, 9132, 9497, 9862, 10227, 10593,
        10958, 11323, 11688, 12054, 12419, 12784, 13149, 13515, 13880, 14245,
        14610, 14976, 15341, 15706, 16071, 16437, 16802, 17167, 17532, 17898,
        18263, 18628, 18993, 19359, 19724, 20089, 20454, 20820, 21185, 21550,
        21915, 22281, 22646, 23011, 23376, 23742, 24107, 24472, 24837, 25203,
        25568, 25933, 26298, 26664, 27029, 27394, 27759, 28125, 28490, 28855};
const uint16_t SFE_UBLOX_DAYS_SINCE_MONTH[2][12] =
    {
        {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}, // Leap Year (Year % 4 == 0)
        {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334}  // Normal Year
};

class SFE_UBLOX_GNSS
{
public:
  SFE_UBLOX_GNSS(void);
  virtual ~SFE_UBLOX_GNSS(void);

  // Depending on the sentence type the processor will load characters into different arrays
  enum sfe_ublox_sentence_types_e
  {
    SFE_UBLOX_SENTENCE_TYPE_NONE = 0,
    SFE_UBLOX_SENTENCE_TYPE_NMEA,
    SFE_UBLOX_SENTENCE_TYPE_UBX,
    SFE_UBLOX_SENTENCE_TYPE_RTCM
  } currentSentence = SFE_UBLOX_SENTENCE_TYPE_NONE;

// A default of 250ms for maxWait seems fine for I2C but is not enough for SerialUSB.
// If you know you are only going to be using I2C / Qwiic communication, you can
// safely reduce defaultMaxWait to 250.
#ifndef defaultMaxWait // Let's allow the user to define their own value if they want to
#define defaultMaxWait 1100
#endif

  // New in v2.0: allow the payload size for packetCfg to be changed
  bool setPacketCfgPayloadSize(size_t payloadSize); // Set packetCfgPayloadSize
  size_t getPacketCfgSpaceRemaining();              // Returns the number of free bytes remaining in packetCfgPayload

  // Begin communication with the GNSS. Advanced users can assume success if required. Useful if the port is already outputting messages at high navigation rate.
  // Begin will then return true if "signs of life" have been seen: reception of _any_ valid UBX packet or _any_ valid NMEA header.
  // By default use the default I2C address, and use Wire port
  bool begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = 0x42, uint16_t maxWait = defaultMaxWait, bool assumeSuccess = false); // Returns true if module is detected
  // serialPort needs to be perviously initialized to correct baud rate
  bool begin(Stream &serialPort, uint16_t maxWait = defaultMaxWait, bool assumeSuccess = false); // Returns true if module is detected
  // SPI - supply instance of SPIClass, chip select pin and SPI speed (in Hz)
  bool begin(SPIClass &spiPort, uint8_t csPin, uint32_t spiSpeed, uint16_t maxWait = defaultMaxWait, bool assumeSuccess = false);

  void end(void); // Stop all automatic message processing. Free all used RAM

  void setI2CpollingWait(uint8_t newPollingWait_ms); // Allow the user to change the I2C polling wait if required
  void setSPIpollingWait(uint8_t newPollingWait_ms); // Allow the user to change the SPI polling wait if required

  // Set the max number of bytes set in a given I2C transaction
  uint8_t i2cTransactionSize = 32; // Default to ATmega328 limit

  // Control the size of the internal I2C transaction amount
  void setI2CTransactionSize(uint8_t bufferSize);
  uint8_t getI2CTransactionSize(void);

  // Support for platforms like ESP32 which do not support multiple I2C restarts
  // If _i2cStopRestart is true, endTransmission will always use a stop. If false, a restart will be used where needed.
  // The default value for _i2cStopRestart is set in the class instantiation code.
  void setI2cStopRestart(bool stop) { _i2cStopRestart = stop; };
  bool getI2cStopRestart(void) { return (_i2cStopRestart); };

  // Control the size of the spi buffer. If the buffer isn't big enough, we'll start to lose bytes
  // That we receive if the buffer is full!
  void setSpiTransactionSize(uint8_t bufferSize);
  uint8_t getSpiTransactionSize(void);

  // Control the size of maxNMEAByteCount
  void setMaxNMEAByteCount(int8_t newMax);
  int8_t getMaxNMEAByteCount(void);

  // Returns true if device answers on _gpsI2Caddress address or via Serial
  bool isConnected(uint16_t maxWait = defaultMaxWait);

// Enable debug messages using the chosen Serial port (Stream)
// Boards like the RedBoard Turbo use SerialUSB (not Serial).
// But other boards like the SAMD51 Thing Plus use Serial (not SerialUSB).
// These lines let the code compile cleanly on as many SAMD boards as possible.
#if defined(ARDUINO_ARCH_SAMD)                                                         // Is this a SAMD board?
#if defined(USB_VID)                                                                   // Is the USB Vendor ID defined?
#if (USB_VID == 0x1B4F)                                                                // Is this a SparkFun board?
#if !defined(ARDUINO_SAMD51_THING_PLUS) & !defined(ARDUINO_SAMD51_MICROMOD)            // If it is not a SAMD51 Thing Plus or SAMD51 MicroMod
  void enableDebugging(Stream &debugPort = SerialUSB, bool printLimitedDebug = false); // Given a port to print to, enable debug messages. Default to all, not limited.
#else
  void enableDebugging(Stream &debugPort = Serial, bool printLimitedDebug = false); // Given a port to print to, enable debug messages. Default to all, not limited.
#endif
#else
  void enableDebugging(Stream &debugPort = Serial, bool printLimitedDebug = false); // Given a port to print to, enable debug messages. Default to all, not limited.
#endif
#else
  void enableDebugging(Stream &debugPort = Serial, bool printLimitedDebug = false); // Given a port to print to, enable debug messages. Default to all, not limited.
#endif
#else
  void enableDebugging(Stream &debugPort = Serial, bool printLimitedDebug = false); // Given a port to print to, enable debug messages. Default to all, not limited.
#endif

  void disableDebugging(void);                       // Turn off debug statements
  void debugPrint(char *message);                    // Safely print debug statements
  void debugPrintln(char *message);                  // Safely print debug statements
  const char *statusString(sfe_ublox_status_e stat); // Pretty print the return value

  // Check for the arrival of new I2C/Serial data

  void disableUBX7Fcheck(bool disabled = true); // When logging RAWX data, we need to be able to disable the "7F" check in checkUbloxI2C

  // Changed in V1.8.1: provides backward compatibility for the examples that call checkUblox directly
  // Will default to using packetCfg to look for explicit autoPVT packets so they get processed correctly by processUBX
  bool checkUblox(uint8_t requestedClass = 0, uint8_t requestedID = 0); // Checks module with user selected commType

  bool checkUbloxI2C(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);    // Method for I2C polling of data, passing any new bytes to process()
  bool checkUbloxSerial(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID); // Method for serial polling of data, passing any new bytes to process()
  bool checkUbloxSpi(ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);    // Method for spi polling of data, passing any new bytes to process()

  // Process the incoming data

  void process(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);             // Processes NMEA and UBX binary sentences one byte at a time
  void processNMEA(char incoming) __attribute__((weak));                                                           // Given a NMEA character, do something with it. User can overwrite if desired to use something like tinyGPS or MicroNMEA libraries
  virtual void processNMEA_v(char incoming);                                                                       // Given a NMEA character, do something with it. User can overwrite if desired to use something like tinyGPS or MicroNMEA libraries
  sfe_ublox_sentence_types_e processRTCMframe(uint8_t incoming, uint16_t *rtcmFrameCounter) __attribute__((weak)); // Monitor the incoming bytes for start and length bytes
  virtual sfe_ublox_sentence_types_e processRTCMframe_v(uint8_t incoming, uint16_t *rtcmFrameCounter);             // Monitor the incoming bytes for start and length bytes
  void processRTCM(uint8_t incoming) __attribute__((weak));                                                        // Given rtcm byte, do something with it. User can overwrite if desired to pipe bytes to radio, internet, etc.
  virtual void processRTCM_v(uint8_t incoming);                                                                    // Given rtcm byte, do something with it. User can overwrite if desired to pipe bytes to radio, internet, etc.
  void processUBX(uint8_t incoming, ubxPacket *incomingUBX, uint8_t requestedClass, uint8_t requestedID);          // Given a character, file it away into the uxb packet structure
  void processUBXpacket(ubxPacket *msg);                                                                           // Once a packet has been received and validated, identify this packet's class/id and update internal flags

  // Send I2C/Serial/SPI commands to the module

  void calcChecksum(ubxPacket *msg);                                                                                     // Sets the checksumA and checksumB of a given messages
  sfe_ublox_status_e sendCommand(ubxPacket *outgoingUBX, uint16_t maxWait = defaultMaxWait, bool expectACKonly = false); // Given a packet and payload, send everything including CRC bytes, return true if we got a response
  sfe_ublox_status_e sendI2cCommand(ubxPacket *outgoingUBX, uint16_t maxWait = defaultMaxWait);
  void sendSerialCommand(ubxPacket *outgoingUBX);
  void sendSpiCommand(ubxPacket *outgoingUBX);

  void printPacket(ubxPacket *packet, bool alwaysPrintPayload = false); // Useful for debugging

  // After sending a message to the module, wait for the expected response (data+ACK or just data)

  sfe_ublox_status_e waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = defaultMaxWait);   // Poll the module until a config packet and an ACK is received, or just an ACK
  sfe_ublox_status_e waitForNoACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime = defaultMaxWait); // Poll the module until a config packet is received

  // Check if any callbacks need to be called
  void checkCallbacks(void);

  // Push (e.g.) RTCM data directly to the module
  // Warning: this function does not check that the data is valid. It is the user's responsibility to ensure the data is valid before pushing.
  // Default to using a restart between transmissions. But processors like ESP32 seem to need a stop (#30). Set stop to true to use a stop instead.
  bool pushRawData(uint8_t *dataBytes, size_t numDataBytes, bool stop = false);

// Push MGA AssistNow data to the module.
// Check for UBX-MGA-ACK responses if required (if mgaAck is YES or ENQUIRE).
// Wait for maxWait millis after sending each packet (if mgaAck is NO).
// Return how many bytes were pushed successfully.
// If skipTime is true, any UBX-MGA-INI-TIME_UTC or UBX-MGA-INI-TIME_GNSS packets found in the data will be skipped,
// allowing the user to override with their own time data with setUTCTimeAssistance.
// offset allows a sub-set of the data to be sent - starting from offset.
#define defaultMGAdelay 7 // Default to waiting for 7ms between each MGA message
  size_t pushAssistNowData(const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  size_t pushAssistNowData(const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  size_t pushAssistNowData(bool skipTime, const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  size_t pushAssistNowData(bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  size_t pushAssistNowData(size_t offset, bool skipTime, const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  size_t pushAssistNowData(size_t offset, bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);

// Provide initial time assistance
#define defaultMGAINITIMEtAccS 2  // Default to setting the seconds time accuracy to 2 seconds
#define defaultMGAINITIMEtAccNs 0 // Default to setting the nanoseconds time accuracy to zero
#define defaultMGAINITIMEsource 0 // Set default source to none, i.e. on receipt of message (will be inaccurate!)
  bool setUTCTimeAssistance(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint32_t nanos = 0,
                            uint16_t tAccS = defaultMGAINITIMEtAccS, uint32_t tAccNs = defaultMGAINITIMEtAccNs, uint8_t source = defaultMGAINITIMEsource,
                            sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);

  // Provide initial position assistance
  // The units for ecefX/Y/Z and posAcc (stddev) are cm.
  bool setPositionAssistanceXYZ(int32_t ecefX, int32_t ecefY, int32_t ecefZ, uint32_t posAcc, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);
  // The units for lat and lon are degrees * 1e-7 (WGS84)
  // The units for alt (WGS84) and posAcc (stddev) are cm.
  bool setPositionAssistanceLLH(int32_t lat, int32_t lon, int32_t alt, uint32_t posAcc, sfe_ublox_mga_assist_ack_e mgaAck = SFE_UBLOX_MGA_ASSIST_ACK_NO, uint16_t maxWait = defaultMGAdelay);

  // Find the start of the AssistNow Offline (UBX_MGA_ANO) data for the chosen day
  // The daysIntoFture parameter makes it easy to get the data for (e.g.) tomorrow based on today's date
  // Returns numDataBytes if unsuccessful
  // TO DO: enhance this so it will find the nearest data for the chosen day - instead of an exact match
  size_t findMGAANOForDate(const String &dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture = 0);
  size_t findMGAANOForDate(const uint8_t *dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture = 0);

// Read the whole navigation data base. The receiver will send all available data from its internal database.
// Data is written to dataBytes. Set maxNumDataBytes to the (maximum) size of dataBytes.
// If the database exceeds maxNumDataBytes, the excess bytes will be lost.
// The function returns the number of database bytes written to dataBytes.
// The return value will be equal to maxNumDataBytes if excess data was received.
// The function will timeout after maxWait milliseconds - in case the final UBX-MGA-ACK was missed.
#define defaultNavDBDMaxWait 3100
  size_t readNavigationDatabase(uint8_t *dataBytes, size_t maxNumDataBytes, uint16_t maxWait = defaultNavDBDMaxWait);

  // Support for data logging
  void setFileBufferSize(uint16_t bufferSize);                             // Set the size of the file buffer. This must be called _before_ .begin.
  uint16_t getFileBufferSize(void);                                        // Return the size of the file buffer
  uint16_t extractFileBufferData(uint8_t *destination, uint16_t numBytes); // Extract numBytes of data from the file buffer. Copy it to destination. It is the user's responsibility to ensure destination is large enough.
  uint16_t fileBufferAvailable(void);                                      // Returns the number of bytes available in file buffer which are waiting to be read
  uint16_t getMaxFileBufferAvail(void);                                    // Returns the maximum number of bytes which the file buffer has contained. Handy for checking the buffer is large enough to handle all the incoming data.
  void clearFileBuffer(void);                                              // Empty the file buffer - discard all contents
  void clearMaxFileBufferAvail(void);                                      // Reset fileBufferMaxAvail

  // Specific commands

  // Port configurations
  bool getPortSettings(uint8_t portID, uint16_t maxWait = defaultMaxWait);                    // Returns the current protocol bits in the UBX-CFG-PRT command for a given port
  bool setPortOutput(uint8_t portID, uint8_t comSettings, uint16_t maxWait = defaultMaxWait); // Configure a given port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setPortInput(uint8_t portID, uint8_t comSettings, uint16_t maxWait = defaultMaxWait);  // Configure a given port to input UBX, NMEA, RTCM3, SPARTN or a combination thereof

  bool setI2CAddress(uint8_t deviceAddress, uint16_t maxTime = defaultMaxWait);                                // Changes the I2C address of the u-blox module
  void setSerialRate(uint32_t baudrate, uint8_t uartPort = COM_PORT_UART1, uint16_t maxTime = defaultMaxWait); // Changes the serial baud rate of the u-blox module, uartPort should be COM_PORT_UART1/2

  bool setI2COutput(uint8_t comSettings, uint16_t maxWait = defaultMaxWait);   // Configure I2C port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setUART1Output(uint8_t comSettings, uint16_t maxWait = defaultMaxWait); // Configure UART1 port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setUART2Output(uint8_t comSettings, uint16_t maxWait = defaultMaxWait); // Configure UART2 port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setUSBOutput(uint8_t comSettings, uint16_t maxWait = defaultMaxWait);   // Configure USB port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  bool setSPIOutput(uint8_t comSettings, uint16_t maxWait = defaultMaxWait);   // Configure SPI port to output UBX, NMEA, RTCM3, SPARTN or a combination thereof
  void setNMEAOutputPort(Stream &nmeaOutputPort);                              // Sets the internal variable for the port to direct NMEA characters to
  void setOutputPort(Stream &outputPort);                                      // Sets the internal variable for the port to direct ALL characters to

  // Reset to defaults

  void factoryReset();                                    // Send factory reset sequence (i.e. load "default" configuration and perform hardReset)
  void hardReset();                                       // Perform a reset leading to a cold start (zero info start-up)
  void softwareResetGNSSOnly();                           // Controlled Software Reset (GNSS only) only restarts the GNSS tasks, without reinitializing the full system or reloading any stored configuration.
  bool factoryDefault(uint16_t maxWait = defaultMaxWait); // Reset module to factory defaults

  // Save configuration to BBR / Flash

  bool saveConfiguration(uint16_t maxWait = defaultMaxWait);                        // Save current configuration to flash and BBR (battery backed RAM)
  bool saveConfigSelective(uint32_t configMask, uint16_t maxWait = defaultMaxWait); // Save the selected configuration sub-sections to flash and BBR (battery backed RAM)

  // Functions to turn on/off message types for a given port ID (see COM_PORT_I2C, etc above)
  bool configureMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t sendRate, uint16_t maxWait = defaultMaxWait);
  bool enableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint8_t sendRate = 1, uint16_t maxWait = defaultMaxWait);
  bool disableMessage(uint8_t msgClass, uint8_t msgID, uint8_t portID, uint16_t maxWait = defaultMaxWait);
  bool enableNMEAMessage(uint8_t msgID, uint8_t portID, uint8_t sendRate = 1, uint16_t maxWait = defaultMaxWait);
  bool disableNMEAMessage(uint8_t msgID, uint8_t portID, uint16_t maxWait = defaultMaxWait);
  bool enableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint8_t sendRate, uint16_t maxWait = defaultMaxWait); // Given a message number turns on a message ID for output over given PortID
  bool disableRTCMmessage(uint8_t messageNumber, uint8_t portID, uint16_t maxWait = defaultMaxWait);                  // Turn off given RTCM message from a given port

  // Functions used for RTK and base station setup
  bool getSurveyMode(uint16_t maxWait = defaultMaxWait);                                                                     // Get the current TimeMode3 settings
  bool getSurveyMode(UBX_CFG_TMODE3_data_t *data = NULL, uint16_t maxWait = defaultMaxWait);                                 // Get the current TimeMode3 settings
  bool setSurveyMode(uint8_t mode, uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = defaultMaxWait);     // Control survey in mode
  bool setSurveyModeFull(uint8_t mode, uint32_t observationTime, float requiredAccuracy, uint16_t maxWait = defaultMaxWait); // Control survey in mode
  bool enableSurveyMode(uint16_t observationTime, float requiredAccuracy, uint16_t maxWait = defaultMaxWait);                // Begin Survey-In for NEO-M8P / ZED-F9x
  bool enableSurveyModeFull(uint32_t observationTime, float requiredAccuracy, uint16_t maxWait = defaultMaxWait);            // Begin Survey-In for NEO-M8P / ZED-F9x
  bool disableSurveyMode(uint16_t maxWait = defaultMaxWait);                                                                 // Stop Survey-In mode
  // Given coordinates, put receiver into static position. Set latlong to true to pass in lat/long values instead of ecef.
  // For ECEF the units are: cm, 0.1mm, cm, 0.1mm, cm, 0.1mm
  // For Lat/Lon/Alt the units are: degrees^-7, degrees^-9, degrees^-7, degrees^-9, cm, 0.1mm
  bool setStaticPosition(int32_t ecefXOrLat, int8_t ecefXOrLatHP, int32_t ecefYOrLon, int8_t ecefYOrLonHP, int32_t ecefZOrAlt, int8_t ecefZOrAltHP, bool latLong = false, uint16_t maxWait = defaultMaxWait);
  bool setStaticPosition(int32_t ecefXOrLat, int32_t ecefYOrLon, int32_t ecefZOrAlt, bool latLong = false, uint16_t maxWait = defaultMaxWait);
  bool setDGNSSConfiguration(sfe_ublox_dgnss_mode_e dgnssMode = SFE_UBLOX_DGNSS_MODE_FIXED, uint16_t maxWait = defaultMaxWait); // Set the DGNSS differential mode

  // Read the module's protocol version
  uint8_t getProtocolVersionHigh(uint16_t maxWait = defaultMaxWait); // Returns the PROTVER XX.00 from UBX-MON-VER register
  uint8_t getProtocolVersionLow(uint16_t maxWait = defaultMaxWait);  // Returns the PROTVER 00.XX from UBX-MON-VER register
  bool getProtocolVersion(uint16_t maxWait = defaultMaxWait);        // Queries module, loads low/high bytes
  moduleSWVersion_t *moduleSWVersion = NULL;                         // Pointer to struct. RAM will be allocated for this if/when necessary

  // Support for geofences
  bool addGeofence(int32_t latitude, int32_t longitude, uint32_t radius, byte confidence = 0, byte pinPolarity = 0, byte pin = 0, uint16_t maxWait = defaultMaxWait); // Add a new geofence
  bool clearGeofences(uint16_t maxWait = defaultMaxWait);                                                                                                             // Clears all geofences
  bool clearAntPIO(uint16_t maxWait = defaultMaxWait);                                                                                                                // Clears the antenna control pin settings to release the PIOs
  bool getGeofenceState(geofenceState &currentGeofenceState, uint16_t maxWait = defaultMaxWait);                                                                      // Returns the combined geofence state
  // Storage for the geofence parameters. RAM is allocated for this if/when required.
  geofenceParams_t *currentGeofenceParams = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

  // Power save / off
  bool powerSaveMode(bool power_save = true, uint16_t maxWait = defaultMaxWait);
  uint8_t getPowerSaveMode(uint16_t maxWait = defaultMaxWait); // Returns 255 if the sendCommand fails
  bool powerOff(uint32_t durationInMs, uint16_t maxWait = defaultMaxWait);
  bool powerOffWithInterrupt(uint32_t durationInMs, uint32_t wakeupSources = VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0, bool forceWhileUsb = true, uint16_t maxWait = defaultMaxWait);
  // Power Mode Setup. Values period and onTime are only valid if mode is SFE_UBLOX_PMS_MODE_INTERVAL
  bool setPowerManagement(sfe_ublox_pms_mode_e mode, uint16_t period=0, uint16_t onTime=0, uint16_t maxWait = defaultMaxWait);
  bool setupPowerMode(sfe_ublox_rxm_mode_e mode, uint16_t maxWait = defaultMaxWait);

  // Change the dynamic platform model using UBX-CFG-NAV5
  bool setDynamicModel(dynModel newDynamicModel = DYN_MODEL_PORTABLE, uint16_t maxWait = defaultMaxWait);
  uint8_t getDynamicModel(uint16_t maxWait = defaultMaxWait); // Get the dynamic model - returns 255 if the sendCommand fails

  // Reset / enable / configure the odometer
  bool resetOdometer(uint16_t maxWait = defaultMaxWait); // Reset the odometer
  bool enableOdometer(bool enable = true, uint16_t maxWait = defaultMaxWait); // Enable / disable the odometer
  bool getOdometerConfig(uint8_t *flags, uint8_t *odoCfg, uint8_t *cogMaxSpeed, uint8_t *cogMaxPosAcc, uint8_t *velLpGain, uint8_t *cogLpGain, uint16_t maxWait = defaultMaxWait); // Read the odometer configuration
  bool setOdometerConfig(uint8_t flags, uint8_t odoCfg, uint8_t cogMaxSpeed, uint8_t cogMaxPosAcc, uint8_t velLpGain, uint8_t cogLpGain, uint16_t maxWait = defaultMaxWait); // Configure the odometer

  // Enable/Disable individual GNSS systems using UBX-CFG-GNSS
  // Note: you must leave at least one major GNSS enabled! If in doubt, enable GPS before disabling the others
  // TO DO: Add support for sigCfgMask and maxTrkCh. (Need to resolve ambiguity with maxWait)
  bool enableGNSS(bool enable, sfe_ublox_gnss_ids_e id, uint16_t maxWait = defaultMaxWait);
  bool isGNSSenabled(sfe_ublox_gnss_ids_e id, uint16_t maxWait = defaultMaxWait);

  // Reset ESF automatic IMU-mount alignment
  bool resetIMUalignment(uint16_t maxWait = defaultMaxWait);

  // Enable/disable esfAutoAlignment
  bool getESFAutoAlignment(uint16_t maxWait = defaultMaxWait);
  bool setESFAutoAlignment(bool enable, uint16_t maxWait = defaultMaxWait);

  // Configure Time Pulse Parameters
  bool getTimePulseParameters(UBX_CFG_TP5_data_t *data = NULL, uint16_t maxWait = defaultMaxWait); // Get the time pulse parameters using UBX_CFG_TP5
  bool setTimePulseParameters(UBX_CFG_TP5_data_t *data = NULL, uint16_t maxWait = defaultMaxWait); // Set the time pulse parameters using UBX_CFG_TP5

  // Jamming/interference monitor configuration
  bool getJammingConfiguration(UBX_CFG_ITFM_data_t *data = NULL, uint16_t maxWait = defaultMaxWait); // Get the jamming/interference monitor configuration using UBX_CFG_ITFM
  bool setJammingConfiguration(UBX_CFG_ITFM_data_t *data = NULL, uint16_t maxWait = defaultMaxWait); // Set the jamming/interference monitor configuration using UBX_CFG_ITFM

  // RF Information (including jamming) - ZED-F9 only
  bool getRFinformation(UBX_MON_RF_data_t *data = NULL, uint16_t maxWait = defaultMaxWait); // Get the RF information using UBX_MON_RF

  // Hardware status (including jamming)
  bool getHWstatus(UBX_MON_HW_data_t *data = NULL, uint16_t maxWait = defaultMaxWait); // Get the hardware status using UBX_MON_HW

  // Extended hardware status
  bool getHW2status(UBX_MON_HW2_data_t *data = NULL, uint16_t maxWait = defaultMaxWait); // Get the extended hardware status using UBX_MON_HW2

  // UBX-CFG-NAVX5 - get/set the ackAiding byte. If ackAiding is 1, UBX-MGA-ACK messages will be sent by the module to acknowledge the MGA data
  uint8_t getAckAiding(uint16_t maxWait = defaultMaxWait);                 // Get the ackAiding byte - returns 255 if the sendCommand fails
  bool setAckAiding(uint8_t ackAiding, uint16_t maxWait = defaultMaxWait); // Set the ackAiding byte

  // AssistNow Autonomous support
  // UBX-CFG-NAVX5 - get/set the aopCfg byte and set the aopOrdMaxErr word. If aopOrbMaxErr is 0 (default), the max orbit error is reset to the firmware default.
  uint8_t getAopCfg(uint16_t maxWait = defaultMaxWait);                                         // Get the AssistNow Autonomous configuration (aopCfg) - returns 255 if the sendCommand fails
  bool setAopCfg(uint8_t aopCfg, uint16_t aopOrbMaxErr = 0, uint16_t maxWait = defaultMaxWait); // Set the aopCfg byte and the aopOrdMaxErr word

  // SPARTN dynamic keys
  //"When the receiver boots, the host should send 'current' and 'next' keys in one message." - Use setDynamicSPARTNKeys for this.
  //"Every time the 'current' key is expired, 'next' takes its place."
  //"Therefore the host should then retrieve the new 'next' key and send only that." - Use setDynamicSPARTNKey for this.
  // The key can be provided in binary (uint8_t) format or in ASCII Hex (char) format, but in both cases keyLengthBytes _must_ represent the binary key length in bytes.
  bool setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const char *key);
  bool setDynamicSPARTNKey(uint8_t keyLengthBytes, uint16_t validFromWno, uint32_t validFromTow, const uint8_t *key);
  bool setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const char *key1,
                            uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const char *key2);
  bool setDynamicSPARTNKeys(uint8_t keyLengthBytes1, uint16_t validFromWno1, uint32_t validFromTow1, const uint8_t *key1,
                            uint8_t keyLengthBytes2, uint16_t validFromWno2, uint32_t validFromTow2, const uint8_t *key2);

  // General configuration (used only on protocol v27 and higher - ie, ZED-F9P)

  uint32_t createKey(uint16_t group, uint16_t id, uint8_t size);                                                                  // Form 32-bit key from group/id/size
  sfe_ublox_status_e getVal(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = defaultMaxWait);                    // Load payload with response
  uint8_t getVal8(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = defaultMaxWait);                              // Returns the value at a given key location
  uint16_t getVal16(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = defaultMaxWait);                            // Returns the value at a given key location
  uint32_t getVal32(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = defaultMaxWait);                            // Returns the value at a given key location
  uint64_t getVal64(uint32_t keyID, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = defaultMaxWait);                            // Returns the value at a given key location
  uint8_t getVal8(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = defaultMaxWait);   // Returns the value at a given group/id/size location
  uint16_t getVal16(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = defaultMaxWait); // Returns the value at a given group/id/size location
  uint32_t getVal32(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = defaultMaxWait); // Returns the value at a given group/id/size location
  uint64_t getVal64(uint16_t group, uint16_t id, uint8_t size, uint8_t layer = VAL_LAYER_RAM, uint16_t maxWait = defaultMaxWait); // Returns the value at a given group/id/size location
  uint8_t setVal(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = defaultMaxWait);               // Sets the 16-bit value at a given group/id/size location
  uint8_t setVal8(uint32_t keyID, uint8_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = defaultMaxWait);               // Sets the 8-bit value at a given group/id/size location
  uint8_t setVal16(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = defaultMaxWait);             // Sets the 16-bit value at a given group/id/size location
  uint8_t setVal32(uint32_t keyID, uint32_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = defaultMaxWait);             // Sets the 32-bit value at a given group/id/size location
  uint8_t setVal64(uint32_t keyID, uint64_t value, uint8_t layer = VAL_LAYER_ALL, uint16_t maxWait = defaultMaxWait);             // Sets the 64-bit value at a given group/id/size location
  uint8_t newCfgValset(uint8_t layer = VAL_LAYER_ALL);                                                                            // Create a new, empty UBX-CFG-VALSET. Add entries with addCfgValset8/16/32/64
  uint8_t newCfgValset8(uint32_t keyID, uint8_t value, uint8_t layer = VAL_LAYER_ALL);                                            // Define a new UBX-CFG-VALSET with the given KeyID and 8-bit value
  uint8_t newCfgValset16(uint32_t keyID, uint16_t value, uint8_t layer = VAL_LAYER_ALL);                                          // Define a new UBX-CFG-VALSET with the given KeyID and 16-bit value
  uint8_t newCfgValset32(uint32_t keyID, uint32_t value, uint8_t layer = VAL_LAYER_ALL);                                          // Define a new UBX-CFG-VALSET with the given KeyID and 32-bit value
  uint8_t newCfgValset64(uint32_t keyID, uint64_t value, uint8_t layer = VAL_LAYER_ALL);                                          // Define a new UBX-CFG-VALSET with the given KeyID and 64-bit value
  uint8_t addCfgValset8(uint32_t keyID, uint8_t value);                                                                           // Add a new KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket
  uint8_t addCfgValset16(uint32_t keyID, uint16_t value);                                                                         // Add a new KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket
  uint8_t addCfgValset32(uint32_t keyID, uint32_t value);                                                                         // Add a new KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket
  uint8_t addCfgValset64(uint32_t keyID, uint64_t value);                                                                         // Add a new KeyID and 64-bit value to an existing UBX-CFG-VALSET ubxPacket
  uint8_t sendCfgValset8(uint32_t keyID, uint8_t value, uint16_t maxWait = defaultMaxWait);                                       // Add the final KeyID and 8-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
  uint8_t sendCfgValset16(uint32_t keyID, uint16_t value, uint16_t maxWait = defaultMaxWait);                                     // Add the final KeyID and 16-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
  uint8_t sendCfgValset32(uint32_t keyID, uint32_t value, uint16_t maxWait = defaultMaxWait);                                     // Add the final KeyID and 32-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
  uint8_t sendCfgValset64(uint32_t keyID, uint64_t value, uint16_t maxWait = defaultMaxWait);                                     // Add the final KeyID and 64-bit value to an existing UBX-CFG-VALSET ubxPacket and send it
  uint8_t sendCfgValset(uint16_t maxWait = defaultMaxWait);                                                                       // Send the CfgValset (UBX-CFG-VALSET) construct
  uint8_t getCfgValsetLen();                                                                                                      // Returns the length of the current CfgValset construct as number-of-keyIDs
  size_t getCfgValsetSpaceRemaining();                                                                                            // Returns the number of free bytes remaining in packetCfg
  void autoSendCfgValsetAtSpaceRemaining(size_t spaceRemaining) { _autoSendAtSpaceRemaining = spaceRemaining; }                   // Cause CFG_VALSET packets to be sent automatically when packetCfg has less than this many bytes available

  // get and set functions for all of the "automatic" message processing

  // Navigation (NAV)

  // getPVT will only return data once in each navigation cycle. By default, that is once per second.
  // Therefore we should set defaultMaxWait to slightly longer than that.
  // If you change the navigation frequency to (e.g.) 4Hz using setNavigationFrequency(4)
  // then you should use a shorter maxWait. 300msec would be about right: getPVT(300)

  bool getNAVPOSECEF(uint16_t maxWait = defaultMaxWait);                                                                      // NAV POSECEF
  bool setAutoNAVPOSECEF(bool enabled, uint16_t maxWait = defaultMaxWait);                                                    // Enable/disable automatic POSECEF reports at the navigation frequency
  bool setAutoNAVPOSECEF(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                               // Enable/disable automatic POSECEF reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVPOSECEFrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                    // Set the rate for automatic POSECEF reports
  bool setAutoNAVPOSECEFcallback(void (*callbackPointer)(UBX_NAV_POSECEF_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic POSECEF reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoNAVPOSECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_POSECEF_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic POSECEF reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVPOSECEF(bool enabled, bool implicitUpdate = true);                                                        // In case no config access to the GPS is possible and POSECEF is send cyclically already
  void flushNAVPOSECEF();                                                                                                     // Mark all the data as read/stale
  void logNAVPOSECEF(bool enabled = true);                                                                                    // Log data to file buffer

  bool getNAVSTATUS(uint16_t maxWait = defaultMaxWait);                                                                     // NAV STATUS
  bool setAutoNAVSTATUS(bool enabled, uint16_t maxWait = defaultMaxWait);                                                   // Enable/disable automatic STATUS reports at the navigation frequency
  bool setAutoNAVSTATUS(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                              // Enable/disable automatic STATUS reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVSTATUSrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                   // Set the rate for automatic STATUS reports
  bool setAutoNAVSTATUScallback(void (*callbackPointer)(UBX_NAV_STATUS_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic STATUS reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoNAVSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_NAV_STATUS_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic STATUS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVSTATUS(bool enabled, bool implicitUpdate = true);                                                       // In case no config access to the GPS is possible and STATUS is send cyclically already
  void flushNAVSTATUS();                                                                                                    // Mark all the data as read/stale
  void logNAVSTATUS(bool enabled = true);                                                                                   // Log data to file buffer

  bool getDOP(uint16_t maxWait = defaultMaxWait);                                                                  // Query module for latest dilution of precision values and load global vars:. If autoDOP is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new DOP is available.
  bool setAutoDOP(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic DOP reports at the navigation frequency
  bool setAutoDOP(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic DOP reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoDOPrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic DOP reports
  bool setAutoDOPcallback(void (*callbackPointer)(UBX_NAV_DOP_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic DOP reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoDOPcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_DOP_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic DOP reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoDOP(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and DOP is send cyclically already
  void flushDOP();                                                                                                 // Mark all the DOP data as read/stale
  void logNAVDOP(bool enabled = true);                                                                             // Log data to file buffer

  bool getVehAtt(uint16_t maxWait = defaultMaxWait);                                                                  // NAV ATT Helper
  bool getNAVATT(uint16_t maxWait = defaultMaxWait);                                                                  // NAV ATT
  bool setAutoNAVATT(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic vehicle attitude reports at the navigation frequency
  bool setAutoNAVATT(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic vehicle attitude reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVATTrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic ATT reports
  bool setAutoNAVATTcallback(void (*callbackPointer)(UBX_NAV_ATT_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic ATT reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoNAVATTcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_ATT_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic ATT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVATT(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and vehicle attitude is send cyclically already
  void flushNAVATT();                                                                                                 // Mark all the data as read/stale
  void logNAVATT(bool enabled = true);                                                                                // Log data to file buffer

  bool getPVT(uint16_t maxWait = defaultMaxWait);                                                                  // Query module for latest group of datums and load global vars: lat, long, alt, speed, SIV, accuracies, etc. If autoPVT is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new PVT is available.
  bool setAutoPVT(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic PVT reports at the navigation frequency
  bool setAutoPVT(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic PVT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoPVTrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic PVT reports
  bool setAutoPVTcallback(void (*callbackPointer)(UBX_NAV_PVT_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic PVT reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoPVTcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_PVT_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic PVT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoPVT(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and PVT is send cyclically already
  void flushPVT();                                                                                                 // Mark all the PVT data as read/stale
  void logNAVPVT(bool enabled = true);                                                                             // Log data to file buffer

  bool getNAVODO(uint16_t maxWait = defaultMaxWait);                                                                  // NAV ODO
  bool setAutoNAVODO(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic ODO reports at the navigation frequency
  bool setAutoNAVODO(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic ODO reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVODOrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic ODO reports
  bool setAutoNAVODOcallback(void (*callbackPointer)(UBX_NAV_ODO_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic ODO reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoNAVODOcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_ODO_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic ODO reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVODO(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and ODO is send cyclically already
  void flushNAVODO();                                                                                                 // Mark all the data as read/stale
  void logNAVODO(bool enabled = true);                                                                                // Log data to file buffer

  bool getNAVVELECEF(uint16_t maxWait = defaultMaxWait);                                                                      // NAV VELECEF
  bool setAutoNAVVELECEF(bool enabled, uint16_t maxWait = defaultMaxWait);                                                    // Enable/disable automatic VELECEF reports at the navigation frequency
  bool setAutoNAVVELECEF(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                               // Enable/disable automatic VELECEF reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVVELECEFrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                    // Set the rate for automatic VELECEF reports
  bool setAutoNAVVELECEFcallback(void (*callbackPointer)(UBX_NAV_VELECEF_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic VELECEF reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoNAVVELECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_VELECEF_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic VELECEF reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVVELECEF(bool enabled, bool implicitUpdate = true);                                                        // In case no config access to the GPS is possible and VELECEF is send cyclically already
  void flushNAVVELECEF();                                                                                                     // Mark all the data as read/stale
  void logNAVVELECEF(bool enabled = true);                                                                                    // Log data to file buffer

  bool getNAVVELNED(uint16_t maxWait = defaultMaxWait);                                                                     // NAV VELNED
  bool setAutoNAVVELNED(bool enabled, uint16_t maxWait = defaultMaxWait);                                                   // Enable/disable automatic VELNED reports at the navigation frequency
  bool setAutoNAVVELNED(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                              // Enable/disable automatic VELNED reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVVELNEDrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                   // Set the rate for automatic VELNED reports
  bool setAutoNAVVELNEDcallback(void (*callbackPointer)(UBX_NAV_VELNED_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic VELNED reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoNAVVELNEDcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_VELNED_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic VELNED reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVVELNED(bool enabled, bool implicitUpdate = true);                                                       // In case no config access to the GPS is possible and VELNED is send cyclically already
  void flushNAVVELNED();                                                                                                    // Mark all the data as read/stale
  void logNAVVELNED(bool enabled = true);                                                                                   // Log data to file buffer

  bool getNAVHPPOSECEF(uint16_t maxWait = defaultMaxWait);                                                                        // NAV HPPOSECEF
  bool setAutoNAVHPPOSECEF(bool enabled, uint16_t maxWait = defaultMaxWait);                                                      // Enable/disable automatic HPPOSECEF reports at the navigation frequency
  bool setAutoNAVHPPOSECEF(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                                 // Enable/disable automatic HPPOSECEF reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVHPPOSECEFrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                      // Set the rate for automatic HPPOSECEF reports
  bool setAutoNAVHPPOSECEFcallback(void (*callbackPointer)(UBX_NAV_HPPOSECEF_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic HPPOSECEF reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoNAVHPPOSECEFcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_HPPOSECEF_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic HPPOSECEF reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVHPPOSECEF(bool enabled, bool implicitUpdate = true);                                                          // In case no config access to the GPS is possible and HPPOSECEF is send cyclically already
  void flushNAVHPPOSECEF();                                                                                                       // Mark all the data as read/stale
  void logNAVHPPOSECEF(bool enabled = true);                                                                                      // Log data to file buffer

  bool getHPPOSLLH(uint16_t maxWait = defaultMaxWait);                                                                       // NAV HPPOSLLH
  bool setAutoHPPOSLLH(bool enabled, uint16_t maxWait = defaultMaxWait);                                                     // Enable/disable automatic HPPOSLLH reports at the navigation frequency
  bool setAutoHPPOSLLH(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                                // Enable/disable automatic HPPOSLLH reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoHPPOSLLHrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                     // Set the rate for automatic HPPOSLLH reports
  bool setAutoHPPOSLLHcallback(void (*callbackPointer)(UBX_NAV_HPPOSLLH_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic HPPOSLLH reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoHPPOSLLHcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_HPPOSLLH_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic HPPOSLLH reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoHPPOSLLH(bool enabled, bool implicitUpdate = true);                                                         // In case no config access to the GPS is possible and HPPOSLLH is send cyclically already
  void flushHPPOSLLH();                                                                                                      // Mark all the HPPPOSLLH data as read/stale. This is handy to get data alignment after CRC failure
  void logNAVHPPOSLLH(bool enabled = true);                                                                                  // Log data to file buffer

  bool getNAVPVAT(uint16_t maxWait = defaultMaxWait);                                                                   // NAV PVAT
  bool setAutoNAVPVAT(bool enabled, uint16_t maxWait = defaultMaxWait);                                                 // Enable/disable automatic PVAT reports at the navigation frequency
  bool setAutoNAVPVAT(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                            // Enable/disable automatic PVAT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVPVATrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                 // Set the rate for automatic PVAT reports
  bool setAutoNAVPVATcallback(void (*callbackPointer)(UBX_NAV_PVAT_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic PVAT reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoNAVPVATcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_PVAT_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic PVAT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVPVAT(bool enabled, bool implicitUpdate = true);                                                     // In case no config access to the GPS is possible and PVAT is send cyclically already
  void flushNAVPVAT();                                                                                                  // Mark all the PVAT data as read/stale
  void logNAVPVAT(bool enabled = true);                                                                                 // Log data to file buffer

  bool getNAVTIMEUTC(uint16_t maxWait = defaultMaxWait);                                                                      // NAV TIMEUTC
  bool setAutoNAVTIMEUTC(bool enabled, uint16_t maxWait = defaultMaxWait);                                                    // Enable/disable automatic TIMEUTC reports at the navigation frequency
  bool setAutoNAVTIMEUTC(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                               // Enable/disable automatic TIMEUTC reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVTIMEUTCrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Set the rate for automatic TIMEUTC reports
  bool setAutoNAVTIMEUTCcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_TIMEUTC_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic TIMEUTC reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVTIMEUTC(bool enabled, bool implicitUpdate = true);                                                        // In case no config access to the GPS is possible and TIMEUTC is send cyclically already
  void flushNAVTIMEUTC();                                                                                                     // Mark all the data as read/stale
  void logNAVTIMEUTC(bool enabled = true);                                                                                    // Log data to file buffer

  bool getNAVCLOCK(uint16_t maxWait = defaultMaxWait);                                                                    // NAV CLOCK
  bool setAutoNAVCLOCK(bool enabled, uint16_t maxWait = defaultMaxWait);                                                  // Enable/disable automatic clock reports at the navigation frequency
  bool setAutoNAVCLOCK(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                             // Enable/disable automatic clock reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVCLOCKrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                  // Set the rate for automatic CLOCK reports
  bool setAutoNAVCLOCKcallback(void (*callbackPointer)(UBX_NAV_CLOCK_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic CLOCK reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoNAVCLOCKcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_CLOCK_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic CLOCK reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVCLOCK(bool enabled, bool implicitUpdate = true);                                                      // In case no config access to the GPS is possible and clock is send cyclically already
  void flushNAVCLOCK();                                                                                                   // Mark all the data as read/stale
  void logNAVCLOCK(bool enabled = true);                                                                                  // Log data to file buffer

  bool getSurveyStatus(uint16_t maxWait = 2100);                                                                        // NAV SVIN - Reads survey in status
  bool setAutoNAVSVIN(bool enabled, uint16_t maxWait = defaultMaxWait);                                                 // Enable/disable automatic survey in reports at the navigation frequency
  bool setAutoNAVSVIN(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                            // Enable/disable automatic survey in reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVSVINrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                 // Set the rate for automatic SVIN reports
  bool setAutoNAVSVINcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_SVIN_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic SVIN reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVSVIN(bool enabled, bool implicitUpdate = true);                                                     // In case no config access to the GPS is possible and survey in is send cyclically already
  void flushNAVSVIN();                                                                                                  // Mark all the data as read/stale
  void logNAVSVIN(bool enabled = true);                                                                                 // Log data to file buffer

  bool getNAVEOE(uint16_t maxWait = defaultMaxWait);                                                                  // Query module for latest dilution of precision values and load global vars:. If autoEOE is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new EOE is available.
  bool setAutoNAVEOE(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic EOE reports at the navigation frequency
  bool setAutoNAVEOE(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic EOE reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVEOErate(uint8_t rate, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                       // Set the rate for automatic EOE reports
  bool setAutoNAVEOEcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_EOE_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic EOE reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVEOE(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and EOE is send cyclically already
  void flushNAVEOE();                                                                                                 // Mark all the EOE data as read/stale
  void logNAVEOE(bool enabled = true);                                                                                // Log data to file buffer

  // Add "auto" support for NAV TIMELS - to avoid needing 'global' storage
  bool getLeapSecondEvent(uint16_t maxWait = defaultMaxWait); // Reads leap second event info

  bool getNAVSAT(uint16_t maxWait = defaultMaxWait);                                                                  // Query module for latest AssistNow Autonomous status and load global vars:. If autoNAVSAT is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new NAVSAT is available.
  bool setAutoNAVSAT(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic NAVSAT reports at the navigation frequency
  bool setAutoNAVSAT(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic NAVSAT reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoNAVSATrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic NAVSAT reports
  bool setAutoNAVSATcallback(void (*callbackPointer)(UBX_NAV_SAT_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic NAVSAT reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoNAVSATcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_SAT_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic NAVSAT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoNAVSAT(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and NAVSAT is send cyclically already
  void flushNAVSAT();                                                                                                 // Mark all the NAVSAT data as read/stale
  void logNAVSAT(bool enabled = true);                                                                                // Log data to file buffer

  bool getRELPOSNED(uint16_t maxWait = defaultMaxWait);                                                                        // Get Relative Positioning Information of the NED frame
  bool setAutoRELPOSNED(bool enabled, uint16_t maxWait = defaultMaxWait);                                                      // Enable/disable automatic RELPOSNED reports
  bool setAutoRELPOSNED(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                                 // Enable/disable automatic RELPOSNED, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoRELPOSNEDrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                      // Set the rate for automatic RELPOSNEDreports
  bool setAutoRELPOSNEDcallback(void (*callbackPointer)(UBX_NAV_RELPOSNED_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic RELPOSNED reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoRELPOSNEDcallbackPtr(void (*callbackPointerPtr)(UBX_NAV_RELPOSNED_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic RELPOSNED reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoRELPOSNED(bool enabled, bool implicitUpdate = true);                                                          // In case no config access to the GPS is possible and RELPOSNED is send cyclically already
  void flushNAVRELPOSNED();                                                                                                    // Mark all the data as read/stale
  void logNAVRELPOSNED(bool enabled = true);                                                                                   // Log data to file buffer

  bool getAOPSTATUS(uint16_t maxWait = defaultMaxWait);                                                                        // Query module for latest AssistNow Autonomous status and load global vars:. If autoAOPSTATUS is disabled, performs an explicit poll and waits, if enabled does not block. Returns true if new AOPSTATUS is available.
  bool setAutoAOPSTATUS(bool enabled, uint16_t maxWait = defaultMaxWait);                                                      // Enable/disable automatic AOPSTATUS reports at the navigation frequency
  bool setAutoAOPSTATUS(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                                 // Enable/disable automatic AOPSTATUS reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoAOPSTATUSrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                      // Set the rate for automatic AOPSTATUS reports
  bool setAutoAOPSTATUScallback(void (*callbackPointer)(UBX_NAV_AOPSTATUS_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic AOPSTATUS reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoAOPSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_NAV_AOPSTATUS_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic AOPSTATUS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoAOPSTATUS(bool enabled, bool implicitUpdate = true);                                                          // In case no config access to the GPS is possible and AOPSTATUS is send cyclically already
  void flushAOPSTATUS();                                                                                                       // Mark all the AOPSTATUS data as read/stale
  void logAOPSTATUS(bool enabled = true);                                                                                      // Log data to file buffer

  // Receiver Manager Messages (RXM)

  // Configure a callback for the UBX-RXM-PMP messages produced by the NEO-D9S
  // Note: on the NEO-D9S, the UBX-RXM-PMP messages are enabled by default on all ports.
  //       You can disable them by calling (e.g.) setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C, 0)
  //       The NEO-D9S does not support UBX-CFG-MSG
  bool setRXMPMPcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_PMP_data_t *));                // Callback receives a pointer to the data, instead of _all_ the data. Much kinder on the stack!
  bool setRXMPMPmessageCallbackPtr(void (*callbackPointerPtr)(UBX_RXM_PMP_message_data_t *)); // Use this if you want all of the PMP message (including sync chars, checksum, etc.) to push to a GNSS

  // Configure a callback for the UBX-RXM-QZSSL6 messages produced by the NEO-D9C
  // Note: on the NEO-D9C, the UBX-RXM-QZSSL6 messages are enabled by default on all ports.
  //       You can disable them by calling (e.g.) setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_QZSSL6_I2C, 0)
  //       The NEO-D9C does not support UBX-CFG-MSG
  bool setRXMQZSSL6messageCallbackPtr(void (*callbackPointerPtr)(UBX_RXM_QZSSL6_message_data_t *)); // Use this if you want all of the QZSSL6 message (including sync chars, checksum, etc.) to push to a GNSS

  bool setRXMCORcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_COR_data_t *)); // RXM COR

  bool getRXMSFRBX(uint16_t maxWait = defaultMaxWait);                                                                    // RXM SFRBX
  bool setAutoRXMSFRBX(bool enabled, uint16_t maxWait = defaultMaxWait);                                                  // Enable/disable automatic RXM SFRBX reports at the navigation frequency
  bool setAutoRXMSFRBX(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                             // Enable/disable automatic RXM SFRBX reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoRXMSFRBXrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                  // Set the rate for automatic SFRBX reports
  bool setAutoRXMSFRBXcallback(void (*callbackPointer)(UBX_RXM_SFRBX_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic SFRBX reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoRXMSFRBXcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_SFRBX_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic SFRBX reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoRXMSFRBX(bool enabled, bool implicitUpdate = true);                                                      // In case no config access to the GPS is possible and RXM SFRBX is send cyclically already
  void flushRXMSFRBX();                                                                                                   // Mark all the data as read/stale
  void logRXMSFRBX(bool enabled = true);                                                                                  // Log data to file buffer

  bool getRXMRAWX(uint16_t maxWait = defaultMaxWait);                                                                   // RXM RAWX
  bool setAutoRXMRAWX(bool enabled, uint16_t maxWait = defaultMaxWait);                                                 // Enable/disable automatic RXM RAWX reports at the navigation frequency
  bool setAutoRXMRAWX(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                            // Enable/disable automatic RXM RAWX reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoRXMRAWXrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                 // Set the rate for automatic RAWX reports
  bool setAutoRXMRAWXcallback(void (*callbackPointer)(UBX_RXM_RAWX_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic RAWX reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoRXMRAWXcallbackPtr(void (*callbackPointerPtr)(UBX_RXM_RAWX_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic RAWX reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoRXMRAWX(bool enabled, bool implicitUpdate = true);                                                     // In case no config access to the GPS is possible and RXM RAWX is send cyclically already
  void flushRXMRAWX();                                                                                                  // Mark all the data as read/stale
  void logRXMRAWX(bool enabled = true);                                                                                 // Log data to file buffer

  // Configuration (CFG)

  // Add "auto" support for CFG PRT - because we use it for isConnected (to stop it being mugged by other messages)
  bool getPortSettingsInternal(uint8_t portID, uint16_t maxWait = defaultMaxWait); // Read the port configuration for a given port using UBX-CFG-PRT
  bool getNavigationFrequencyInternal(uint16_t maxWait = defaultMaxWait);          // Get the number of nav solutions sent per second currently being output by module

  // Timing messages (TIM)

  bool getTIMTM2(uint16_t maxWait = defaultMaxWait);                                                                  // TIM TM2
  bool setAutoTIMTM2(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic TIM TM2 reports at the navigation frequency
  bool setAutoTIMTM2(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic TIM TM2 reports at the navigation frequency, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoTIMTM2rate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic TIM TM2 reports
  bool setAutoTIMTM2callback(void (*callbackPointer)(UBX_TIM_TM2_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic TM2 reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoTIMTM2callbackPtr(void (*callbackPointerPtr)(UBX_TIM_TM2_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic TM2 reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoTIMTM2(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and TIM TM2 is send cyclically already
  void flushTIMTM2();                                                                                                 // Mark all the data as read/stale
  void logTIMTM2(bool enabled = true);                                                                                // Log data to file buffer

  // Sensor fusion (dead reckoning) (ESF)

  bool getEsfAlignment(uint16_t maxWait = defaultMaxWait);                                                            // ESF ALG Helper
  bool getESFALG(uint16_t maxWait = defaultMaxWait);                                                                  // ESF ALG
  bool setAutoESFALG(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic ESF ALG reports
  bool setAutoESFALG(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic ESF ALG reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoESFALGrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic ALG reports
  bool setAutoESFALGcallback(void (*callbackPointer)(UBX_ESF_ALG_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic ALG reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoESFALGcallbackPtr(void (*callbackPointerPtr)(UBX_ESF_ALG_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic ALG reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoESFALG(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and ESF ALG is send cyclically already
  void flushESFALG();                                                                                                 // Mark all the data as read/stale
  void logESFALG(bool enabled = true);                                                                                // Log data to file buffer

  bool getEsfInfo(uint16_t maxWait = defaultMaxWait);                                                                       // ESF STATUS Helper
  bool getESFSTATUS(uint16_t maxWait = defaultMaxWait);                                                                     // ESF STATUS
  bool setAutoESFSTATUS(bool enabled, uint16_t maxWait = defaultMaxWait);                                                   // Enable/disable automatic ESF STATUS reports
  bool setAutoESFSTATUS(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                              // Enable/disable automatic ESF STATUS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoESFSTATUSrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                   // Set the rate for automatic STATUS reports
  bool setAutoESFSTATUScallback(void (*callbackPointer)(UBX_ESF_STATUS_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic STATUS reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoESFSTATUScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_STATUS_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic STATUS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoESFSTATUS(bool enabled, bool implicitUpdate = true);                                                       // In case no config access to the GPS is possible and ESF STATUS is send cyclically already
  void flushESFSTATUS();                                                                                                    // Mark all the data as read/stale
  void logESFSTATUS(bool enabled = true);                                                                                   // Log data to file buffer

  bool getEsfIns(uint16_t maxWait = defaultMaxWait);                                                                  // ESF INS Helper
  bool getESFINS(uint16_t maxWait = defaultMaxWait);                                                                  // ESF INS
  bool setAutoESFINS(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic ESF INS reports
  bool setAutoESFINS(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic ESF INS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoESFINSrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic INS reports
  bool setAutoESFINScallback(void (*callbackPointer)(UBX_ESF_INS_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic INS reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoESFINScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_INS_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic INS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoESFINS(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and ESF INS is send cyclically already
  void flushESFINS();                                                                                                 // Mark all the data as read/stale
  void logESFINS(bool enabled = true);                                                                                // Log data to file buffer

  bool setAutoESFMEAS(bool enabled, uint16_t maxWait = defaultMaxWait);                                                 // Enable/disable automatic ESF MEAS reports
  bool setAutoESFMEAS(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                            // Enable/disable automatic ESF MEAS reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoESFMEASrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                 // Set the rate for automatic MEAS reports
  bool setAutoESFMEAScallback(void (*callbackPointer)(UBX_ESF_MEAS_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic MEAS reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoESFMEAScallbackPtr(void (*callbackPointerPtr)(UBX_ESF_MEAS_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic MEAS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoESFMEAS(bool enabled, bool implicitUpdate = true);                                                     // In case no config access to the GPS is possible and ESF MEAS is send cyclically already
  void logESFMEAS(bool enabled = true);                                                                                 // Log data to file buffer

  bool setAutoESFRAW(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic ESF RAW reports
  bool setAutoESFRAW(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic ESF RAW reports, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoESFRAWrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic RAW reports
  bool setAutoESFRAWcallback(void (*callbackPointer)(UBX_ESF_RAW_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic RAW reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoESFRAWcallbackPtr(void (*callbackPointerPtr)(UBX_ESF_RAW_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic RAW reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoESFRAW(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and ESF RAW is send cyclically already
  void logESFRAW(bool enabled = true);                                                                                // Log data to file buffer

  // High navigation rate (HNR)

  bool getHNRAtt(uint16_t maxWait = defaultMaxWait);                                                                  // HNR ATT Helper
  bool getHNRATT(uint16_t maxWait = defaultMaxWait);                                                                  // Returns true if the get HNR attitude is successful
  bool setAutoHNRATT(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic HNR Attitude reports at the HNR rate
  bool setAutoHNRATT(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic HNR Attitude reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoHNRATTrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic ATT reports
  bool setAutoHNRATTcallback(void (*callbackPointer)(UBX_HNR_ATT_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic ATT reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoHNRATTcallbackPtr(void (*callbackPointerPtr)(UBX_HNR_ATT_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic ATT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoHNRATT(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and HNR Attitude is send cyclically already
  void flushHNRATT();                                                                                                 // Mark all the data as read/stale
  void logHNRATT(bool enabled = true);                                                                                // Log data to file buffer

  bool getHNRDyn(uint16_t maxWait = defaultMaxWait);                                                                  // HNR INS Helper
  bool getHNRINS(uint16_t maxWait = defaultMaxWait);                                                                  // Returns true if the get HNR dynamics is successful
  bool setAutoHNRINS(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic HNR dynamics reports at the HNR rate
  bool setAutoHNRINS(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic HNR dynamics reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoHNRINSrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic INS reports
  bool setAutoHNRINScallback(void (*callbackPointer)(UBX_HNR_INS_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic INS reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoHNRINScallbackPtr(void (*callbackPointerPtr)(UBX_HNR_INS_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic INS reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoHNRINS(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and HNR dynamics is send cyclically already
  void flushHNRINS();                                                                                                 // Mark all the data as read/stale
  void logHNRINS(bool enabled = true);                                                                                // Log data to file buffer

  bool getHNRPVT(uint16_t maxWait = defaultMaxWait);                                                                  // Returns true if the get HNR PVT is successful
  bool setAutoHNRPVT(bool enabled, uint16_t maxWait = defaultMaxWait);                                                // Enable/disable automatic HNR PVT reports at the HNR rate
  bool setAutoHNRPVT(bool enabled, bool implicitUpdate, uint16_t maxWait = defaultMaxWait);                           // Enable/disable automatic HNR PVT reports at the HNR rate, with implicitUpdate == false accessing stale data will not issue parsing of data in the rxbuffer of your interface, instead you have to call checkUblox when you want to perform an update
  bool setAutoHNRPVTrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait = defaultMaxWait);                // Set the rate for automatic PVT reports
  bool setAutoHNRPVTcallback(void (*callbackPointer)(UBX_HNR_PVT_data_t), uint16_t maxWait = defaultMaxWait);         // Enable automatic PVT reports at the navigation frequency. Data is accessed from the callback.
  bool setAutoHNRPVTcallbackPtr(void (*callbackPointerPtr)(UBX_HNR_PVT_data_t *), uint16_t maxWait = defaultMaxWait); // Enable automatic PVT reports at the navigation frequency. Data is accessed from the callback.
  bool assumeAutoHNRPVT(bool enabled, bool implicitUpdate = true);                                                    // In case no config access to the GPS is possible and HNR PVT is send cyclically already
  void flushHNRPVT();                                                                                                 // Mark all the data as read/stale
  void logHNRPVT(bool enabled = true);                                                                                // Log data to file buffer

  // Helper functions for CFG RATE

  bool setNavigationFrequency(uint8_t navFreq, uint16_t maxWait = defaultMaxWait); // Set the number of nav solutions sent per second
  uint8_t getNavigationFrequency(uint16_t maxWait = defaultMaxWait);               // Get the number of nav solutions sent per second currently being output by module
  bool setMeasurementRate(uint16_t rate, uint16_t maxWait = defaultMaxWait);       // Set the elapsed time between GNSS measurements in milliseconds, which defines the rate
  uint16_t getMeasurementRate(uint16_t maxWait = defaultMaxWait);                  // Return the elapsed time between GNSS measurements in milliseconds
  bool setNavigationRate(uint16_t rate, uint16_t maxWait = defaultMaxWait);        // Set the ratio between the number of measurements and the number of navigation solutions. Unit is cycles. Max is 127
  uint16_t getNavigationRate(uint16_t maxWait = defaultMaxWait);                   // Return the ratio between the number of measurements and the number of navigation solutions. Unit is cycles
  void flushCFGRATE();                                                             // Mark the measurement and navigation rate data as stale - used by the set rate functions

  // Helper functions for DOP

  uint16_t getGeometricDOP(uint16_t maxWait = defaultMaxWait);
  uint16_t getPositionDOP(uint16_t maxWait = defaultMaxWait);
  uint16_t getTimeDOP(uint16_t maxWait = defaultMaxWait);
  uint16_t getVerticalDOP(uint16_t maxWait = defaultMaxWait);
  uint16_t getHorizontalDOP(uint16_t maxWait = defaultMaxWait);
  uint16_t getNorthingDOP(uint16_t maxWait = defaultMaxWait);
  uint16_t getEastingDOP(uint16_t maxWait = defaultMaxWait);

  // Helper functions for ATT

  float getATTroll(uint16_t maxWait = defaultMaxWait);    // Returned as degrees
  float getATTpitch(uint16_t maxWait = defaultMaxWait);   // Returned as degrees
  float getATTheading(uint16_t maxWait = defaultMaxWait); // Returned as degrees

  // Helper functions for PVT

  uint32_t getTimeOfWeek(uint16_t maxWait = defaultMaxWait);
  uint16_t getYear(uint16_t maxWait = defaultMaxWait);
  uint8_t getMonth(uint16_t maxWait = defaultMaxWait);
  uint8_t getDay(uint16_t maxWait = defaultMaxWait);
  uint8_t getHour(uint16_t maxWait = defaultMaxWait);
  uint8_t getMinute(uint16_t maxWait = defaultMaxWait);
  uint8_t getSecond(uint16_t maxWait = defaultMaxWait);
  uint16_t getMillisecond(uint16_t maxWait = defaultMaxWait);
  int32_t getNanosecond(uint16_t maxWait = defaultMaxWait);
  uint32_t getUnixEpoch(uint16_t maxWait = defaultMaxWait);
  uint32_t getUnixEpoch(uint32_t &microsecond, uint16_t maxWait = defaultMaxWait);

  bool getDateValid(uint16_t maxWait = defaultMaxWait);
  bool getTimeValid(uint16_t maxWait = defaultMaxWait);
  bool getTimeFullyResolved(uint16_t maxWait = defaultMaxWait);
  bool getConfirmedDate(uint16_t maxWait = defaultMaxWait);
  bool getConfirmedTime(uint16_t maxWait = defaultMaxWait);

  uint8_t getFixType(uint16_t maxWait = defaultMaxWait); // Returns the type of fix: 0=no, 3=3D, 4=GNSS+Deadreckoning

  bool getGnssFixOk(uint16_t maxWait = defaultMaxWait); // Get whether we have a valid fix (i.e within DOP & accuracy masks)
  bool getNAVPVTPSMMode(uint16_t maxWait = defaultMaxWait); // Not fully documented power save mode value
  bool getDiffSoln(uint16_t maxWait = defaultMaxWait);  // Get whether differential corrections were applied
  bool getHeadVehValid(uint16_t maxWait = defaultMaxWait);
  uint8_t getCarrierSolutionType(uint16_t maxWait = defaultMaxWait); // Returns RTK solution: 0=no, 1=float solution, 2=fixed solution

  uint8_t getSIV(uint16_t maxWait = defaultMaxWait);         // Returns number of sats used in fix
  int32_t getLongitude(uint16_t maxWait = defaultMaxWait);   // Returns the current longitude in degrees * 10-7. Auto selects between HighPrecision and Regular depending on ability of module.
  int32_t getLatitude(uint16_t maxWait = defaultMaxWait);    // Returns the current latitude in degrees * 10^-7. Auto selects between HighPrecision and Regular depending on ability of module.
  int32_t getAltitude(uint16_t maxWait = defaultMaxWait);    // Returns the current altitude in mm above ellipsoid
  int32_t getAltitudeMSL(uint16_t maxWait = defaultMaxWait); // Returns the current altitude in mm above mean sea level
  int32_t getHorizontalAccEst(uint16_t maxWait = defaultMaxWait);
  int32_t getVerticalAccEst(uint16_t maxWait = defaultMaxWait);
  int32_t getNedNorthVel(uint16_t maxWait = defaultMaxWait);
  int32_t getNedEastVel(uint16_t maxWait = defaultMaxWait);
  int32_t getNedDownVel(uint16_t maxWait = defaultMaxWait);
  int32_t getGroundSpeed(uint16_t maxWait = defaultMaxWait); // Returns speed in mm/s
  int32_t getHeading(uint16_t maxWait = defaultMaxWait);     // Returns heading in degrees * 10^-5
  uint32_t getSpeedAccEst(uint16_t maxWait = defaultMaxWait);
  uint32_t getHeadingAccEst(uint16_t maxWait = defaultMaxWait);
  uint16_t getPDOP(uint16_t maxWait = defaultMaxWait); // Returns positional dillution of precision * 10^-2 (dimensionless)

  bool getInvalidLlh(uint16_t maxWait = defaultMaxWait);

  int32_t getHeadVeh(uint16_t maxWait = defaultMaxWait);
  int16_t getMagDec(uint16_t maxWait = defaultMaxWait);
  uint16_t getMagAcc(uint16_t maxWait = defaultMaxWait);

  int32_t getGeoidSeparation(uint16_t maxWait = defaultMaxWait);

  // Helper functions for HPPOSECEF

  uint32_t getPositionAccuracy(uint16_t maxWait = defaultMaxWait); // Returns the 3D accuracy of the current high-precision fix, in mm. Supported on NEO-M8P, ZED-F9P,
  int32_t getHighResECEFX(uint16_t maxWait = defaultMaxWait);      // Returns the ECEF X coordinate (cm)
  int32_t getHighResECEFY(uint16_t maxWait = defaultMaxWait);      // Returns the ECEF Y coordinate (cm)
  int32_t getHighResECEFZ(uint16_t maxWait = defaultMaxWait);      // Returns the ECEF Z coordinate (cm)
  int8_t getHighResECEFXHp(uint16_t maxWait = defaultMaxWait);     // Returns the ECEF X coordinate High Precision Component (0.1 mm)
  int8_t getHighResECEFYHp(uint16_t maxWait = defaultMaxWait);     // Returns the ECEF Y coordinate High Precision Component (0.1 mm)
  int8_t getHighResECEFZHp(uint16_t maxWait = defaultMaxWait);     // Returns the ECEF Z coordinate High Precision Component (0.1 mm)

  // Helper functions for HPPOSLLH

  uint32_t getTimeOfWeekFromHPPOSLLH(uint16_t maxWait = defaultMaxWait);
  int32_t getHighResLongitude(uint16_t maxWait = defaultMaxWait);
  int32_t getHighResLatitude(uint16_t maxWait = defaultMaxWait);
  int32_t getElipsoid(uint16_t maxWait = defaultMaxWait);
  int32_t getMeanSeaLevel(uint16_t maxWait = defaultMaxWait);
  int8_t getHighResLongitudeHp(uint16_t maxWait = defaultMaxWait);
  int8_t getHighResLatitudeHp(uint16_t maxWait = defaultMaxWait);
  int8_t getElipsoidHp(uint16_t maxWait = defaultMaxWait);
  int8_t getMeanSeaLevelHp(uint16_t maxWait = defaultMaxWait);
  uint32_t getHorizontalAccuracy(uint16_t maxWait = defaultMaxWait);
  uint32_t getVerticalAccuracy(uint16_t maxWait = defaultMaxWait);

  // Helper functions for PVAT

  int32_t getVehicleRoll(uint16_t maxWait = defaultMaxWait);    // Returns vehicle roll in degrees * 10^-5
  int32_t getVehiclePitch(uint16_t maxWait = defaultMaxWait);   // Returns vehicle pitch in degrees * 10^-5
  int32_t getVehicleHeading(uint16_t maxWait = defaultMaxWait); // Returns vehicle heading in degrees * 10^-5
  int32_t getMotionHeading(uint16_t maxWait = defaultMaxWait);  // Returns the motion heading in degrees * 10^-5

  // Helper functions for SVIN

  bool getSurveyInActive(uint16_t maxWait = defaultMaxWait);
  bool getSurveyInValid(uint16_t maxWait = defaultMaxWait);
  uint16_t getSurveyInObservationTime(uint16_t maxWait = defaultMaxWait);     // Truncated to 65535 seconds
  uint32_t getSurveyInObservationTimeFull(uint16_t maxWait = defaultMaxWait); // Return the full uint32_t
  float getSurveyInMeanAccuracy(uint16_t maxWait = defaultMaxWait);           // Returned as m

  // Helper functions for TIMELS

  uint8_t getLeapIndicator(int32_t &timeToLsEvent, uint16_t maxWait = defaultMaxWait);
  int8_t getCurrentLeapSeconds(sfe_ublox_ls_src_e &source, uint16_t maxWait = defaultMaxWait);

  // Helper functions for RELPOSNED

  float getRelPosN(uint16_t maxWait = defaultMaxWait);    // Returned as m
  float getRelPosE(uint16_t maxWait = defaultMaxWait);    // Returned as m
  float getRelPosD(uint16_t maxWait = defaultMaxWait);    // Returned as m
  float getRelPosAccN(uint16_t maxWait = defaultMaxWait); // Returned as m
  float getRelPosAccE(uint16_t maxWait = defaultMaxWait); // Returned as m
  float getRelPosAccD(uint16_t maxWait = defaultMaxWait); // Returned as m

  // Helper functions for AOPSTATUS

  uint8_t getAOPSTATUSuseAOP(uint16_t maxWait = defaultMaxWait); // Returns the UBX-NAV-AOPSTATUS useAOP flag. Don't confuse this with getAopCfg - which returns the aopCfg byte from UBX-CFG-NAVX5
  uint8_t getAOPSTATUSstatus(uint16_t maxWait = defaultMaxWait); // Returns the UBX-NAV-AOPSTATUS status field. A host application can determine the optimal time to shut down the receiver by monitoring the status field for a steady 0.

  // Helper functions for ESF

  float getESFroll(uint16_t maxWait = defaultMaxWait);  // Returned as degrees
  float getESFpitch(uint16_t maxWait = defaultMaxWait); // Returned as degrees
  float getESFyaw(uint16_t maxWait = defaultMaxWait);   // Returned as degrees
  bool getSensorFusionMeasurement(UBX_ESF_MEAS_sensorData_t *sensorData, UBX_ESF_MEAS_data_t ubxDataStruct, uint8_t sensor);
  bool getRawSensorMeasurement(UBX_ESF_RAW_sensorData_t *sensorData, UBX_ESF_RAW_data_t ubxDataStruct, uint8_t sensor);
  bool getSensorFusionStatus(UBX_ESF_STATUS_sensorStatus_t *sensorStatus, uint8_t sensor, uint16_t maxWait = defaultMaxWait);
  bool getSensorFusionStatus(UBX_ESF_STATUS_sensorStatus_t *sensorStatus, UBX_ESF_STATUS_data_t ubxDataStruct, uint8_t sensor);

  // Helper functions for HNR

  bool setHNRNavigationRate(uint8_t rate, uint16_t maxWait = defaultMaxWait); // Returns true if the setHNRNavigationRate is successful
  uint8_t getHNRNavigationRate(uint16_t maxWait = defaultMaxWait);            // Returns 0 if the getHNRNavigationRate fails
  float getHNRroll(uint16_t maxWait = defaultMaxWait);                        // Returned as degrees
  float getHNRpitch(uint16_t maxWait = defaultMaxWait);                       // Returned as degrees
  float getHNRheading(uint16_t maxWait = defaultMaxWait);                     // Returned as degrees

  // Set the mainTalkerId used by NMEA messages - allows all NMEA messages except GSV to be prefixed with GP instead of GN
  bool setMainTalkerID(sfe_ublox_talker_ids_e id = SFE_UBLOX_MAIN_TALKER_ID_DEFAULT, uint16_t maxWait = defaultMaxWait);

  // Enable/Disable NMEA High Precision Mode - include extra decimal places in the Lat and Lon
  bool setHighPrecisionMode(bool enable = true, uint16_t maxWait = defaultMaxWait);

  // Helper functions for NMEA logging
  void setNMEALoggingMask(uint32_t messages = SFE_UBLOX_FILTER_NMEA_ALL); // Add selected NMEA messages to file buffer - if enabled. Default to adding ALL messages to the file buffer
  uint32_t getNMEALoggingMask();                                          // Return which NMEA messages are selected for logging to the file buffer - if enabled

  // Helper functions to control which NMEA messages are passed to processNMEA
  void setProcessNMEAMask(uint32_t messages = SFE_UBLOX_FILTER_NMEA_ALL); // Control which NMEA messages are passed to processNMEA. Default to passing ALL messages
  uint32_t getProcessNMEAMask();                                          // Return which NMEA messages are passed to processNMEA

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  // Support for "auto" storage of NMEA messages
  uint8_t getLatestNMEAGPGGA(NMEA_GGA_data_t *data);                           // Return the most recent GPGGA: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGPGGAcallback(void (*callbackPointer)(NMEA_GGA_data_t));         // Enable a callback on the arrival of a GPGGA message
  bool setNMEAGPGGAcallbackPtr(void (*callbackPointerPtr)(NMEA_GGA_data_t *)); // Enable a callback on the arrival of a GPGGA message
  uint8_t getLatestNMEAGNGGA(NMEA_GGA_data_t *data);                           // Return the most recent GNGGA: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGNGGAcallback(void (*callbackPointer)(NMEA_GGA_data_t));         // Enable a callback on the arrival of a GNGGA message
  bool setNMEAGNGGAcallbackPtr(void (*callbackPointerPtr)(NMEA_GGA_data_t *)); // Enable a callback on the arrival of a GNGGA message
  uint8_t getLatestNMEAGPVTG(NMEA_VTG_data_t *data);                           // Return the most recent GPVTG: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGPVTGcallback(void (*callbackPointer)(NMEA_VTG_data_t));         // Enable a callback on the arrival of a GPVTG message
  bool setNMEAGPVTGcallbackPtr(void (*callbackPointerPtr)(NMEA_VTG_data_t *)); // Enable a callback on the arrival of a GPVTG message
  uint8_t getLatestNMEAGNVTG(NMEA_VTG_data_t *data);                           // Return the most recent GNVTG: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGNVTGcallback(void (*callbackPointer)(NMEA_VTG_data_t));         // Enable a callback on the arrival of a GNVTG message
  bool setNMEAGNVTGcallbackPtr(void (*callbackPointerPtr)(NMEA_VTG_data_t *)); // Enable a callback on the arrival of a GNVTG message
  uint8_t getLatestNMEAGPRMC(NMEA_RMC_data_t *data);                           // Return the most recent GPRMC: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGPRMCcallback(void (*callbackPointer)(NMEA_RMC_data_t));         // Enable a callback on the arrival of a GPRMC message
  bool setNMEAGPRMCcallbackPtr(void (*callbackPointerPtr)(NMEA_RMC_data_t *)); // Enable a callback on the arrival of a GPRMC message
  uint8_t getLatestNMEAGNRMC(NMEA_RMC_data_t *data);                           // Return the most recent GNRMC: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGNRMCcallback(void (*callbackPointer)(NMEA_RMC_data_t));         // Enable a callback on the arrival of a GNRMC message
  bool setNMEAGNRMCcallbackPtr(void (*callbackPointerPtr)(NMEA_RMC_data_t *)); // Enable a callback on the arrival of a GNRMC message
  uint8_t getLatestNMEAGPZDA(NMEA_ZDA_data_t *data);                           // Return the most recent GPZDA: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGPZDAcallback(void (*callbackPointer)(NMEA_ZDA_data_t));         // Enable a callback on the arrival of a GPZDA message
  bool setNMEAGPZDAcallbackPtr(void (*callbackPointerPtr)(NMEA_ZDA_data_t *)); // Enable a callback on the arrival of a GPZDA message
  uint8_t getLatestNMEAGNZDA(NMEA_ZDA_data_t *data);                           // Return the most recent GNZDA: 0 = no data, 1 = stale data, 2 = fresh data
  bool setNMEAGNZDAcallback(void (*callbackPointer)(NMEA_ZDA_data_t));         // Enable a callback on the arrival of a GNZDA message
  bool setNMEAGNZDAcallbackPtr(void (*callbackPointerPtr)(NMEA_ZDA_data_t *)); // Enable a callback on the arrival of a GNZDA message
#endif

  // Functions to extract signed and unsigned 8/16/32-bit data from a ubxPacket
  // From v2.0: These are public. The user can call these to extract data from custom packets
  uint64_t extractLongLong(ubxPacket *msg, uint16_t spotToStart);  // Combine eight bytes from payload into uint64_t
  uint32_t extractLong(ubxPacket *msg, uint16_t spotToStart);      // Combine four bytes from payload into long
  int32_t extractSignedLong(ubxPacket *msg, uint16_t spotToStart); // Combine four bytes from payload into signed long (avoiding any ambiguity caused by casting)
  uint16_t extractInt(ubxPacket *msg, uint16_t spotToStart);       // Combine two bytes from payload into int
  int16_t extractSignedInt(ubxPacket *msg, uint16_t spotToStart);
  uint8_t extractByte(ubxPacket *msg, uint16_t spotToStart);      // Get byte from payload
  int8_t extractSignedChar(ubxPacket *msg, uint16_t spotToStart); // Get signed 8-bit value from payload

  // Pointers to storage for the "automatic" messages
  // RAM is allocated for these if/when required.

  UBX_NAV_POSECEF_t *packetUBXNAVPOSECEF = NULL;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_STATUS_t *packetUBXNAVSTATUS = NULL;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_DOP_t *packetUBXNAVDOP = NULL;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_EOE_t *packetUBXNAVEOE = NULL;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_ATT_t *packetUBXNAVATT = NULL;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_PVT_t *packetUBXNAVPVT = NULL;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_ODO_t *packetUBXNAVODO = NULL;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_TIMEUTC_t *packetUBXNAVTIMEUTC = NULL;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_VELECEF_t *packetUBXNAVVELECEF = NULL;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_VELNED_t *packetUBXNAVVELNED = NULL;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_HPPOSECEF_t *packetUBXNAVHPPOSECEF = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_HPPOSLLH_t *packetUBXNAVHPPOSLLH = NULL;   // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_PVAT_t *packetUBXNAVPVAT = NULL;           // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_CLOCK_t *packetUBXNAVCLOCK = NULL;         // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_TIMELS_t *packetUBXNAVTIMELS = NULL;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_SVIN_t *packetUBXNAVSVIN = NULL;           // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_SAT_t *packetUBXNAVSAT = NULL;             // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_RELPOSNED_t *packetUBXNAVRELPOSNED = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_NAV_AOPSTATUS_t *packetUBXNAVAOPSTATUS = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

  UBX_RXM_PMP_t *packetUBXRXMPMP = NULL;                      // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_PMP_message_t *packetUBXRXMPMPmessage = NULL;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_QZSSL6_message_t *packetUBXRXMQZSSL6message = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_COR_t *packetUBXRXMCOR = NULL;                      // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_SFRBX_t *packetUBXRXMSFRBX = NULL;                  // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_RXM_RAWX_t *packetUBXRXMRAWX = NULL;                    // Pointer to struct. RAM will be allocated for this if/when necessary

  UBX_CFG_PRT_t *packetUBXCFGPRT = NULL;   // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_CFG_RATE_t *packetUBXCFGRATE = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

  UBX_TIM_TM2_t *packetUBXTIMTM2 = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

  UBX_ESF_ALG_t *packetUBXESFALG = NULL;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_INS_t *packetUBXESFINS = NULL;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_MEAS_t *packetUBXESFMEAS = NULL;     // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_RAW_t *packetUBXESFRAW = NULL;       // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_ESF_STATUS_t *packetUBXESFSTATUS = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

  UBX_HNR_PVT_t *packetUBXHNRPVT = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_HNR_ATT_t *packetUBXHNRATT = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_HNR_INS_t *packetUBXHNRINS = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary

  UBX_MGA_ACK_DATA0_t *packetUBXMGAACK = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  UBX_MGA_DBD_t *packetUBXMGADBD = NULL;       // Pointer to struct. RAM will be allocated for this if/when necessary

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  NMEA_GPGGA_t *storageNMEAGPGGA = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNGGA_t *storageNMEAGNGGA = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GPVTG_t *storageNMEAGPVTG = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNVTG_t *storageNMEAGNVTG = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GPRMC_t *storageNMEAGPRMC = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNRMC_t *storageNMEAGNRMC = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GPZDA_t *storageNMEAGPZDA = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
  NMEA_GNZDA_t *storageNMEAGNZDA = NULL; // Pointer to struct. RAM will be allocated for this if/when necessary
#endif

  uint16_t rtcmFrameCounter = 0; // Tracks the type of incoming byte inside RTCM frame

private:
  // Depending on the ubx binary response class, store binary responses into different places
  enum classTypes
  {
    CLASS_NONE = 0,
    CLASS_ACK,
    CLASS_NOT_AN_ACK
  } ubxFrameClass = CLASS_NONE;

  enum commTypes
  {
    COMM_TYPE_I2C = 0,
    COMM_TYPE_SERIAL,
    COMM_TYPE_SPI
  } commType = COMM_TYPE_I2C; // Controls which port we look to for incoming bytes

  // Functions
  bool checkUbloxInternal(ubxPacket *incomingUBX, uint8_t requestedClass = 255, uint8_t requestedID = 255); // Checks module with user selected commType
  void addToChecksum(uint8_t incoming);                                                                     // Given an incoming byte, adjust rollingChecksumA/B
  size_t pushAssistNowDataInternal(size_t offset, bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait);
  size_t findMGAANOForDateInternal(const uint8_t *dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture);

  // Return true if this "automatic" message has storage allocated for it
  bool checkAutomatic(uint8_t Class, uint8_t ID);

  // Calculate how much RAM is needed to store the payload for a given automatic message
  uint16_t getMaxPayloadSize(uint8_t Class, uint8_t ID);

  // Do the actual transfer to SPI
  void spiTransfer(uint8_t byteToTransfer);

  bool initGeofenceParams();  // Allocate RAM for currentGeofenceParams and initialize it
  bool initModuleSWVersion(); // Allocate RAM for moduleSWVersion and initialize it

  // The initPacket functions need to be private as they don't check if memory has already been allocated.
  // Functions like setAutoNAVPOSECEF will check that memory has not been allocated before calling initPacket.
  bool initPacketUBXNAVPOSECEF();       // Allocate RAM for packetUBXNAVPOSECEF and initialize it
  bool initPacketUBXNAVSTATUS();        // Allocate RAM for packetUBXNAVSTATUS and initialize it
  bool initPacketUBXNAVDOP();           // Allocate RAM for packetUBXNAVDOP and initialize it
  bool initPacketUBXNAVATT();           // Allocate RAM for packetUBXNAVATT and initialize it
  bool initPacketUBXNAVPVT();           // Allocate RAM for packetUBXNAVPVT and initialize it
  bool initPacketUBXNAVODO();           // Allocate RAM for packetUBXNAVODO and initialize it
  bool initPacketUBXNAVVELECEF();       // Allocate RAM for packetUBXNAVVELECEF and initialize it
  bool initPacketUBXNAVVELNED();        // Allocate RAM for packetUBXNAVVELNED and initialize it
  bool initPacketUBXNAVHPPOSECEF();     // Allocate RAM for packetUBXNAVHPPOSECEF and initialize it
  bool initPacketUBXNAVHPPOSLLH();      // Allocate RAM for packetUBXNAVHPPOSLLH and initialize it
  bool initPacketUBXNAVPVAT();          // Allocate RAM for packetUBXNAVPVAT and initialize it
  bool initPacketUBXNAVTIMEUTC();       // Allocate RAM for packetUBXNAVTIMEUTC and initialize it
  bool initPacketUBXNAVCLOCK();         // Allocate RAM for packetUBXNAVCLOCK and initialize it
  bool initPacketUBXNAVTIMELS();        // Allocate RAM for packetUBXNAVTIMELS and initialize it
  bool initPacketUBXNAVSVIN();          // Allocate RAM for packetUBXNAVSVIN and initialize it
  bool initPacketUBXNAVSAT();           // Allocate RAM for packetUBXNAVSAT and initialize it
  bool initPacketUBXNAVRELPOSNED();     // Allocate RAM for packetUBXNAVRELPOSNED and initialize it
  bool initPacketUBXNAVAOPSTATUS();     // Allocate RAM for packetUBXNAVAOPSTATUS and initialize it
  bool initPacketUBXNAVEOE();           // Allocate RAM for packetUBXNAVEOE and initialize it
  bool initPacketUBXRXMPMP();           // Allocate RAM for packetUBXRXMPMP and initialize it
  bool initPacketUBXRXMPMPmessage();    // Allocate RAM for packetUBXRXMPMPRaw and initialize it
  bool initPacketUBXRXMQZSSL6message(); // Allocate RAM for packetUBXRXMQZSSL6raw and initialize it
  bool initPacketUBXRXMCOR();           // Allocate RAM for packetUBXRXMCOR and initialize it
  bool initPacketUBXRXMSFRBX();         // Allocate RAM for packetUBXRXMSFRBX and initialize it
  bool initPacketUBXRXMRAWX();          // Allocate RAM for packetUBXRXMRAWX and initialize it
  bool initPacketUBXCFGPRT();           // Allocate RAM for packetUBXCFGPRT and initialize it
  bool initPacketUBXCFGRATE();          // Allocate RAM for packetUBXCFGRATE and initialize it
  bool initPacketUBXTIMTM2();           // Allocate RAM for packetUBXTIMTM2 and initialize it
  bool initPacketUBXESFALG();           // Allocate RAM for packetUBXESFALG and initialize it
  bool initPacketUBXESFSTATUS();        // Allocate RAM for packetUBXESFSTATUS and initialize it
  bool initPacketUBXESFINS();           // Allocate RAM for packetUBXESFINS and initialize it
  bool initPacketUBXESFMEAS();          // Allocate RAM for packetUBXESFMEAS and initialize it
  bool initPacketUBXESFRAW();           // Allocate RAM for packetUBXESFRAW and initialize it
  bool initPacketUBXHNRATT();           // Allocate RAM for packetUBXHNRATT and initialize it
  bool initPacketUBXHNRINS();           // Allocate RAM for packetUBXHNRINS and initialize it
  bool initPacketUBXHNRPVT();           // Allocate RAM for packetUBXHNRPVT and initialize it
  bool initPacketUBXMGAACK();           // Allocate RAM for packetUBXMGAACK and initialize it
  bool initPacketUBXMGADBD();           // Allocate RAM for packetUBXMGADBD and initialize it

#ifndef SFE_UBLOX_DISABLE_AUTO_NMEA
  bool initStorageNMEAGPGGA(); // Allocate RAM for incoming NMEA GPGGA messages and initialize it
  bool initStorageNMEAGNGGA(); // Allocate RAM for incoming NMEA GNGGA messages and initialize it
  bool initStorageNMEAGPVTG(); // Allocate RAM for incoming NMEA GPVTG messages and initialize it
  bool initStorageNMEAGNVTG(); // Allocate RAM for incoming NMEA GNVTG messages and initialize it
  bool initStorageNMEAGPRMC(); // Allocate RAM for incoming NMEA GPRMC messages and initialize it
  bool initStorageNMEAGNRMC(); // Allocate RAM for incoming NMEA GNRMC messages and initialize it
  bool initStorageNMEAGPZDA(); // Allocate RAM for incoming NMEA GPZDA messages and initialize it
  bool initStorageNMEAGNZDA(); // Allocate RAM for incoming NMEA GNZDA messages and initialize it
#endif

  // Variables
  TwoWire *_i2cPort;              // The generic connection to user's chosen I2C hardware
  Stream *_serialPort;            // The generic connection to user's chosen Serial hardware
  Stream *_nmeaOutputPort = NULL; // The user can assign an output port to print NMEA sentences if they wish
  Stream *_debugSerial;           // The stream to send debug messages to if enabled
  Stream *_outputPort = NULL;
  SPIClass *_spiPort; // The instance of SPIClass
  uint8_t _csPin;     // The chip select pin
  uint32_t _spiSpeed; // The speed to use for SPI (Hz)

  uint8_t _gpsI2Caddress = 0x42; // Default 7-bit unshifted address of the ublox 6/7/8/M8/F9 series
  // This can be changed using the ublox configuration software

  bool _printDebug = false;        // Flag to print the serial commands we are sending to the Serial port for debug
  bool _printLimitedDebug = false; // Flag to print limited debug messages. Useful for I2C debugging or high navigation rates

  bool ubx7FcheckDisabled = false; // Flag to indicate if the "7F" check should be ignored in checkUbloxI2C

  sfe_ublox_nmea_filtering_t _logNMEA;     // Flags to indicate which NMEA messages should be added to the file buffer for logging
  sfe_ublox_nmea_filtering_t _processNMEA; // Flags to indicate which NMEA messages should be passed to processNMEA

  // The packet buffers
  // These are pointed at from within the ubxPacket
  uint8_t payloadAck[2];           // Holds the requested ACK/NACK
  uint8_t payloadBuf[2];           // Temporary buffer used to screen incoming packets or dump unrequested packets
  size_t packetCfgPayloadSize = 0; // Size for the packetCfg payload. .begin will set this to MAX_PAYLOAD_SIZE if necessary. User can change with setPacketCfgPayloadSize
  uint8_t *payloadCfg = NULL;
  uint8_t *payloadAuto = NULL;

  uint8_t *spiBuffer = NULL;                              // A buffer to store any bytes being recieved back from the device while we are sending via SPI
  uint8_t spiBufferIndex = 0;                             // Index into the SPI buffer
  uint8_t spiTransactionSize = SFE_UBLOX_SPI_BUFFER_SIZE; // Default size of the SPI buffer

  // Init the packet structures and init them with pointers to the payloadAck, payloadCfg, payloadBuf and payloadAuto arrays
  ubxPacket packetAck = {0, 0, 0, 0, 0, payloadAck, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
  ubxPacket packetBuf = {0, 0, 0, 0, 0, payloadBuf, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
  ubxPacket packetCfg = {0, 0, 0, 0, 0, payloadCfg, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
  ubxPacket packetAuto = {0, 0, 0, 0, 0, payloadAuto, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  // Flag if this packet is unrequested (and so should be ignored and not copied into packetCfg or packetAck)
  bool ignoreThisPayload = false;

  // Identify which buffer is in use
  // Data is stored in packetBuf until the requested class and ID can be validated
  // If a match is seen, data is diverted into packetAck or packetCfg
  //"Automatic" messages which have RAM allocated for them are diverted into packetAuto
  sfe_ublox_packet_buffer_e activePacketBuffer = SFE_UBLOX_PACKET_PACKETBUF;

  // Limit checking of new data to every X ms
  // If we are expecting an update every X Hz then we should check every quarter that amount of time
  // Otherwise we may block ourselves from seeing new data
  uint8_t i2cPollingWait = 100;    // Default to 100ms. Adjusted when user calls setNavigationFrequency() or setHNRNavigationRate() or setMeasurementRate()
  uint8_t i2cPollingWaitNAV = 100; // We need to record the desired polling rate for standard nav messages
  uint8_t i2cPollingWaitHNR = 100; // and for HNR too so we can set i2cPollingWait to the lower of the two

  // The SPI polling wait is a little different. checkUbloxSpi will delay for this amount before returning if
  // there is no data waiting to be read. This prevents waitForACKResponse from pounding the SPI bus too hard.
  uint8_t spiPollingWait = 9; // Default to 9ms; waitForACKResponse delays for 1ms on top of this. User can adjust with setSPIPollingWait.

  unsigned long lastCheck = 0;

  uint16_t ubxFrameCounter; // Count all UBX frame bytes. [Fixed header(2bytes), CLS(1byte), ID(1byte), length(2bytes), payload(x bytes), checksums(2bytes)]
  uint8_t rollingChecksumA; // Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes
  uint8_t rollingChecksumB; // Rolls forward as we receive incoming bytes. Checked against the last two A/B checksum bytes

  int8_t nmeaByteCounter; // Count all NMEA message bytes.
  // Abort NMEA message reception if nmeaByteCounter exceeds maxNMEAByteCount.
  // The user can adjust maxNMEAByteCount by calling setMaxNMEAByteCount
  int8_t maxNMEAByteCount = SFE_UBLOX_MAX_NMEA_BYTE_COUNT;
  uint8_t nmeaAddressField[6]; // NMEA Address Field - includes the start character (*)
  bool logThisNMEA();          // Return true if we should log this NMEA message
  bool processThisNMEA();      // Return true if we should pass this NMEA message to processNMEA
  bool isNMEAHeaderValid();    // Return true if the six byte NMEA header appears valid. Used to set _signsOfLife

  bool isThisNMEAauto();                 // Check if the NMEA message (in nmeaAddressField) is "auto" (i.e. has RAM allocated for it)
  bool doesThisNMEAHaveCallback();       // Do we need to copy the data into the callback copy?
  uint8_t *getNMEAWorkingLengthPtr();    // Get a pointer to the working copy length
  uint8_t *getNMEAWorkingNMEAPtr();      // Get a pointer to the working copy NMEA data
  uint8_t *getNMEACompleteLengthPtr();   // Get a pointer to the complete copy length
  uint8_t *getNMEACompleteNMEAPtr();     // Get a pointer to the complete copy NMEA data
  uint8_t *getNMEACallbackLengthPtr();   // Get a pointer to the callback copy length
  uint8_t *getNMEACallbackNMEAPtr();     // Get a pointer to the callback copy NMEA data
  uint8_t getNMEAMaxLength();            // Get the maximum length of this NMEA message
  nmeaAutomaticFlags *getNMEAFlagsPtr(); // Get a pointer to the flags

  // Flag to prevent reentry into checkCallbacks
  // Prevent badness if the user accidentally calls checkCallbacks from inside a callback
  volatile bool checkCallbacksReentrant = false;

  // Support for data logging
  uint8_t *ubxFileBuffer = NULL;                                // Pointer to the file buffer. RAM is allocated for this if required in .begin
  uint16_t fileBufferSize = 0;                                  // The size of the file buffer. This can be changed by calling setFileBufferSize _before_ .begin
  uint16_t fileBufferHead;                                      // The incoming byte is written into the file buffer at this location
  uint16_t fileBufferTail;                                      // The next byte to be read from the buffer will be read from this location
  uint16_t fileBufferMaxAvail = 0;                              // The maximum number of bytes the file buffer has contained. Handy for checking the buffer is large enough to handle all the incoming data.
  bool createFileBuffer(void);                                  // Create the file buffer. Called by .begin
  uint16_t fileBufferSpaceAvailable(void);                      // Check how much space is available in the buffer
  uint16_t fileBufferSpaceUsed(void);                           // Check how much space is used in the buffer
  bool storePacket(ubxPacket *msg);                             // Add a UBX packet to the file buffer
  bool storeFileBytes(uint8_t *theBytes, uint16_t numBytes);    // Add theBytes to the file buffer
  void writeToFileBuffer(uint8_t *theBytes, uint16_t numBytes); // Write theBytes to the file buffer

  // Support for platforms like ESP32 which do not support multiple I2C restarts
  // If _i2cStopRestart is true, endTransmission will always use a stop. If false, a restart will be used where needed.
  // The default value for _i2cStopRestart is set in the class instantiation code.
  bool _i2cStopRestart;

  // Storage just in case the user tries to push a single byte using pushRawBytes
  bool _pushSingleByte = false;
  uint8_t _pushThisSingleByte;

  // .begin will return true if the assumeSuccess parameter is true and if _signsOfLife is true
  // _signsOfLife is set to true when: a valid UBX message is seen; a valig NMEA header is seen.
  bool _signsOfLife;

  // Keep track of how many keys have been added to CfgValset
  uint8_t _numCfgKeyIDs = 0;

  // Send the current CFG_VALSET message when packetCfg has less than this many bytes available
  size_t _autoSendAtSpaceRemaining = 0;
};

#endif
