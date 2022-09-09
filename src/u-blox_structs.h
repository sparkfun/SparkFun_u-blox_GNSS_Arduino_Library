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

#ifndef __u_blox_structs_h__
#define __u_blox_structs_h__

#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

#ifndef DEF_NUM_SENS
#define DEF_NUM_SENS 7 // The maximum number of ESF sensors
#endif

#ifndef DEF_MAX_NUM_ESF_RAW_REPEATS
#define DEF_MAX_NUM_ESF_RAW_REPEATS 10 // The NEO-M8U sends ESF RAW data in blocks / sets of ten readings. (The ZED-F9R sends them one at a time.)
#endif

#ifndef DEF_MAX_NUM_ESF_MEAS
#define DEF_MAX_NUM_ESF_MEAS 31 // numMeas is 5 bits, indicating up to 31 groups could be received
#endif

// Additional flags and pointers that need to be stored with each message type
struct ubxAutomaticFlags
{
  union
  {
    uint8_t all;
    struct
    {
      uint8_t automatic : 1;         // Will this message be delivered and parsed "automatically" (without polling)
      uint8_t implicitUpdate : 1;    // Is the update triggered by accessing stale data (=true) or by a call to checkUblox (=false)
      uint8_t addToFileBuffer : 1;   // Should the raw UBX data be added to the file buffer?
      uint8_t callbackCopyValid : 1; // Is the copy of the data struct used by the callback valid/fresh?
    } bits;
  } flags;
};

// NAV-specific structs

// UBX-NAV-POSECEF (0x01 0x01): Position solution in ECEF
const uint16_t UBX_NAV_POSECEF_LEN = 20;

typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  int32_t ecefX; // ECEF X coordinate: cm
  int32_t ecefY; // ECEF Y coordinate: cm
  int32_t ecefZ; // ECEF Z coordinate: cm
  uint32_t pAcc; // Position Accuracy Estimate: cm
} UBX_NAV_POSECEF_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t ecefX : 1;
      uint32_t ecefY : 1;
      uint32_t ecefZ : 1;
      uint32_t pAcc : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_POSECEF_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_POSECEF_data_t data;
  UBX_NAV_POSECEF_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_POSECEF_data_t);
  void (*callbackPointerPtr)(UBX_NAV_POSECEF_data_t *);
  UBX_NAV_POSECEF_data_t *callbackData;
} UBX_NAV_POSECEF_t;

// UBX-NAV-POSLLH (0x01 0x02): Geodetic position solution
const uint16_t UBX_NAV_POSLLH_LEN = 28;

typedef struct
{
  uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
  int32_t lon;    // Longitude: Degrees * 1e-7
  int32_t lat;    // Latitude: Degrees * 1e-7
  int32_t height; // Height above ellipsoid: mm
  int32_t hMSL;   // Height above mean sea level: mm
  uint32_t hAcc;  // Horizontal Accuracy Estimate: mm
  uint32_t vAcc;  // Vertical Accuracy Estimate: mm
} UBX_NAV_POSLLH_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t lon : 1;
      uint32_t lat : 1;
      uint32_t height : 1;
      uint32_t hMSL : 1;
      uint32_t hAcc : 1;
      uint32_t vAcc : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_POSLLH_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_POSLLH_data_t data;
  UBX_NAV_POSLLH_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_POSLLH_data_t);
  void (*callbackPointerPtr)(UBX_NAV_POSLLH_data_t *);
  UBX_NAV_POSLLH_data_t *callbackData;
} UBX_NAV_POSLLH_t;

// UBX-NAV-STATUS (0x01 0x03): Receiver navigation status
const uint16_t UBX_NAV_STATUS_LEN = 16;

typedef struct
{
  uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
  uint8_t gpsFix; // GPSfix Type: 0x00 = no fix; 0x01 = dead reckoning only; 0x02 = 2D-fix; 0x03 = 3D-fix
                  // 0x04 = GPS + dead reckoning combined; 0x05 = Time only fix; 0x06..0xff = reserved
  union
  {
    uint8_t all;
    struct
    {
      uint8_t gpsFixOk : 1; // 1 = position and velocity valid and within DOP and ACC Masks.
      uint8_t diffSoln : 1; // 1 = differential corrections were applied
      uint8_t wknSet : 1;   // 1 = Week Number valid (see Time Validity section for details)
      uint8_t towSet : 1;   // 1 = Time of Week valid (see Time Validity section for details)
    } bits;
  } flags;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t diffCorr : 1;      // 1 = differential corrections available
      uint8_t carrSolnValid : 1; // 1 = valid carrSoln
      uint8_t reserved : 4;
      uint8_t mapMatching : 2; // map matching status: 00: none
                               // 01: valid but not used, i.e. map matching data was received, but was too old
                               // 10: valid and used, map matching data was applied
                               // 11: valid and used, map matching data was applied.
    } bits;
  } fixStat;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t psmState : 2; // power save mode state
                            // 0: ACQUISITION [or when psm disabled]
                            // 1: TRACKING
                            // 2: POWER OPTIMIZED TRACKING
                            // 3: INACTIVE
      uint8_t reserved1 : 1;
      uint8_t spoofDetState : 2; // Spoofing detection state
                                 // 0: Unknown or deactivated
                                 // 1: No spoofing indicated
                                 // 2: Spoofing indicated
                                 // 3: Multiple spoofing indications
      uint8_t reserved2 : 1;
      uint8_t carrSoln : 2; // Carrier phase range solution status:
                            // 0: no carrier phase range solution
                            // 1: carrier phase range solution with floating ambiguities
                            // 2: carrier phase range solution with fixed ambiguities
    } bits;
  } flags2;
  uint32_t ttff; // Time to first fix (millisecond time tag): ms
  uint32_t msss; // Milliseconds since Startup / Reset: ms
} UBX_NAV_STATUS_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t gpsFix : 1;

      uint32_t gpsFixOk : 1;
      uint32_t diffSoln : 1;
      uint32_t wknSet : 1;
      uint32_t towSet : 1;

      uint32_t diffCorr : 1;
      uint32_t carrSolnValid : 1;
      uint32_t mapMatching : 1;

      uint32_t psmState : 1;
      uint32_t spoofDetState : 1;
      uint32_t carrSoln : 1;

      uint32_t ttff : 1;
      uint32_t msss : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_STATUS_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_STATUS_data_t data;
  UBX_NAV_STATUS_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_STATUS_data_t);
  void (*callbackPointerPtr)(UBX_NAV_STATUS_data_t *);
  UBX_NAV_STATUS_data_t *callbackData;
} UBX_NAV_STATUS_t;

// UBX-NAV-DOP (0x01 0x04): Dilution of precision
const uint16_t UBX_NAV_DOP_LEN = 18;

typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  uint16_t gDOP; // Geometric DOP: * 0.01
  uint16_t pDOP; // Position DOP: * 0.01
  uint16_t tDOP; // Time DOP: * 0.01
  uint16_t vDOP; // Vertical DOP: * 0.01
  uint16_t hDOP; // Horizontal DOP: * 0.01
  uint16_t nDOP; // Northing DOP: * 0.01
  uint16_t eDOP; // Easting DOP: * 0.01
} UBX_NAV_DOP_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t gDOP : 1;
      uint32_t pDOP : 1;
      uint32_t tDOP : 1;
      uint32_t vDOP : 1;
      uint32_t hDOP : 1;
      uint32_t nDOP : 1;
      uint32_t eDOP : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_DOP_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_DOP_data_t data;
  UBX_NAV_DOP_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_DOP_data_t);
  void (*callbackPointerPtr)(UBX_NAV_DOP_data_t *);
  UBX_NAV_DOP_data_t *callbackData;
} UBX_NAV_DOP_t;

// UBX-NAV-ATT (0x01 0x05): Attitude solution
const uint16_t UBX_NAV_ATT_LEN = 32;

typedef struct
{
  uint32_t iTOW;   // GPS time of week of the navigation epoch: ms
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1[3];
  int32_t roll;        // Vehicle roll: Degrees * 1e-5
  int32_t pitch;       // Vehicle pitch: Degrees * 1e-5
  int32_t heading;     // Vehicle heading: Degrees * 1e-5
  uint32_t accRoll;    // Vehicle roll accuracy (if null, roll angle is not available): Degrees * 1e-5
  uint32_t accPitch;   // Vehicle pitch accuracy (if null, roll angle is not available): Degrees * 1e-5
  uint32_t accHeading; // Vehicle heading accuracy (if null, roll angle is not available): Degrees * 1e-5
} UBX_NAV_ATT_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t version : 1;
      uint32_t roll : 1;
      uint32_t pitch : 1;
      uint32_t heading : 1;
      uint32_t accRoll : 1;
      uint32_t accPitch : 1;
      uint32_t accHeading : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_ATT_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_ATT_data_t data;
  UBX_NAV_ATT_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_ATT_data_t);
  void (*callbackPointerPtr)(UBX_NAV_ATT_data_t *);
  UBX_NAV_ATT_data_t *callbackData;
} UBX_NAV_ATT_t;

// UBX-NAV-PVT (0x01 0x07): Navigation position velocity time solution
const uint16_t UBX_NAV_PVT_LEN = 92;

typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  uint16_t year; // Year (UTC)
  uint8_t month; // Month, range 1..12 (UTC)
  uint8_t day;   // Day of month, range 1..31 (UTC)
  uint8_t hour;  // Hour of day, range 0..23 (UTC)
  uint8_t min;   // Minute of hour, range 0..59 (UTC)
  uint8_t sec;   // Seconds of minute, range 0..60 (UTC)
  union
  {
    uint8_t all;
    struct
    {
      uint8_t validDate : 1;     // 1 = valid UTC Date
      uint8_t validTime : 1;     // 1 = valid UTC time of day
      uint8_t fullyResolved : 1; // 1 = UTC time of day has been fully resolved (no seconds uncertainty).
      uint8_t validMag : 1;      // 1 = valid magnetic declination
    } bits;
  } valid;
  uint32_t tAcc;   // Time accuracy estimate (UTC): ns
  int32_t nano;    // Fraction of second, range -1e9 .. 1e9 (UTC): ns
  uint8_t fixType; // GNSSfix Type:
                   // 0: no fix
                   // 1: dead reckoning only
                   // 2: 2D-fix
                   // 3: 3D-fix
                   // 4: GNSS + dead reckoning combined
                   // 5: time only fix
  union
  {
    uint8_t all;
    struct
    {
      uint8_t gnssFixOK : 1; // 1 = valid fix (i.e within DOP & accuracy masks)
      uint8_t diffSoln : 1;  // 1 = differential corrections were applied
      uint8_t psmState : 3;
      uint8_t headVehValid : 1; // 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode
      uint8_t carrSoln : 2;     // Carrier phase range solution status:
                                // 0: no carrier phase range solution
                                // 1: carrier phase range solution with floating ambiguities
                                // 2: carrier phase range solution with fixed ambiguities
    } bits;
  } flags;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t reserved : 5;
      uint8_t confirmedAvai : 1; // 1 = information about UTC Date and Time of Day validity confirmation is available
      uint8_t confirmedDate : 1; // 1 = UTC Date validity could be confirmed
      uint8_t confirmedTime : 1; // 1 = UTC Time of Day could be confirmed
    } bits;
  } flags2;
  uint8_t numSV;    // Number of satellites used in Nav Solution
  int32_t lon;      // Longitude: deg * 1e-7
  int32_t lat;      // Latitude: deg * 1e-7
  int32_t height;   // Height above ellipsoid: mm
  int32_t hMSL;     // Height above mean sea level: mm
  uint32_t hAcc;    // Horizontal accuracy estimate: mm
  uint32_t vAcc;    // Vertical accuracy estimate: mm
  int32_t velN;     // NED north velocity: mm/s
  int32_t velE;     // NED east velocity: mm/s
  int32_t velD;     // NED down velocity: mm/s
  int32_t gSpeed;   // Ground Speed (2-D): mm/s
  int32_t headMot;  // Heading of motion (2-D): deg * 1e-5
  uint32_t sAcc;    // Speed accuracy estimate: mm/s
  uint32_t headAcc; // Heading accuracy estimate (both motion and vehicle): deg * 1e-5
  uint16_t pDOP;    // Position DOP * 0.01
  union
  {
    uint8_t all;
    struct
    {
      uint8_t invalidLlh : 1; // 1 = Invalid lon, lat, height and hMSL
    } bits;
  } flags3;
  uint8_t reserved1[5];
  int32_t headVeh; // Heading of vehicle (2-D): deg * 1e-5
  int16_t magDec;  // Magnetic declination: deg * 1e-2
  uint16_t magAcc; // Magnetic declination accuracy: deg * 1e-2
} UBX_NAV_PVT_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t year : 1;
      uint32_t month : 1;
      uint32_t day : 1;
      uint32_t hour : 1;
      uint32_t min : 1;
      uint32_t sec : 1;

      uint32_t validDate : 1;
      uint32_t validTime : 1;
      uint32_t fullyResolved : 1;
      uint32_t validMag : 1;

      uint32_t tAcc : 1;
      uint32_t nano : 1;
      uint32_t fixType : 1;
      uint32_t gnssFixOK : 1;
      uint32_t diffSoln : 1;
      uint32_t psmState : 1;
      uint32_t headVehValid : 1;
      uint32_t carrSoln : 1;

      uint32_t confirmedAvai : 1;
      uint32_t confirmedDate : 1;
      uint32_t confirmedTime : 1;

      uint32_t numSV : 1;
      uint32_t lon : 1;
      uint32_t lat : 1;
      uint32_t height : 1;
      uint32_t hMSL : 1;
      uint32_t hAcc : 1;
      uint32_t vAcc : 1;
      uint32_t velN : 1;
      uint32_t velE : 1;
    } bits;
  } moduleQueried1;
  union
  {
    uint32_t all;
    struct
    {
      uint32_t velD : 1;
      uint32_t gSpeed : 1;
      uint32_t headMot : 1;
      uint32_t sAcc : 1;
      uint32_t headAcc : 1;
      uint32_t pDOP : 1;

      uint32_t invalidLlh : 1;

      uint32_t headVeh : 1;
      uint32_t magDec : 1;
      uint32_t magAcc : 1;
    } bits;
  } moduleQueried2;
} UBX_NAV_PVT_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_PVT_data_t data;
  UBX_NAV_PVT_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_PVT_data_t);
  void (*callbackPointerPtr)(UBX_NAV_PVT_data_t *);
  UBX_NAV_PVT_data_t *callbackData;
} UBX_NAV_PVT_t;

// UBX-NAV-ODO (0x01 0x09): Odometer solution
const uint16_t UBX_NAV_ODO_LEN = 20;

typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1[3];
  uint32_t iTOW;          // GPS time of week of the navigation epoch: ms
  uint32_t distance;      // Ground distance since last reset: m
  uint32_t totalDistance; // Total cumulative ground distance: m
  uint32_t distanceStd;   // Ground distance accuracy (1-sigma): m
} UBX_NAV_ODO_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t version : 1;
      uint32_t iTOW : 1;
      uint32_t distance : 1;
      uint32_t totalDistance : 1;
      uint32_t distanceStd : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_ODO_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_ODO_data_t data;
  UBX_NAV_ODO_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_ODO_data_t);
  void (*callbackPointerPtr)(UBX_NAV_ODO_data_t *);
  UBX_NAV_ODO_data_t *callbackData;
} UBX_NAV_ODO_t;

// UBX-NAV-VELECEF (0x01 0x11): Velocity solution in ECEF
const uint16_t UBX_NAV_VELECEF_LEN = 20;

typedef struct
{
  uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
  int32_t ecefVX; // ECEF X velocity: cm/s
  int32_t ecefVY; // ECEF Y velocity: cm/s
  int32_t ecefVZ; // ECEF Z velocity: cm/s
  uint32_t sAcc;  // Speed accuracy estimate: cm/s
} UBX_NAV_VELECEF_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t ecefVX : 1;
      uint32_t ecefVY : 1;
      uint32_t ecefVZ : 1;
      uint32_t sAcc : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_VELECEF_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_VELECEF_data_t data;
  UBX_NAV_VELECEF_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_VELECEF_data_t);
  void (*callbackPointerPtr)(UBX_NAV_VELECEF_data_t *);
  UBX_NAV_VELECEF_data_t *callbackData;
} UBX_NAV_VELECEF_t;

// UBX-NAV-VELNED (0x01 0x12): Velocity solution in NED frame
const uint16_t UBX_NAV_VELNED_LEN = 36;

typedef struct
{
  uint32_t iTOW;   // GPS time of week of the navigation epoch: ms
  int32_t velN;    // North velocity component: cm/s
  int32_t velE;    // East velocity component: cm/s
  int32_t velD;    // Down velocity component: cm/s
  uint32_t speed;  // Speed (3-D): cm/s
  uint32_t gSpeed; // Ground Speed (2-D): cm/s
  int32_t heading; // Heading of motion 2-D: Degrees * 1e-5
  uint32_t sAcc;   // Speed accuracy estimate: cm/s
  uint32_t cAcc;   // Course/Heading accuracy estimate: Degrees * 1e-5
} UBX_NAV_VELNED_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t velN : 1;
      uint32_t velE : 1;
      uint32_t velD : 1;
      uint32_t speed : 1;
      uint32_t gSpeed : 1;
      uint32_t heading : 1;
      uint32_t sAcc : 1;
      uint32_t cAcc : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_VELNED_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_VELNED_data_t data;
  UBX_NAV_VELNED_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_VELNED_data_t);
  void (*callbackPointerPtr)(UBX_NAV_VELNED_data_t *);
  UBX_NAV_VELNED_data_t *callbackData;
} UBX_NAV_VELNED_t;

// UBX-NAV-HPPOSECEF (0x01 0x13): High precision position solution in ECEF
const uint16_t UBX_NAV_HPPOSECEF_LEN = 28;

typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1[3];
  uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
  int32_t ecefX;  // ECEF X coordinate: cm
  int32_t ecefY;  // ECEF Y coordinate: cm
  int32_t ecefZ;  // ECEF Z coordinate: cm
  int8_t ecefXHp; // High precision component of ECEF X coordinate: mm * 0.1
  int8_t ecefYHp; // High precision component of ECEF Y coordinate: mm * 0.1
  int8_t ecefZHp; // High precision component of ECEF Z coordinate: mm * 0.1
  union
  {
    uint8_t all;
    struct
    {
      uint8_t invalidEcef : 1; // 1 = Invalid ecefX, ecefY, ecefZ, ecefXHp, ecefYHp and ecefZHp
    } bits;
  } flags;
  uint32_t pAcc; // Position Accuracy Estimate: mm * 0.1
} UBX_NAV_HPPOSECEF_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t version : 1;
      uint32_t iTOW : 1;
      uint32_t ecefX : 1;
      uint32_t ecefY : 1;
      uint32_t ecefZ : 1;
      uint32_t ecefXHp : 1;
      uint32_t ecefYHp : 1;
      uint32_t ecefZHp : 1;

      uint32_t invalidEcef : 1;

      uint32_t pAcc : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_HPPOSECEF_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_HPPOSECEF_data_t data;
  UBX_NAV_HPPOSECEF_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_HPPOSECEF_data_t);
  void (*callbackPointerPtr)(UBX_NAV_HPPOSECEF_data_t *);
  UBX_NAV_HPPOSECEF_data_t *callbackData;
} UBX_NAV_HPPOSECEF_t;

// UBX-NAV-HPPOSLLH (0x01 0x14): High precision geodetic position solution
const uint16_t UBX_NAV_HPPOSLLH_LEN = 36;

typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1[2];
  union
  {
    uint8_t all;
    struct
    {
      uint8_t invalidLlh : 1; // 1 = Invalid lon, lat, height, hMSL, lonHp, latHp, heightHp and hMSLHp
    } bits;
  } flags;
  uint32_t iTOW;   // GPS time of week of the navigation epoch: ms
  int32_t lon;     // Longitude: deg * 1e-7
  int32_t lat;     // Latitude: deg * 1e-7
  int32_t height;  // Height above ellipsoid: mm
  int32_t hMSL;    // Height above mean sea level: mm
  int8_t lonHp;    // High precision component of longitude: deg * 1e-9
  int8_t latHp;    // High precision component of latitude: deg * 1e-9
  int8_t heightHp; // High precision component of height above ellipsoid: mm * 0.1
  int8_t hMSLHp;   // High precision component of height above mean sea level: mm * 0.1
  uint32_t hAcc;   // Horizontal accuracy estimate: mm * 0.1
  uint32_t vAcc;   // Vertical accuracy estimate: mm * 0.1
} UBX_NAV_HPPOSLLH_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t version : 1;

      uint32_t invalidLlh : 1;

      uint32_t iTOW : 1;
      uint32_t lon : 1;
      uint32_t lat : 1;
      uint32_t height : 1;
      uint32_t hMSL : 1;
      uint32_t lonHp : 1;
      uint32_t latHp : 1;
      uint32_t heightHp : 1;
      uint32_t hMSLHp : 1;
      uint32_t hAcc : 1;
      uint32_t vAcc : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_HPPOSLLH_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_HPPOSLLH_data_t data;
  UBX_NAV_HPPOSLLH_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_HPPOSLLH_data_t);
  void (*callbackPointerPtr)(UBX_NAV_HPPOSLLH_data_t *);
  UBX_NAV_HPPOSLLH_data_t *callbackData;
} UBX_NAV_HPPOSLLH_t;

// UBX-NAV-PVAT (0x01 0x17): Navigation position velocity attitude time solution
const uint16_t UBX_NAV_PVAT_LEN = 116;

typedef struct
{
  uint32_t iTOW;   // GPS time of week of the navigation epoch: ms
  uint8_t version; // Message version (0x00 for this version)
  union
  {
    uint8_t all;
    struct
    {
      uint8_t validDate : 1;     // 1 = valid UTC Date
      uint8_t validTime : 1;     // 1 = valid UTC time of day
      uint8_t fullyResolved : 1; // 1 = UTC time of day has been fully resolved (no seconds uncertainty).
      uint8_t validMag : 1;      // 1 = valid magnetic declination
    } bits;
  } valid;
  uint16_t year; // Year (UTC)
  uint8_t month; // Month, range 1..12 (UTC)
  uint8_t day;   // Day of month, range 1..31 (UTC)
  uint8_t hour;  // Hour of day, range 0..23 (UTC)
  uint8_t min;   // Minute of hour, range 0..59 (UTC)
  uint8_t sec;   // Seconds of minute, range 0..60 (UTC)
  uint8_t reserved0;
  uint8_t reserved1[2];
  uint32_t tAcc;   // Time accuracy estimate (UTC): ns
  int32_t nano;    // Fraction of second, range -1e9 .. 1e9 (UTC): ns
  uint8_t fixType; // GNSSfix Type:
                   // 0: no fix
                   // 1: dead reckoning only
                   // 2: 2D-fix
                   // 3: 3D-fix
                   // 4: GNSS + dead reckoning combined
                   // 5: time only fix
  union
  {
    uint8_t all;
    struct
    {
      uint8_t gnssFixOK : 1; // 1 = valid fix (i.e within DOP & accuracy masks)
      uint8_t diffSoln : 1;  // 1 = differential corrections were applied
      uint8_t reserved : 1;
      uint8_t vehRollValid : 1;    // 1 = roll of vehicle is valid, only set if the receiver is in sensor fusion mode
      uint8_t vehPitchValid : 1;   // 1 = pitch of vehicle is valid, only set if the receiver is in sensor fusion mode
      uint8_t vehHeadingValid : 1; // 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode
      uint8_t carrSoln : 2;        // Carrier phase range solution status:
                                   // 0: no carrier phase range solution
                                   // 1: carrier phase range solution with floating ambiguities
                                   // 2: carrier phase range solution with fixed ambiguities
    } bits;
  } flags;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t reserved : 5;
      uint8_t confirmedAvai : 1; // 1 = information about UTC Date and Time of Day validity confirmation is available
      uint8_t confirmedDate : 1; // 1 = UTC Date validity could be confirmed
      uint8_t confirmedTime : 1; // 1 = UTC Time of Day could be confirmed
    } bits;
  } flags2;
  uint8_t numSV;             // Number of satellites used in Nav Solution
  int32_t lon;               // Longitude: deg * 1e-7
  int32_t lat;               // Latitude: deg * 1e-7
  int32_t height;            // Height above ellipsoid: mm
  int32_t hMSL;              // Height above mean sea level: mm
  uint32_t hAcc;             // Horizontal accuracy estimate: mm
  uint32_t vAcc;             // Vertical accuracy estimate: mm
  int32_t velN;              // NED north velocity: mm/s
  int32_t velE;              // NED east velocity: mm/s
  int32_t velD;              // NED down velocity: mm/s
  int32_t gSpeed;            // Ground Speed (2-D): mm/s
  uint32_t sAcc;             // Speed accuracy estimate: mm/s
  int32_t vehRoll;           // Vehicle roll: 1e-5 deg
  int32_t vehPitch;          // Vehicle pitch: 1e-5 deg
  int32_t vehHeading;        // Vehicle heading: 1e-5 deg
  int32_t motHeading;        // Motion heading.: 1e-5 deg
  uint16_t accRoll;          // Vehicle roll accuracy (if null, roll angle is not available): 1e-2 deg
  uint16_t accPitch;         // Vehicle pitch accuracy (if null, pitch angle is not available): 1e-2 deg
  uint16_t accHeading;       // Vehicle heading accuracy (if null, heading angle is not available): 1e-2 deg
  int16_t magDec;            // Magnetic declination: 1e-2 deg
  uint16_t magAcc;           // Magnetic declination accuracy: 1e-2 deg
  uint16_t errEllipseOrient; // Orientation of semi-major axis of error ellipse (degrees from true north): 1e-2 deg
  uint32_t errEllipseMajor;  // Semi-major axis of error ellipse: mm
  uint32_t errEllipseMinor;  // Semi-minor axis of error ellipse: mm
  uint8_t reserved2[4];
  uint8_t reserved3[4];
} UBX_NAV_PVAT_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t version : 1;

      uint32_t validDate : 1;
      uint32_t validTime : 1;
      uint32_t fullyResolved : 1;
      uint32_t validMag : 1;

      uint32_t year : 1;
      uint32_t month : 1;
      uint32_t day : 1;
      uint32_t hour : 1;
      uint32_t min : 1;
      uint32_t sec : 1;

      uint32_t tAcc : 1;
      uint32_t nano : 1;
      uint32_t fixType : 1;

      uint32_t gnssFixOK : 1;
      uint32_t diffSoln : 1;
      uint32_t vehRollValid : 1;
      uint32_t vehPitchValid : 1;
      uint32_t vehHeadingValid : 1;
      uint32_t carrSoln : 1;

      uint32_t confirmedAvai : 1;
      uint32_t confirmedDate : 1;
      uint32_t confirmedTime : 1;

      uint32_t numSV : 1;
      uint32_t lon : 1;
      uint32_t lat : 1;
      uint32_t height : 1;
      uint32_t hMSL : 1;
      uint32_t hAcc : 1;
      uint32_t vAcc : 1;
    } bits;
  } moduleQueried1;
  union
  {
    uint32_t all;
    struct
    {
      uint32_t velN : 1;
      uint32_t velE : 1;
      uint32_t velD : 1;
      uint32_t gSpeed : 1;
      uint32_t sAcc : 1;
      uint32_t vehRoll : 1;
      uint32_t vehPitch : 1;
      uint32_t vehHeading : 1;
      uint32_t motHeading : 1;
      uint32_t accRoll : 1;
      uint32_t accPitch : 1;
      uint32_t accHeading : 1;
      uint32_t magDec : 1;
      uint32_t magAcc : 1;
      uint32_t errEllipseOrient : 1;
      uint32_t errEllipseMajor : 1;
      uint32_t errEllipseMinor : 1;
    } bits;
  } moduleQueried2;
} UBX_NAV_PVAT_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_PVAT_data_t data;
  UBX_NAV_PVAT_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_PVAT_data_t);
  void (*callbackPointerPtr)(UBX_NAV_PVAT_data_t *);
  UBX_NAV_PVAT_data_t *callbackData;
} UBX_NAV_PVAT_t;

// UBX-NAV-TIMEUTC (0x01 0x21): UTC time solution
const uint16_t UBX_NAV_TIMEUTC_LEN = 20;

typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  uint32_t tAcc; // Time accuracy estimate (UTC): ns
  int32_t nano;  // Fraction of second, range -1e9 .. 1e9 (UTC): ns
  uint16_t year; // Year (UTC)
  uint8_t month; // Month, range 1..12 (UTC)
  uint8_t day;   // Day of month, range 1..31 (UTC)
  uint8_t hour;  // Hour of day, range 0..23 (UTC)
  uint8_t min;   // Minute of hour, range 0..59 (UTC)
  uint8_t sec;   // Seconds of minute, range 0..60 (UTC)
  union
  {
    uint8_t all;
    struct
    {
      uint8_t validTOW : 1; // 1 = Valid Time of Week
      uint8_t validWKN : 1; // 1 = Valid Week Number
      uint8_t validUTC : 1; // 1 = Valid UTC Time
      uint8_t reserved : 1;
      uint8_t utcStandard : 4; // UTC standard identifier
    } bits;
  } valid;
} UBX_NAV_TIMEUTC_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t tAcc : 1;
      uint32_t nano : 1;
      uint32_t year : 1;
      uint32_t month : 1;
      uint32_t day : 1;
      uint32_t hour : 1;
      uint32_t min : 1;
      uint32_t sec : 1;

      uint32_t validTOW : 1;
      uint32_t validWKN : 1;
      uint32_t validUTC : 1;
      uint32_t utcStandard : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_TIMEUTC_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_TIMEUTC_data_t data;
  UBX_NAV_TIMEUTC_moduleQueried_t moduleQueried;
  void (*callbackPointerPtr)(UBX_NAV_TIMEUTC_data_t *);
  UBX_NAV_TIMEUTC_data_t *callbackData;
} UBX_NAV_TIMEUTC_t;

// UBX-NAV-CLOCK (0x01 0x22): Clock solution
const uint16_t UBX_NAV_CLOCK_LEN = 20;

typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  int32_t clkB;  // Clock bias: ns
  int32_t clkD;  // Clock drift: ns/s
  uint32_t tAcc; // Time accuracy estimate: ns
  uint32_t fAcc; // Frequency accuracy estimate: ps/s
} UBX_NAV_CLOCK_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t clkB : 1;
      uint32_t clkD : 1;
      uint32_t tAcc : 1;
      uint32_t fAcc : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_CLOCK_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_CLOCK_data_t data;
  UBX_NAV_CLOCK_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_CLOCK_data_t);
  void (*callbackPointerPtr)(UBX_NAV_CLOCK_data_t *);
  UBX_NAV_CLOCK_data_t *callbackData;
} UBX_NAV_CLOCK_t;

// UBX-NAV-TIMELS (0x01 0x26): Leap second event information
const uint16_t UBX_NAV_TIMELS_LEN = 24;

typedef struct
{
  uint32_t iTOW;   // GPS time of week of the navigation epoch: ms
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1[3];
  uint8_t srcOfCurrLs;    // Information source for the current number of leap seconds
  int8_t currLs;          // Current number of leap seconds since start of GPS (Jan 6, 1980), s
  uint8_t srcOfLsChange;  // Information source for the future leap second event
  int8_t lsChange;        // Future leap second change if one is scheduled, +1, 0, -1s
  int32_t timeToLsEvent;  // Num of secs until the next or from the last leap second, s
  uint16_t dateOfLsGpsWn; // GPS week num (WN) of the next or the last leap second event
  uint16_t dateOfLsGpsDn; // GPS day of week num (DN) for the next or last leap second event
  uint8_t reserved2[3];
  union
  {
    uint8_t all;
    struct
    {
      uint8_t validCurrLs : 1;        // 1 = Valid current number of leap seconds value
      uint8_t validTimeToLsEvent : 1; // 1 = Valid time to next leap second event or from the last leap second event if no future event scheduled
    } bits;
  } valid;
} UBX_NAV_TIMELS_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t version : 1;
      uint32_t srcOfCurrLs : 1;
      uint32_t currLs : 1;
      uint32_t srcOfLsChange : 1;
      uint32_t lsChange : 1;
      uint32_t timeToLsEvent : 1;
      uint32_t dateOfLsGpsWn : 1;
      uint32_t dateOfLsGpsDn : 1;
      uint32_t validCurrLs : 1;
      uint32_t validTimeToLsEvent : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_TIMELS_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_TIMELS_data_t data;
  UBX_NAV_TIMELS_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_TIMELS_data_t);
  void (*callbackPointerPtr)(UBX_NAV_TIMELS_data_t *);
  UBX_NAV_TIMELS_data_t *callbackData;
} UBX_NAV_TIMELS_t;

// UBX-NAV-SAT (0x01 0x35): Satellite Information
const uint16_t UBX_NAV_SAT_MAX_BLOCKS = 255; // numSvs is 8-bit
const uint16_t UBX_NAV_SAT_MAX_LEN = 8 + (12 * UBX_NAV_SAT_MAX_BLOCKS);

typedef struct
{
  uint32_t iTOW;   // GPS time of week
  uint8_t version; // Message version (0x01 for this version)
  uint8_t numSvs;  // Number of satellites
  uint8_t reserved1[2];
} UBX_NAV_SAT_header_t;

typedef struct
{
  uint8_t gnssId; // GNSS identifier
  uint8_t svId;   // Satellite identifier
  uint8_t cno;    // Carrier-to-noise density ratio: dB-Hz
  int8_t elev;    // Elevation (range: +/-90): deg
  int16_t azim;   // Azimuth (range 0-360): deg
  int16_t prRes;  // Pseudorange residual: m * 0.1
  union
  {
    uint32_t all;
    struct
    {
      uint32_t qualityInd : 3;  // Signal quality indicator: 0: no signal
                                // 1: searching signal
                                // 2: signal acquired
                                // 3: signal detected but unusable
                                // 4: code locked and time synchronized
                                // 5, 6, 7: code and carrier locked and time synchronized
      uint32_t svUsed : 1;      // 1 = Signal in the subset specified in Signal Identifiers is currently being used for navigation
      uint32_t health : 2;      // Signal health flag: 0: unknown  1: healthy  2: unhealthy
      uint32_t diffCorr : 1;    // 1 = differential correction data is available for this SV
      uint32_t smoothed : 1;    // 1 = carrier smoothed pseudorange used
      uint32_t orbitSource : 3; // Orbit source: 0: no orbit information is available for this SV
                                // 1: ephemeris is used
                                // 2: almanac is used
                                // 3: AssistNow Offline orbit is used
                                // 4: AssistNow Autonomous orbit is used
                                // 5, 6, 7: other orbit information is used
      uint32_t ephAvail : 1;    // 1 = ephemeris is available for this SV
      uint32_t almAvail : 1;    // 1 = almanac is available for this SV
      uint32_t anoAvail : 1;    // 1 = AssistNow Offline data is available for this SV
      uint32_t aopAvail : 1;    // 1 = AssistNow Autonomous data is available for this SV
      uint32_t reserved1 : 1;
      uint32_t sbasCorrUsed : 1;   // 1 = SBAS corrections have been used for a signal in the subset specified in Signal Identifiers
      uint32_t rtcmCorrUsed : 1;   // 1 = RTCM corrections have been used for a signal in the subset specified in Signal Identifiers
      uint32_t slasCorrUsed : 1;   // 1 = QZSS SLAS corrections have been used for a signal in the subset specified in Signal Identifiers
      uint32_t spartnCorrUsed : 1; // 1 = SPARTN corrections have been used for a signal in the subset specified in Signal Identifiers
      uint32_t prCorrUsed : 1;     // 1 = Pseudorange corrections have been used for a signal in the subset specified in Signal Identifiers
      uint32_t crCorrUsed : 1;     // 1 = Carrier range corrections have been used for a signal in the subset specified in Signal Identifiers
      uint32_t doCorrUsed : 1;     // 1 = Range rate (Doppler) corrections have been used for a signal in the subset specified in Signal Identifiers
      uint32_t reserved2 : 9;
    } bits;
  } flags;
} UBX_NAV_SAT_block_t;

typedef struct
{
  UBX_NAV_SAT_header_t header;
  UBX_NAV_SAT_block_t blocks[UBX_NAV_SAT_MAX_BLOCKS];
} UBX_NAV_SAT_data_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_SAT_data_t data;
  bool moduleQueried;
  void (*callbackPointer)(UBX_NAV_SAT_data_t);
  void (*callbackPointerPtr)(UBX_NAV_SAT_data_t *);
  UBX_NAV_SAT_data_t *callbackData;
} UBX_NAV_SAT_t;

// UBX-NAV-SVIN (0x01 0x3B): Survey-in data
const uint16_t UBX_NAV_SVIN_LEN = 40;

typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1[3];
  uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
  uint32_t dur;   // Passed survey-in observation time: s
  int32_t meanX;  // Current survey-in mean position ECEF X coordinate: cm
  int32_t meanY;  // Current survey-in mean position ECEF Y coordinate: cm
  int32_t meanZ;  // Current survey-in mean position ECEF Z coordinate: cm
  int8_t meanXHP; // Current high-precision survey-in mean position ECEF X coordinate: mm * 0.1
  int8_t meanYHP; // Current high-precision survey-in mean position ECEF Y coordinate: mm * 0.1
  int8_t meanZHP; // Current high-precision survey-in mean position ECEF Z coordinate: mm * 0.1
  uint8_t reserved2;
  uint32_t meanAcc; // Current survey-in mean position accuracy: mm * 0.1
  uint32_t obs;     // Number of position observations used during survey-in
  int8_t valid;     // Survey-in position validity flag, 1 = valid, otherwise 0
  int8_t active;    // Survey-in in progress flag, 1 = in-progress, otherwise 0
  uint8_t reserved3[2];
} UBX_NAV_SVIN_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t version : 1;
      uint32_t iTOW : 1;
      uint32_t dur : 1;
      uint32_t meanX : 1;
      uint32_t meanY : 1;
      uint32_t meanZ : 1;
      uint32_t meanXHP : 1;
      uint32_t meanYHP : 1;
      uint32_t meanZHP : 1;
      uint32_t meanAcc : 1;
      uint32_t obs : 1;
      uint32_t valid : 1;
      uint32_t active : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_SVIN_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_SVIN_data_t data;
  UBX_NAV_SVIN_moduleQueried_t moduleQueried;
  void (*callbackPointerPtr)(UBX_NAV_SVIN_data_t *);
  UBX_NAV_SVIN_data_t *callbackData;
} UBX_NAV_SVIN_t;

// UBX-NAV-RELPOSNED (0x01 0x3C): Relative positioning information in NED frame
// Note:
//  RELPOSNED on the M8 is only 40 bytes long
//  RELPOSNED on the F9 is 64 bytes long and contains much more information
const uint16_t UBX_NAV_RELPOSNED_LEN = 40;
const uint16_t UBX_NAV_RELPOSNED_LEN_F9 = 64;

typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved0;
  uint16_t refStationId; // Reference Station ID
  uint32_t iTOW;         // GPS time of week of the navigation epoch: ms
  int32_t relPosN;       // North component of relative position vector: cm
  int32_t relPosE;       // East component of relative position vector: cm
  int32_t relPosD;       // Down component of relative position vector: cm
  int32_t relPosLength;  // Length of the relative position vector: cm
  int32_t relPosHeading; // Heading of the relative position vector: Degrees * 1e-5
  uint8_t reserved1[4];
  int8_t relPosHPN;      // High-precision North component of relative position vector: mm * 0.1
  int8_t relPosHPE;      // High-precision East component of relative position vector: mm * 0.1
  int8_t relPosHPD;      // High-precision Down component of relative position vector: mm * 0.1
  int8_t relPosHPLength; // High-precision component of the length of the relative position vector: mm * 0.1
  uint32_t accN;         // Accuracy of relative position North component: mm * 0.1
  uint32_t accE;         // Accuracy of relative position East component: mm * 0.1
  uint32_t accD;         // Accuracy of relative position Down component: mm * 0.1
  uint32_t accLength;    // Accuracy of length of the relative position vector: mm * 0.1
  uint32_t accHeading;   // Accuracy of heading of the relative position vector: Degrees * 1e-5
  uint8_t reserved2[4];
  union
  {
    uint32_t all;
    struct
    {
      uint32_t gnssFixOK : 1;          // A valid fix (i.e within DOP & accuracy masks)
      uint32_t diffSoln : 1;           // 1 if differential corrections were applied
      uint32_t relPosValid : 1;        // 1 if relative position components and accuracies are valid
      uint32_t carrSoln : 2;           // Carrier phase range solution status:
                                       // 0 = no carrier phase range solution
                                       // 1 = carrier phase range solution with floating ambiguities
                                       // 2 = carrier phase range solution with fixed ambiguities
      uint32_t isMoving : 1;           // 1 if the receiver is operating in moving baseline mode
      uint32_t refPosMiss : 1;         // 1 if extrapolated reference position was used to compute moving baseline solution this epoch
      uint32_t refObsMiss : 1;         // 1 if extrapolated reference observations were used to compute moving baseline solution this epoch
      uint32_t relPosHeadingValid : 1; // 1 if relPosHeading is valid
      uint32_t relPosNormalized : 1;   // 1 if the components of the relative position vector (including the high-precision parts) are normalized
    } bits;
  } flags;
} UBX_NAV_RELPOSNED_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t version : 1;
      uint32_t refStationId : 1;
      uint32_t iTOW : 1;
      uint32_t relPosN : 1;
      uint32_t relPosE : 1;
      uint32_t relPosD : 1;
      uint32_t relPosLength : 1;
      uint32_t relPosHeading : 1;
      uint32_t relPosHPN : 1;
      uint32_t relPosHPE : 1;
      uint32_t relPosHPD : 1;
      uint32_t relPosHPLength : 1;
      uint32_t accN : 1;
      uint32_t accE : 1;
      uint32_t accD : 1;
      uint32_t accLength : 1;
      uint32_t accHeading : 1;

      uint32_t gnssFixOK : 1;
      uint32_t diffSoln : 1;
      uint32_t relPosValid : 1;
      uint32_t carrSoln : 1;
      uint32_t isMoving : 1;
      uint32_t refPosMiss : 1;
      uint32_t refObsMiss : 1;
      uint32_t relPosHeadingValid : 1;
      uint32_t relPosNormalized : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_RELPOSNED_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_RELPOSNED_data_t data;
  UBX_NAV_RELPOSNED_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_RELPOSNED_data_t);
  void (*callbackPointerPtr)(UBX_NAV_RELPOSNED_data_t *);
  UBX_NAV_RELPOSNED_data_t *callbackData;
} UBX_NAV_RELPOSNED_t;

// UBX-NAV-AOPSTATUS (0x01 0x60): AssistNow Autonomous status
const uint16_t UBX_NAV_AOPSTATUS_LEN = 16;

typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  union
  {
    uint8_t all;
    struct
    {
      uint8_t useAOP : 1; // AOP enabled flag
    } bits;
  } aopCfg;       // AssistNow Autonomous configuration
  uint8_t status; // AssistNow Autonomous subsystem is idle (0) or running (not 0)
  uint8_t reserved1[10];
} UBX_NAV_AOPSTATUS_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;

      uint32_t useAOP : 1;

      uint32_t status : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_AOPSTATUS_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_AOPSTATUS_data_t data;
  UBX_NAV_AOPSTATUS_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_NAV_AOPSTATUS_data_t);
  void (*callbackPointerPtr)(UBX_NAV_AOPSTATUS_data_t *);
  UBX_NAV_AOPSTATUS_data_t *callbackData;
} UBX_NAV_AOPSTATUS_t;

// UBX-NAV-EOE (0x01 0x61): End of Epoch
const uint16_t UBX_NAV_EOE_LEN = 4;

typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
} UBX_NAV_EOE_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
    } bits;
  } moduleQueried;
} UBX_NAV_EOE_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_NAV_EOE_data_t data;
  UBX_NAV_EOE_moduleQueried_t moduleQueried;
  void (*callbackPointerPtr)(UBX_NAV_EOE_data_t *);
  UBX_NAV_EOE_data_t  *callbackData;
} UBX_NAV_EOE_t;

// RXM-specific structs

// UBX-RXM-SFRBX (0x02 0x13): Broadcast navigation data subframe
// Note: length is variable
// Note: on protocol version 17: numWords is (0..16)
//       on protocol version 18+: numWords is (0..10)
const uint8_t UBX_RXM_SFRBX_MAX_WORDS = 16;
const uint16_t UBX_RXM_SFRBX_MAX_LEN = 8 + (4 * UBX_RXM_SFRBX_MAX_WORDS);

typedef struct
{
  uint8_t gnssId; // GNSS identifier
  uint8_t svId;   // Satellite identifier
  uint8_t reserved1;
  uint8_t freqId;   // GLONASS frequency slot
  uint8_t numWords; // The number of data words contained in this message (0..16)
  uint8_t chn;      // The tracking channel number the message was received on
  uint8_t version;  // Message version (0x01 for this version)
  uint8_t reserved2;
  uint32_t dwrd[UBX_RXM_SFRBX_MAX_WORDS]; // The data words
} UBX_RXM_SFRBX_data_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_RXM_SFRBX_data_t data;
  bool moduleQueried;
  void (*callbackPointer)(UBX_RXM_SFRBX_data_t);
  void (*callbackPointerPtr)(UBX_RXM_SFRBX_data_t *);
  UBX_RXM_SFRBX_data_t *callbackData;
} UBX_RXM_SFRBX_t;

// UBX-RXM-RAWX (0x02 0x15): Multi-GNSS raw measurement data
// Note: length is variable
const uint8_t UBX_RXM_RAWX_MAX_BLOCKS = 92; // See issue #70 and PR #74 for more info
const uint16_t UBX_RXM_RAWX_MAX_LEN = 16 + (32 * UBX_RXM_RAWX_MAX_BLOCKS);

typedef struct
{
  uint8_t rcvTow[8]; // Measurement time of week in receiver local time [64-bit float]
  uint16_t week;     // GPS week number
  int8_t leapS;      // GPS leap seconds
  uint8_t numMeas;   // Number of measurements to follow
  union
  {
    uint8_t all;
    struct
    {
      uint8_t leapSec : 1;  // Leap seconds have been determined
      uint8_t clkReset : 1; // Clock reset applied
    } bits;
  } recStat;
  uint8_t version; // Message version (0x01 for this version)
  uint8_t reserved1[2];
} UBX_RXM_RAWX_header_t;

typedef struct
{
  uint8_t prMes[8];  // Pseudorange measurement: m [64-bit float]
  uint8_t cpMes[8];  // Carrier phase measurement: cycles [64-bit float]
  uint8_t doMes[4];  // Doppler measurement: Hz [32-bit float]
  uint8_t gnssId;    // GNSS identifier
  uint8_t svId;      // Satellite identifier
  uint8_t sigId;     // New signal identifier
  uint8_t freqId;    // GLONASS frequency slot
  uint16_t lockTime; // Carrier phase locktime counter: ms
  uint8_t cno;       // Carrier-to-noise density ratio: dB-Hz
  uint8_t prStdev;   // Estimated pseudorange measurement standard deviation: m * 0.01 * 2^n [4-bit]
  uint8_t cpStdev;   // Estimated carrier phase measurement standard deviation: cycles * 0.004 [4-bit]
  uint8_t doStdev;   // Estimated Doppler measurement standard deviation: Hz * 0.002 * 2^n [4-bit]
  union
  {
    uint8_t all;
    struct
    {
      uint8_t prValid : 1;    // Pseudorange valid
      uint8_t cpValid : 1;    // Carrier phase valid
      uint8_t halfCyc : 1;    // Half cycle valid
      uint8_t subHalfCyc : 1; // Half cycle subtracted from phase
    } bits;
  } trkStat;
  uint8_t reserved2;
} UBX_RXM_RAWX_block_t;

typedef struct
{
  UBX_RXM_RAWX_header_t header;
  UBX_RXM_RAWX_block_t blocks[UBX_RXM_RAWX_MAX_BLOCKS];
} UBX_RXM_RAWX_data_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_RXM_RAWX_data_t data;
  bool moduleQueried;
  void (*callbackPointer)(UBX_RXM_RAWX_data_t);
  void (*callbackPointerPtr)(UBX_RXM_RAWX_data_t *);
  UBX_RXM_RAWX_data_t *callbackData;
} UBX_RXM_RAWX_t;

// UBX-RXM-COR (0x02 0x34): Differential correction input status
const uint16_t UBX_RXM_COR_LEN = 12;

typedef struct
{
  uint8_t version;      // Message version (0x01 for this version)
  uint8_t ebno;         // Energy per bit to noise power spectral density ratio (Eb/N0): 2^-3 dB
                        // 0: unknown. Reported only for protocol UBX-RXM-PMP (SPARTN) to monitor signal quality.
  uint8_t reserved0[2]; // Reserved
  union
  {
    uint32_t all;
    struct
    {
      uint32_t protocol : 5;        // Input correction data protocol:
                                    // 0: Unknown
                                    // 1: RTCM3
                                    // 2: SPARTN (Secure Position Augmentation for Real Time Navigation)
                                    // 29: UBX-RXM-PMP (SPARTN)
                                    // 30: UBX-RXM-QZSSL6
      uint32_t errStatus : 2;       // Error status of the received correction message content based on possibly available error codes or checksums:
                                    // 0: Unknown
                                    // 1: Error-free
                                    // 2: Erroneous
      uint32_t msgUsed : 2;         // Status of receiver using the input message:
                                    // 0: Unknown
                                    // 1: Not used
                                    // 2: Used
      uint32_t correctionId : 16;   // Identifier for the correction stream:
                                    // For RTCM 3: Reference station ID (DF003) of the received RTCM input message.
                                    // Valid range 0-4095.
                                    // For all other messages, reports 0xFFFF.
                                    // For other correction protocols 0xFFFF.
      uint32_t msgTypeValid : 1;    // Validity of the msgType field. Set to False e.g. if the protocol does not define msgType.
      uint32_t msgSubTypeValid : 1; // Validity of the msgSubType field. Set to False e.g. if the protocol does not define subtype for the msgType.
      uint32_t msgInputHandle : 1;  // Input handling support of the input message:
                                    // 0: Receiver does not have input handling support for this message
                                    // 1: Receiver has input handling support for this message
      uint32_t msgEncrypted : 2;    // Encryption status of the input message:
                                    // 0: Unknown
                                    // 1: Not encrypted
                                    // 2: Encrypted
      uint32_t msgDecrypted : 2;    // Decryption status of the input message:
                                    // 0: Unknown
                                    // 1: Not decrypted
                                    // 2: Successfully decrypted
    } bits;
  } statusInfo;
  uint16_t msgType;    // Message type
  uint16_t msgSubType; // Message subtype
} UBX_RXM_COR_data_t;

// The COR data can only be accessed via a callback. COR cannot be polled.
typedef struct
{
  ubxAutomaticFlags automaticFlags;
  void (*callbackPointerPtr)(UBX_RXM_COR_data_t *);
  UBX_RXM_COR_data_t *callbackData;
} UBX_RXM_COR_t;

// UBX-RXM-PMP (0x02 0x72): PMP raw data (D9 modules)
// There are two versions of this message but, fortunately, both have a max len of 528
const uint16_t UBX_RXM_PMP_MAX_USER_DATA = 504;
const uint16_t UBX_RXM_PMP_MAX_LEN = UBX_RXM_PMP_MAX_USER_DATA + 24;

typedef struct
{
  uint8_t version;             // Message version (0x00 / 0x01)
  uint8_t reserved0;           // Reserved
  uint16_t numBytesUserData;   // version 0x00: reserved0 ; version 0x01: Number of bytes the userData block has in this frame (0...504)
  uint32_t timeTag;            // Time since startup when frame started : ms
  uint32_t uniqueWord[2];      // Received unique words
  uint16_t serviceIdentifier;  // Received service identifier
  uint8_t spare;               // Received spare data
  uint8_t uniqueWordBitErrors; // Number of bit errors in both unique words

  // The position of fecBits, ebno and reserved1 depends on the message version
  uint16_t fecBits;  // Number of bits corrected by FEC (forward error correction)
  uint8_t ebno;      // Energy per bit to noise power spectral density ratio : 2^-3 dB
  uint8_t reserved1; // Reserved

  uint8_t userData[UBX_RXM_PMP_MAX_USER_DATA]; // Received user data: version 0x00 : starts at byte 20 ; version 0x01 : starts at byte 24

} UBX_RXM_PMP_data_t;

// The PMP data can only be accessed via a callback. PMP cannot be polled.
typedef struct
{
  ubxAutomaticFlags automaticFlags;
  void (*callbackPointerPtr)(UBX_RXM_PMP_data_t *);
  UBX_RXM_PMP_data_t *callbackData;
} UBX_RXM_PMP_t;

// Define a struct to hold the entire PMP message so the whole thing can be pushed to a GNSS.
// Remember that the length of the payload could be variable (with version 1 messages).
typedef struct
{
  uint8_t sync1; // 0xB5
  uint8_t sync2; // 0x62
  uint8_t cls;
  uint8_t ID;
  uint8_t lengthLSB;
  uint8_t lengthMSB;
  uint8_t payload[UBX_RXM_PMP_MAX_LEN];
  uint8_t checksumA;
  uint8_t checksumB;
} UBX_RXM_PMP_message_data_t;

// The PMP data can only be accessed via a callback. PMP cannot be polled.
typedef struct
{
  ubxAutomaticFlags automaticFlags;
  void (*callbackPointerPtr)(UBX_RXM_PMP_message_data_t *);
  UBX_RXM_PMP_message_data_t *callbackData;
} UBX_RXM_PMP_message_t;

// UBX-RXM-QZSSL6 (0x02 0x73): QZSS L6 raw data (D9C modules)
#define UBX_RXM_QZSSL6_NUM_CHANNELS 2
const uint16_t UBX_RXM_QZSSL6_DATALEN = 250;
const uint16_t UBX_RXM_QZSSL6_MAX_LEN = UBX_RXM_QZSSL6_DATALEN + 14;

typedef struct
{
  uint8_t version;        // Message version (0x00 / 0x01)
  uint8_t svId;           // Satellite identifier
  uint16_t cno;           // Mean C/N0
  uint32_t timeTag;       // Time since startup when frame started : ms
  uint8_t groupDelay;     // L6 group delay w.r.t. L2 on channel
  uint8_t bitErrCorr;     // Number of bit errors corrected by Reed-Solomon decoder
  uint16_t chInfo;        // Information about receiver channel associated with a received QZSS L6 message
  uint8_t reserved0[2];   // Reserved
  uint8_t msgBytes[UBX_RXM_QZSSL6_DATALEN];  // Bytes in a QZSS L6 message
} UBX_RXM_QZSSL6_data_t;

struct ubxQZSSL6AutomaticFlags
{
  union
  {
    uint8_t all;
    struct
    {
      uint8_t automatic : 1;         // Will this message be delivered and parsed "automatically" (without polling)
      uint8_t implicitUpdate : 1;    // Is the update triggered by accessing stale data (=true) or by a call to checkUblox (=false)
      uint8_t addToFileBuffer : 1;   // Should the raw UBX data be added to the file buffer?
      uint8_t callbackCopyValid : UBX_RXM_QZSSL6_NUM_CHANNELS; // Is the copies of the data structs used by the callback valid/fresh?
    } bits;
  } flags;
};

// Define a struct to hold the entire QZSSL6 message so the whole thing can be pushed to a GNSS.
// Remember that the length of the payload could be variable (with version 1 messages).
typedef struct
{
  uint8_t sync1; // 0xB5
  uint8_t sync2; // 0x62
  uint8_t cls;
  uint8_t ID;
  uint8_t lengthLSB;
  uint8_t lengthMSB;
  uint8_t payload[UBX_RXM_QZSSL6_MAX_LEN];
  uint8_t checksumA;
  uint8_t checksumB;
} UBX_RXM_QZSSL6_message_data_t;

// The QZSSL6 data can only be accessed via a callback. QZSSL6 cannot be polled.
typedef struct
{
  ubxQZSSL6AutomaticFlags automaticFlags;
  void (*callbackPointerPtr)(UBX_RXM_QZSSL6_message_data_t *);
  UBX_RXM_QZSSL6_message_data_t *callbackData;
} UBX_RXM_QZSSL6_message_t;

// CFG-specific structs

// UBX-CFG-PRT (0x06 0x00): Port configuration
// The content changes depending on which port type is being configured
// This struct defines the common structure
const uint16_t UBX_CFG_PRT_LEN = 20;

typedef struct
{
  uint8_t portID;    // Port identifier number
  uint8_t reserved0; // Reserved
  union
  {
    uint16_t all;
    struct
    {
      uint16_t en : 1;    // Enable TX ready feature for this port
      uint16_t pol : 1;   // Polarity: 0 High-active; 1 Low-active
      uint16_t pin : 5;   // PIO to be used (must not be in use by another function)
      uint16_t thres : 9; // Threshold
    } bits;
  } txReady;
  uint32_t mode;     // Content changes depending on the port type
  uint32_t baudRate; // Content changes depending on the port type
  union
  {
    uint16_t all;
    struct
    {
      uint16_t inUbx : 1;  // UBX protocol
      uint16_t inNmea : 1; // NMEA protocol
      uint16_t inRtcm : 1; // RTCM2 protocol
      uint16_t reserved : 2;
      uint16_t inRtcm3 : 1; // RTCM3 protocol (not supported for protocol versions less than 20.00)
      uint16_t inSPARTN : 1;
    } bits;
  } inProtoMask;
  union
  {
    uint16_t all;
    struct
    {
      uint16_t outUbx : 1;  // UBX protocol
      uint16_t outNmea : 1; // NMEA protocol
      uint16_t reserved : 3;
      uint16_t outRtcm3 : 1; // RTCM3 protocol (not supported for protocol versions less than 20.00)
      uint16_t outSPARTN : 1;
    } bits;
  } outProtoMask;
  uint16_t flags; // Content changes depending on the port type
  uint16_t reserved1;
} UBX_CFG_PRT_data_t;

typedef struct
{
  UBX_CFG_PRT_data_t data;
  bool dataValid;
} UBX_CFG_PRT_t;

// UBX-CFG-RATE (0x06 0x08): Navigation/measurement rate settings
const uint16_t UBX_CFG_RATE_LEN = 6;

typedef struct
{
  uint16_t measRate; // The elapsed time between GNSS measurements, which defines the rate: ms
  uint16_t navRate;  // The ratio between the number of measurements and the number of navigation solutions: cycles
  uint16_t timeRef;  // The time system to which measurements are aligned: 0: UTC; 1: GPS; 2: GLONASS; 3: BeiDou; 4: Galileo
} UBX_CFG_RATE_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t measRate : 1;
      uint32_t navRate : 1;
      uint32_t timeRef : 1;
    } bits;
  } moduleQueried;
} UBX_CFG_RATE_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_CFG_RATE_data_t data;
  UBX_CFG_RATE_moduleQueried_t moduleQueried;
} UBX_CFG_RATE_t;

// UBX-CFG-TP5 (0x06 0x31): Time pulse parameters
const uint16_t UBX_CFG_TP5_LEN = 32;

typedef struct
{
  uint8_t tpIdx;   // Time pulse selection (0 = TIMEPULSE, 1 = TIMEPULSE2)
  uint8_t version; // Message version (0x01 for this version)
  uint8_t reserved1[2];
  int16_t antCableDelay;      // Antenna cable delay: ns
  int16_t rfGroupDelay;       // RF group delay: ns
  uint32_t freqPeriod;        // Frequency or period time, depending on setting of bit 'isFreq': Hz_or_us
  uint32_t freqPeriodLock;    // Frequency or period time when locked to GNSS time, only used if 'lockedOtherSet' is set: Hz_or_us
  uint32_t pulseLenRatio;     // Pulse length or duty cycle, depending on 'isLength': us_or_2^-32
  uint32_t pulseLenRatioLock; // Pulse length or duty cycle when locked to GNSS time, only used if 'lockedOtherSet' is set: us_or_2^-32
  int32_t userConfigDelay;    // User-configurable time pulse delay: ns
  union
  {
    uint32_t all;
    struct
    {
      uint32_t active : 1;         // If set enable time pulse; if pin assigned to another function, other function takes precedence.
      uint32_t lockGnssFreq : 1;   // If set, synchronize time pulse to GNSS as soon as GNSS time is valid. If not set, or before GNSS time is valid, use local clock.
      uint32_t lockedOtherSet : 1; // If set the receiver switches between the timepulse settings given by 'freqPeriodLocked' & 'pulseLenLocked' and those given by 'freqPeriod' & 'pulseLen'.
      uint32_t isFreq : 1;         // If set 'freqPeriodLock' and 'freqPeriod' are interpreted as frequency, otherwise interpreted as period.
      uint32_t isLength : 1;       // If set 'pulseLenRatioLock' and 'pulseLenRatio' interpreted as pulse length, otherwise interpreted as duty cycle.
      uint32_t alignToTow : 1;     // Align pulse to top of second (period time must be integer fraction of 1s). Also set 'lockGnssFreq' to use this feature.
      uint32_t polarity : 1;       // Pulse polarity: 0: falling edge at top of second; 1: rising edge at top of second
      uint32_t gridUtcGnss : 4;    // Timegrid to use: 0: UTC; 1: GPS; 2: GLONASS; 3: BeiDou; 4: Galileo
      uint32_t syncMode : 3;       // Sync Manager lock mode to use:
                                   // 0: switch to 'freqPeriodLock' and 'pulseLenRatioLock' as soon as Sync Manager has an accurate time, never switch back to 'freqPeriod' and 'pulseLenRatio'
                                   // 1: switch to 'freqPeriodLock' and 'pulseLenRatioLock' as soon as Sync Manager has an accurate time, and switch back to 'freqPeriod' and 'pulseLenRatio' as soon as time gets inaccurate
    } bits;
  } flags;
} UBX_CFG_TP5_data_t;

// UBX-CFG-ITFM (0x06 0x39): Jamming/interference monitor configuration
const uint16_t UBX_CFG_ITFM_LEN = 8;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t bbThreshold : 4;    // Broadband jamming detection threshold
      uint32_t cwThreshold : 5;    // CW jamming detection threshold
      uint32_t algorithmBits : 22; // Reserved algorithm settings - should be set to 0x16B156 in hex for correct settings
      uint32_t enable : 1;         // Enable interference detection
    } bits;
  } config;
  union
  {
    uint32_t all;
    struct
    {
      uint32_t generalBits : 12; // General settings - should be set to 0x31E in hex for correct setting
      uint32_t antSetting : 2;   // Antenna setting, 0=unknown, 1=passive, 2=active
      uint32_t enable2 : 1;      // Set to 1 to scan auxiliary bands (u-blox 8 / u-blox M8 only, otherwise ignored)
    } bits;
  } config2;
} UBX_CFG_ITFM_data_t;

// UBX-CFG-TMODE3 (0x06 0x71): Time Mode Settings 3
const uint16_t UBX_CFG_TMODE3_LEN = 40;

typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t reserved1;
  union
  {
    uint16_t all;
    struct
    {
      uint16_t mode : 8; // Receiver Mode: 0 Disabled; 1 Survey In; 2 Fixed Mode (true ARP position information required); 3-255 Reserved
      uint16_t lla : 1;  // Position is given in LAT/LON/ALT (default is ECEF)
    } bits;
  } flags;
  int32_t ecefXOrLat;  // WGS84 ECEF X coordinate (or latitude) of the ARP position, depending on flags above: cm or deg*1e-7
  int32_t ecefYOrLon;  // WGS84 ECEF Y coordinate (or latitude) of the ARP position, depending on flags above: cm or deg*1e-7
  int32_t ecefZOrAlt;  // WGS84 ECEF Z coordinate (or altitude) of the ARP position, depending on flags above: cm
  int8_t ecefXOrLatHP; // High-precision WGS84 ECEF X coordinate (or latitude) of the ARP position, depending on flags above: 0.1 mm or deg*1e-9
  int8_t ecefYOrLonHP; // High-precision WGS84 ECEF Y coordinate (or longitude) of the ARP position, depending on flags above: 0.1 mm or deg*1e-9
  int8_t ecefZOrAltHP; // High-precision WGS84 ECEF Z coordinate (or altitude) of the ARP position, depending on flags above: 0.1 mm
  uint8_t reserved2;
  uint32_t fixedPosAcc;  // Fixed position 3D accuracy: 0.1 mm
  uint32_t svinMinDur;   // Survey-in minimum duration: s
  uint32_t svinAccLimit; // Survey-in position accuracy limit: 0.1 mm
  uint8_t reserved3[8];
} UBX_CFG_TMODE3_data_t;

// MON-specific structs

// UBX-MON-HW (0x0A 0x09): Hardware status
const uint16_t UBX_MON_HW_LEN = 60;

typedef struct
{
  uint32_t pinSel;     // Mask of pins set as peripheral/PIO
  uint32_t pinBank;    // Mask of pins set as bank A/B
  uint32_t pinDir;     // Mask of pins set as input/output
  uint32_t pinVal;     // Mask of pins value low/high
  uint16_t noisePerMS; // Noise level as measured by the GPS core
  uint16_t agcCnt;     // AGC monitor (counts SIGHI xor SIGLO, range 0 to 8191)
  uint8_t aStatus;     // Status of the antenna supervisor state machine (0=INIT, 1=DONTKNOW, 2=OK, 3=SHORT, 4=OPEN)
  uint8_t aPower;      // Current power status of antenna (0=OFF, 1=ON, 2=DONTKNOW)
  union
  {
    uint8_t all;
    struct
    {
      uint8_t rtcCalib : 1;     // RTC is calibrated
      uint8_t safeBoot : 1;     // Safeboot mode (0 = inactive, 1 = active)
      uint8_t jammingState : 2; // Output from jamming/interference monitor (0 = unknown or feature disabled,
                                // 1 = ok - no significant jamming,
                                // 2 = warning - interference visible but fix OK,
                                // 3 = critical - interference visible and no fix)
      uint8_t xtalAbsent : 1;   // RTC xtal has been determined to be absent
    } bits;
  } flags;
  uint8_t reserved1;    // Reserved
  uint32_t usedMask;    // Mask of pins that are used by the virtual pin manager
  uint8_t VP[17];       // Array of pin mappings for each of the 17 physical pins
  uint8_t jamInd;       // CW jamming indicator, scaled (0 = no CW jamming, 255 = strong CW jamming)
  uint8_t reserved2[2]; // Reserved
  uint32_t pinIrq;      // Mask of pins value using the PIO Irq
  uint32_t pullH;       // Mask of pins value using the PIO pull high resistor
  uint8_t pullL;        // Mask of pins value using the PIO pull low resistor
} UBX_MON_HW_data_t;

// UBX-MON-HW2 (0x0A 0x0B): Extended hardware status
const uint16_t UBX_MON_HW2_LEN = 28;

typedef struct
{
  int8_t ofsI;       // Imbalance of I-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance)
  uint8_t magI;      // Magnitude of I-part of complex signal, scaled (0 = no signal, 255 = max. magnitude)
  int8_t ofsQ;       // Imbalance of Q-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance)
  uint8_t magQ;      // Magnitude of Q-part of complex signal, scaled (0 = no signal, 255 = max. magnitude)
  uint8_t cfgSource; // Source of low-level configuration (114 = ROM, 111 = OTP, 112 = config pins, 102 = flash image)
  uint8_t reserved0[3];
  uint32_t lowLevCfg; // Low-level configuration (obsolete for protocol versions greater than 15.00)
  uint8_t reserved1[8];
  uint32_t postStatus;  // POST status word
  uint8_t reserved2[4]; // Reserved
} UBX_MON_HW2_data_t;

// UBX-MON-RF (0x0a 0x38): RF information
const uint16_t UBX_MON_RF_MAX_BLOCKS = 2; // 0 = L1; 1 = L2 / L5
const uint16_t UBX_MON_RF_MAX_LEN = 4 + (24 * UBX_MON_RF_MAX_BLOCKS);

typedef struct
{
  uint8_t version; // Message version (0x00 for this version)
  uint8_t nBlocks; // The number of RF blocks included
  uint8_t reserved0[2];
} UBX_MON_RF_header_t;

typedef struct
{
  uint8_t blockId; // RF block ID (0 = L1 band, 1 = L2 or L5 band depending on product configuration)
  union
  {
    uint8_t all;
    struct
    {
      uint8_t jammingState : 2; // output from Jamming/Interference Monitor (0 = unknown or feature disabled,
                                // 1 = ok - no significant jamming,
                                // 2 = warning - interference visible but fix OK,
                                // 3 = critical - interference visible and no fix)
    } bits;
  } flags;
  uint8_t antStatus;    // Status of the antenna supervisor state machine (0x00=INIT, 0x01=DONTKNOW, 0x02=OK, 0x03=SHORT, 0x04=OPEN)
  uint8_t antPower;     // Current power status of antenna (0x00=OFF, 0x01=ON, 0x02=DONTKNOW)
  uint32_t postStatus;  // POST status word
  uint8_t reserved1[4]; // Reserved
  uint16_t noisePerMS;  // Noise level as measured by the GPS core
  uint16_t agcCnt;      // AGC Monitor (counts SIGHI xor SIGLO, range 0 to 8191)
  uint8_t jamInd;       // CW jamming indicator, scaled (0=no CW jamming, 255 = strong CW jamming)
  int8_t ofsI;          // Imbalance of I-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance)
  uint8_t magI;         // Magnitude of I-part of complex signal, scaled (0 = no signal, 255 = max.magnitude)
  int8_t ofsQ;          // Imbalance of Q-part of complex signal, scaled (-128 = max. negative imbalance, 127 = max. positive imbalance)
  uint8_t magQ;         // Magnitude of Q-part of complex signal, scaled (0 = no signal, 255 = max.magnitude)
  uint8_t reserved2[3]; // Reserved
} UBX_MON_RF_block_t;

typedef struct
{
  UBX_MON_RF_header_t header;
  UBX_MON_RF_block_t blocks[UBX_MON_RF_MAX_BLOCKS];
} UBX_MON_RF_data_t;

// TIM-specific structs

// UBX-TIM-TM2 (0x0D 0x03): Time mark data
const uint16_t UBX_TIM_TM2_LEN = 28;

typedef struct
{
  uint8_t ch; // Channel (i.e. EXTINT) upon which the pulse was measured
  union
  {
    uint8_t all;
    struct
    {
      uint8_t mode : 1;           // 0=single; 1=running
      uint8_t run : 1;            // 0=armed; 1=stopped
      uint8_t newFallingEdge : 1; // New falling edge detected
      uint8_t timeBase : 2;       // 0=Time base is Receiver time; 1=Time base is GNSS time; 2=Time base is UTC
      uint8_t utc : 1;            // 0=UTC not available; 1=UTC available
      uint8_t time : 1;           // 0=Time is not valid; 1=Time is valid (Valid GNSS fix)
      uint8_t newRisingEdge : 1;  // New rising edge detected
    } bits;
  } flags;
  uint16_t count;     // Rising edge counter
  uint16_t wnR;       // Week number of last rising edge
  uint16_t wnF;       // Week number of last falling edge
  uint32_t towMsR;    // TOW of rising edge: ms
  uint32_t towSubMsR; // Millisecond fraction of tow of rising edge: ns
  uint32_t towMsF;    // TOW of falling edge: ms
  uint32_t towSubMsF; // Millisecond fraction of tow of falling edge: ns
  uint32_t accEst;    // Accuracy estimate: ns
} UBX_TIM_TM2_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t ch : 1;
      uint32_t mode : 1;
      uint32_t run : 1;
      uint32_t newFallingEdge : 1;
      uint32_t timeBase : 1;
      uint32_t utc : 1;
      uint32_t time : 1;
      uint32_t newRisingEdge : 1;

      uint32_t count : 1;
      uint32_t wnR : 1;
      uint32_t wnF : 1;
      uint32_t towMsR : 1;
      uint32_t towSubMsR : 1;
      uint32_t towMsF : 1;
      uint32_t towSubMsF : 1;
      uint32_t accEst : 1;
    } bits;
  } moduleQueried;
} UBX_TIM_TM2_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_TIM_TM2_data_t data;
  UBX_TIM_TM2_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_TIM_TM2_data_t);
  void (*callbackPointerPtr)(UBX_TIM_TM2_data_t *);
  UBX_TIM_TM2_data_t *callbackData;
} UBX_TIM_TM2_t;

// ESF-specific structs

// UBX-ESF-ALG (0x10 0x14): IMU alignment information
const uint16_t UBX_ESF_ALG_LEN = 16;

typedef struct
{
  uint32_t iTOW;   // GPS time of week of the HNR epoch: ms
  uint8_t version; // Message version (0x01 for this version)
  union
  {
    uint8_t all;
    struct
    {
      uint8_t autoMntAlgOn : 1; // Automatic IMU-mount alignment on/off bit
      uint8_t status : 3;       // Status of the IMU-mount alignment
                                //   0: user-defined/fixed angles are used
                                //   1: IMU-mount roll/pitch angles alignment is ongoing
                                //   2: IMU-mount roll/pitch/yaw angles alignment is ongoing
                                //   3: coarse IMU-mount alignment are used
                                //   4: fine IMU-mount alignment are used
    } bits;
  } flags;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t tiltAlgError : 1; // IMU-mount tilt (roll and/or pitch) alignment error (0: no error, 1: error)
      uint8_t yawAlgError : 1;  // IMU-mount yaw alignment error (0: no error, 1: error)
      uint8_t angleError : 1;   // IMU-mount misalignment Euler angle singularity error (0: no error, 1: error)
    } bits;
  } error;
  uint8_t reserved1;
  uint32_t yaw;  // IMU-mount yaw angle [0, 360]: Degrees * 1e-2
  int16_t pitch; // IMU-mount pitch angle [-90, 90]: Degrees * 1e-2
  int16_t roll;  // IMU-mount roll angle [-180, 180]: Degrees * 1e-2
} UBX_ESF_ALG_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t version : 1;

      uint32_t autoMntAlgOn : 1;
      uint32_t status : 1;

      uint32_t tiltAlgError : 1;
      uint32_t yawAlgError : 1;
      uint32_t angleError : 1;

      uint32_t yaw : 1;
      uint32_t pitch : 1;
      uint32_t roll : 1;
    } bits;
  } moduleQueried;
} UBX_ESF_ALG_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_ESF_ALG_data_t data;
  UBX_ESF_ALG_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_ESF_ALG_data_t);
  void (*callbackPointerPtr)(UBX_ESF_ALG_data_t *);
  UBX_ESF_ALG_data_t *callbackData;
} UBX_ESF_ALG_t;

// UBX-ESF-INS (0x10 0x15): Vehicle dynamics information
const uint16_t UBX_ESF_INS_LEN = 36;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t version : 8;       // Message version (0x01 for this version)
      uint32_t xAngRateValid : 1; // Compensated x-axis angular rate data validity flag (0: not valid, 1: valid)
      uint32_t yAngRateValid : 1; // Compensated y-axis angular rate data validity flag (0: not valid, 1: valid)
      uint32_t zAngRateValid : 1; // Compensated z-axis angular rate data validity flag (0: not valid, 1: valid)
      uint32_t xAccelValid : 1;   // Compensated x-axis acceleration data validity flag (0: not valid, 1: valid)
      uint32_t yAccelValid : 1;   // Compensated y-axis acceleration data validity flag (0: not valid, 1: valid)
      uint32_t zAccelValid : 1;   // Compensated z-axis acceleration data validity flag (0: not valid, 1: valid)
    } bits;
  } bitfield0;
  uint8_t reserved1[4];
  uint32_t iTOW;    // GPS time of week of the HNR epoch: ms
  int32_t xAngRate; // Compensated x-axis angular rate: Degrees/s * 1e-3
  int32_t yAngRate; // Compensated y-axis angular rate: Degrees/s * 1e-3
  int32_t zAngRate; // Compensated z-axis angular rate: Degrees/s * 1e-3
  int32_t xAccel;   // Compensated x-axis acceleration (gravity-free): m/s^2 * 1e-2
  int32_t yAccel;   // Compensated y-axis acceleration (gravity-free): m/s^2 * 1e-2
  int32_t zAccel;   // Compensated z-axis acceleration (gravity-free): m/s^2 * 1e-2
} UBX_ESF_INS_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t version : 1;
      uint32_t xAngRateValid : 1;
      uint32_t yAngRateValid : 1;
      uint32_t zAngRateValid : 1;
      uint32_t xAccelValid : 1;
      uint32_t yAccelValid : 1;
      uint32_t zAccelValid : 1;

      uint32_t iTOW : 1;
      uint32_t xAngRate : 1;
      uint32_t yAngRate : 1;
      uint32_t zAngRate : 1;
      uint32_t xAccel : 1;
      uint32_t yAccel : 1;
      uint32_t zAccel : 1;
    } bits;
  } moduleQueried;
} UBX_ESF_INS_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_ESF_INS_data_t data;
  UBX_ESF_INS_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_ESF_INS_data_t);
  void (*callbackPointerPtr)(UBX_ESF_INS_data_t *);
  UBX_ESF_INS_data_t *callbackData;
} UBX_ESF_INS_t;

// UBX-ESF-MEAS (0x10 0x02): External sensor fusion measurements
// Note: length is variable
// Note: ESF RAW data cannot be polled. It is "Output" only
const uint16_t UBX_ESF_MEAS_MAX_LEN = 8 + (4 * DEF_MAX_NUM_ESF_MEAS) + 4;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t dataField : 24; // Data
      uint32_t dataType : 6;   // Type of data (0 = no data; 1..63 = data type)
    } bits;
  } data;
} UBX_ESF_MEAS_sensorData_t;

typedef struct
{
  uint32_t timeTag; // Time tag of measurement generated by external sensor
  union
  {
    uint16_t all;
    struct
    {
      uint16_t timeMarkSent : 2;   // Time mark signal was supplied just prior to sending this message:
                                   //   0 = none, 1 = on Ext0, 2 = on Ext1
      uint16_t timeMarkEdge : 1;   // Trigger on rising (0) or falling (1) edge of time mark signal
      uint16_t calibTtagValid : 1; // Calibration time tag available. Always set to zero.
      uint16_t reserved : 7;
      uint16_t numMeas : 5; // Number of measurements contained in this message (optional, can be obtained from message size)
    } bits;
  } flags;
  uint16_t id; // Identification number of data provider
  UBX_ESF_MEAS_sensorData_t data[DEF_MAX_NUM_ESF_MEAS];
  uint32_t calibTtag; // OPTIONAL: Receiver local time calibrated: ms
} UBX_ESF_MEAS_data_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_ESF_MEAS_data_t data;
  void (*callbackPointer)(UBX_ESF_MEAS_data_t);
  void (*callbackPointerPtr)(UBX_ESF_MEAS_data_t *);
  UBX_ESF_MEAS_data_t *callbackData;
} UBX_ESF_MEAS_t;

// UBX-ESF-RAW (0x10 0x03): Raw sensor measurements
// Note: length is variable
// Note: The ZED-F9R sends sets of seven sensor readings one at a time
//       But the NEO-M8U sends them in sets of ten (i.e. seventy readings per message)
// Note: ESF RAW data cannot be polled. It is "Output" only
const uint16_t UBX_ESF_RAW_MAX_LEN = 4 + (8 * DEF_NUM_SENS * DEF_MAX_NUM_ESF_RAW_REPEATS);

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t dataField : 24; // Data
      uint32_t dataType : 8;   // Type of data (0 = no data; 1..255 = data type)
    } bits;
  } data;
  uint32_t sTag; // Sensor time tag
} UBX_ESF_RAW_sensorData_t;

typedef struct
{
  uint8_t reserved1[4];
  UBX_ESF_RAW_sensorData_t data[DEF_NUM_SENS * DEF_MAX_NUM_ESF_RAW_REPEATS];
  uint8_t numEsfRawBlocks; // Note: this is not contained in the ESF RAW message. It is calculated from the message length.
} UBX_ESF_RAW_data_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_ESF_RAW_data_t data;
  void (*callbackPointer)(UBX_ESF_RAW_data_t);
  void (*callbackPointerPtr)(UBX_ESF_RAW_data_t *);
  UBX_ESF_RAW_data_t *callbackData;
} UBX_ESF_RAW_t;

// UBX-ESF-STATUS (0x10 0x10): External sensor fusion status
// Note: length is variable
const uint16_t UBX_ESF_STATUS_MAX_LEN = 16 + (4 * DEF_NUM_SENS);

typedef struct
{
  union
  {
    uint8_t all;
    struct
    {
      uint8_t type : 6;  // Sensor data type
      uint8_t used : 1;  // If set, sensor data is used for the current sensor fusion solution
      uint8_t ready : 1; // If set, sensor is set up (configuration is available or not required) but not used for computing the current sensor fusion solution.
    } bits;
  } sensStatus1;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t calibStatus : 2; // 00: Sensor is not calibrated
                               // 01: Sensor is calibrating
                               // 10/11: Sensor is calibrated
      uint8_t timeStatus : 2;  // 00: No data
                               // 01: Reception of the first byte used to tag the measurement
                               // 10: Event input used to tag the measurement
                               // 11: Time tag provided with the data
    } bits;
  } sensStatus2;
  uint8_t freq; // Observation frequency: Hz
  union
  {
    uint8_t all;
    struct
    {
      uint8_t badMeas : 1;     // Bad measurements detected
      uint8_t badTTag : 1;     // Bad measurement time-tags detected
      uint8_t missingMeas : 1; // Missing or time-misaligned measurements detected
      uint8_t noisyMeas : 1;   // High measurement noise-level detected
    } bits;
  } faults;
} UBX_ESF_STATUS_sensorStatus_t;

typedef struct
{
  uint32_t iTOW;   // GPS time of week of the HNR epoch: ms
  uint8_t version; // Message version (0x02 for this version)
  uint8_t reserved1[7];
  uint8_t fusionMode; // Fusion mode:
                      //  0: Initialization mode: receiver is initializing some unknown values required for doing sensor fusion
                      //  1: Fusion mode: GNSS and sensor data are used for navigation solution computation
                      //  2: Suspended fusion mode: sensor fusion is temporarily disabled due to e.g. invalid sensor data or detected ferry
                      //  3: Disabled fusion mode: sensor fusion is permanently disabled until receiver reset due e.g. to sensor error
  uint8_t reserved2[2];
  uint8_t numSens; // Number of sensors
  UBX_ESF_STATUS_sensorStatus_t status[DEF_NUM_SENS];
} UBX_ESF_STATUS_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t version : 1;
      uint32_t fusionMode : 1;
      uint32_t numSens : 1;

      uint32_t status : DEF_NUM_SENS;
    } bits;
  } moduleQueried;
} UBX_ESF_STATUS_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_ESF_STATUS_data_t data;
  UBX_ESF_STATUS_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_ESF_STATUS_data_t);
  void (*callbackPointerPtr)(UBX_ESF_STATUS_data_t *);
  UBX_ESF_STATUS_data_t *callbackData;
} UBX_ESF_STATUS_t;

// MGA-specific structs

// UBX-MGA-ACK-DATA0 (0x13 0x60): Multiple GNSS acknowledge message
const uint16_t UBX_MGA_ACK_DATA0_LEN = 8;

typedef struct
{
  uint8_t type;               // Type of acknowledgment:
                              // 0: The message was not used by the receiver (see infoCode field for an indication of why)
                              // 1: The message was accepted for use by the receiver (the infoCode field will be 0)
  uint8_t version;            // Message version
  uint8_t infoCode;           // Provides greater information on what the receiver chose to do with the message contents
                              // See sfe_ublox_mga_ack_infocode_e
  uint8_t msgId;              // UBX message ID of the acknowledged message
  uint8_t msgPayloadStart[4]; // The first 4 bytes of the acknowledged message's payload
} UBX_MGA_ACK_DATA0_data_t;

#define UBX_MGA_ACK_DATA0_RINGBUFFER_LEN 16 // Provide storage for 16 MGA ACK packets
typedef struct
{
  uint8_t head;
  uint8_t tail;
  UBX_MGA_ACK_DATA0_data_t data[UBX_MGA_ACK_DATA0_RINGBUFFER_LEN]; // Create a storage array for the MGA ACK packets
} UBX_MGA_ACK_DATA0_t;

// UBX-MGA-DBD (0x13 0x80): Navigation database dump entry
const uint16_t UBX_MGA_DBD_LEN = 164; // "The maximum payload size for firmware 2.01 onwards is 164 bytes"

typedef struct
{
  uint8_t dbdEntryHeader1; // We need to save the entire message - header, payload and checksum
  uint8_t dbdEntryHeader2;
  uint8_t dbdEntryClass;
  uint8_t dbdEntryID;
  uint8_t dbdEntryLenLSB; // We need to store the length of the DBD entry. The entry itself does not contain a length...
  uint8_t dbdEntryLenMSB;
  uint8_t dbdEntry[UBX_MGA_DBD_LEN];
  uint8_t dbdEntryChecksumA;
  uint8_t dbdEntryChecksumB;
} UBX_MGA_DBD_data_t;

#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_MEGAAVR)
#define UBX_MGA_DBD_RINGBUFFER_LEN 190 // Fix to let the code compile on AVR platforms - including the UNO and DxCore (DA/DB).
#else
#define UBX_MGA_DBD_RINGBUFFER_LEN 250 // Provide storage for MGA DBD packets. TO DO: confirm if 250 is large enough for all modules!
#endif

typedef struct
{
  uint8_t head;
  uint8_t tail;
  UBX_MGA_DBD_data_t data[UBX_MGA_DBD_RINGBUFFER_LEN]; // Create a storage array for the MGA DBD packets
} UBX_MGA_DBD_t;

// HNR-specific structs

// UBX-HNR-PVT (0x28 0x00): High rate output of PVT solution
const uint16_t UBX_HNR_PVT_LEN = 72;

typedef struct
{
  uint32_t iTOW; // GPS time of week of the HNR epoch: ms
  uint16_t year; // Year (UTC)
  uint8_t month; // Month, range 1..12 (UTC)
  uint8_t day;   // Day of month, range 1..31 (UTC)
  uint8_t hour;  // Hour of day, range 0..23 (UTC)
  uint8_t min;   // Minute of hour, range 0..59 (UTC)
  uint8_t sec;   // Seconds of minute, range 0..60 (UTC)
  union
  {
    uint8_t all;
    struct
    {
      uint8_t validDate : 1;     // 1 = Valid UTC Date
      uint8_t validTime : 1;     // 1 = Valid UTC Time of Day
      uint8_t fullyResolved : 1; // 1 = UTC Time of Day has been fully resolved
    } bits;
  } valid;
  int32_t nano;   // Fraction of second (UTC): ns
  uint8_t gpsFix; // GPSfix Type, range 0..5
                  // 0x00 = No Fix
                  // 0x01 = Dead Reckoning only
                  // 0x02 = 2D-Fix
                  // 0x03 = 3D-Fix
                  // 0x04 = GPS + dead reckoning combined
                  // 0x05 = Time only fix
                  // 0x06..0xff: reserved
  union
  {
    uint8_t all;
    struct
    {
      uint8_t gpsFixOK : 1;     // >1 = Fix within limits (e.g. DOP & accuracy)
      uint8_t diffSoln : 1;     // 1 = DGPS used
      uint8_t WKNSET : 1;       // 1 = Valid GPS week number
      uint8_t TOWSET : 1;       // 1 = Valid GPS time of week (iTOW & fTOW)
      uint8_t headVehValid : 1; // 1= Heading of vehicle is valid
    } bits;
  } flags;
  uint8_t reserved1[2];
  int32_t lon;      // Longitude: Degrees * 1e-7
  int32_t lat;      // Latitude: Degrees * 1e-7
  int32_t height;   // Height above ellipsoid: mm
  int32_t hMSL;     // Height above MSL: mm
  int32_t gSpeed;   // Ground Speed (2-D): mm/s
  int32_t speed;    // Speed (3-D): mm/s
  int32_t headMot;  // Heading of motion (2-D): Degrees * 1e-5
  int32_t headVeh;  // Heading of vehicle (2-D): Degrees * 1e-5
  uint32_t hAcc;    // Horizontal accuracy: mm
  uint32_t vAcc;    // Vertical accuracy: mm
  uint32_t sAcc;    // Speed accuracy: mm/s
  uint32_t headAcc; // Heading accuracy: Degrees * 1e-5
  uint8_t reserved2[4];
} UBX_HNR_PVT_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t year : 1;
      uint32_t month : 1;
      uint32_t day : 1;
      uint32_t hour : 1;
      uint32_t min : 1;
      uint32_t sec : 1;

      uint32_t validDate : 1;
      uint32_t validTime : 1;
      uint32_t fullyResolved : 1;

      uint32_t nano : 1;
      uint32_t gpsFix : 1;

      uint32_t gpsFixOK : 1;
      uint32_t diffSoln : 1;
      uint32_t WKNSET : 1;
      uint32_t TOWSET : 1;
      uint32_t headVehValid : 1;

      uint32_t lon : 1;
      uint32_t lat : 1;
      uint32_t height : 1;
      uint32_t hMSL : 1;
      uint32_t gSpeed : 1;
      uint32_t speed : 1;
      uint32_t headMot : 1;
      uint32_t headVeh : 1;
      uint32_t hAcc : 1;
      uint32_t vAcc : 1;
      uint32_t sAcc : 1;
      uint32_t headAcc : 1;
    } bits;
  } moduleQueried;
} UBX_HNR_PVT_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_HNR_PVT_data_t data;
  UBX_HNR_PVT_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_HNR_PVT_data_t);
  void (*callbackPointerPtr)(UBX_HNR_PVT_data_t *);
  UBX_HNR_PVT_data_t *callbackData;
} UBX_HNR_PVT_t;

// UBX-HNR-ATT (0x28 0x01): Attitude solution
const uint16_t UBX_HNR_ATT_LEN = 32;

typedef struct
{
  uint32_t iTOW; // GPS time of week of the navigation epoch: ms
  uint8_t version;
  uint8_t reserved1[3];
  int32_t roll;        // Vehicle roll: Degrees * 1e-5
  int32_t pitch;       // Vehicle pitch: Degrees * 1e-5
  int32_t heading;     // Vehicle heading: Degrees * 1e-5
  uint32_t accRoll;    // Vehicle roll accuracy: Degrees * 1e-5
  uint32_t accPitch;   // Vehicle pitch accuracy: Degrees * 1e-5
  uint32_t accHeading; // Vehicle heading accuracy: Degrees * 1e-5
} UBX_HNR_ATT_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t iTOW : 1;
      uint32_t version : 1;
      uint32_t roll : 1;
      uint32_t pitch : 1;
      uint32_t heading : 1;
      uint32_t accRoll : 1;
      uint32_t accPitch : 1;
      uint32_t accHeading : 1;
    } bits;
  } moduleQueried;
} UBX_HNR_ATT_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_HNR_ATT_data_t data;
  UBX_HNR_ATT_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_HNR_ATT_data_t);
  void (*callbackPointerPtr)(UBX_HNR_ATT_data_t *);
  UBX_HNR_ATT_data_t *callbackData;
} UBX_HNR_ATT_t;

// UBX-HNR-INS (0x28 0x02): Vehicle dynamics information
const uint16_t UBX_HNR_INS_LEN = 36;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t version : 8;       // Message version (0x00 for this version)
      uint32_t xAngRateValid : 1; // Compensated x-axis angular rate data validity flag (0: not valid, 1: valid)
      uint32_t yAngRateValid : 1; // Compensated y-axis angular rate data validity flag (0: not valid, 1: valid)
      uint32_t zAngRateValid : 1; // Compensated z-axis angular rate data validity flag (0: not valid, 1: valid)
      uint32_t xAccelValid : 1;   // Compensated x-axis acceleration data validity flag (0: not valid, 1: valid)
      uint32_t yAccelValid : 1;   // Compensated y-axis acceleration data validity flag (0: not valid, 1: valid)
      uint32_t zAccelValid : 1;   // Compensated z-axis acceleration data validity flag (0: not valid, 1: valid)
    } bits;
  } bitfield0;
  uint8_t reserved1[4];
  uint32_t iTOW;    // GPS time of week of the HNR epoch: ms
  int32_t xAngRate; // Compensated x-axis angular rate: Degrees/s * 1e-3
  int32_t yAngRate; // Compensated y-axis angular rate: Degrees/s * 1e-3
  int32_t zAngRate; // Compensated z-axis angular rate: Degrees/s * 1e-3
  int32_t xAccel;   // Compensated x-axis acceleration (with gravity): m/s^2 * 1e-2
  int32_t yAccel;   // Compensated y-axis acceleration (with gravity): m/s^2 * 1e-2
  int32_t zAccel;   // Compensated z-axis acceleration (with gravity): m/s^2 * 1e-2
} UBX_HNR_INS_data_t;

typedef struct
{
  union
  {
    uint32_t all;
    struct
    {
      uint32_t all : 1;

      uint32_t version : 1;
      uint32_t xAngRateValid : 1;
      uint32_t yAngRateValid : 1;
      uint32_t zAngRateValid : 1;
      uint32_t xAccelValid : 1;
      uint32_t yAccelValid : 1;
      uint32_t zAccelValid : 1;

      uint32_t iTOW : 1;
      uint32_t xAngRate : 1;
      uint32_t yAngRate : 1;
      uint32_t zAngRate : 1;
      uint32_t xAccel : 1;
      uint32_t yAccel : 1;
      uint32_t zAccel : 1;
    } bits;
  } moduleQueried;
} UBX_HNR_INS_moduleQueried_t;

typedef struct
{
  ubxAutomaticFlags automaticFlags;
  UBX_HNR_INS_data_t data;
  UBX_HNR_INS_moduleQueried_t moduleQueried;
  void (*callbackPointer)(UBX_HNR_INS_data_t);
  void (*callbackPointerPtr)(UBX_HNR_INS_data_t *);
  UBX_HNR_INS_data_t *callbackData;
} UBX_HNR_INS_t;

// NMEA-specific structs

// Additional flags and pointers that need to be stored with each message type
struct nmeaAutomaticFlags
{
  union
  {
    uint8_t all;
    struct
    {
      uint8_t completeCopyValid : 1; // Is the copy of the data struct used by the get function valid/fresh? 0 = invalid, 1 = valid
      uint8_t completeCopyRead : 1;  // Has the complete copy been read? 0 = unread, 1 = read
      uint8_t callbackCopyValid : 1; // Is the copy of the data struct used by the callback valid/fresh? 0 = invalid/stale, 1 = valid/fresh
    } bits;
  } flags;
};

// The max length for NMEA messages should be 82 bytes, but GGA messages can exceed that if they include the
// extra decimal places for "High Precision Mode".
//
// To be safe, let's allocate 100 bytes to store the GGA message

const uint8_t NMEA_GGA_MAX_LENGTH = 100;

typedef struct
{
  uint8_t length; // The number of bytes in nmea
  uint8_t nmea[NMEA_GGA_MAX_LENGTH];
} NMEA_GGA_data_t;

typedef struct
{
  nmeaAutomaticFlags automaticFlags;
  NMEA_GGA_data_t workingCopy;  // Incoming data is added to the working copy
  NMEA_GGA_data_t completeCopy; // The working copy is copied into the complete copy when all data has been received and the checksum is valid
  void (*callbackPointer)(NMEA_GGA_data_t);
  void (*callbackPointerPtr)(NMEA_GGA_data_t *);
  NMEA_GGA_data_t *callbackCopy; // The callback gets its own preserved copy of the complete copy
} NMEA_GPGGA_t;

typedef struct
{
  nmeaAutomaticFlags automaticFlags;
  NMEA_GGA_data_t workingCopy;  // Incoming data is added to the working copy
  NMEA_GGA_data_t completeCopy; // The working copy is copied into the complete copy when all data has been received and the checksum is valid
  void (*callbackPointer)(NMEA_GGA_data_t);
  void (*callbackPointerPtr)(NMEA_GGA_data_t *);
  NMEA_GGA_data_t *callbackCopy; // The callback gets its own preserved copy of the complete copy
} NMEA_GNGGA_t;

const uint8_t NMEA_VTG_MAX_LENGTH = 100;

typedef struct
{
  uint8_t length; // The number of bytes in nmea
  uint8_t nmea[NMEA_VTG_MAX_LENGTH];
} NMEA_VTG_data_t;

typedef struct
{
  nmeaAutomaticFlags automaticFlags;
  NMEA_VTG_data_t workingCopy;  // Incoming data is added to the working copy
  NMEA_VTG_data_t completeCopy; // The working copy is copied into the complete copy when all data has been received and the checksum is valid
  void (*callbackPointer)(NMEA_VTG_data_t);
  void (*callbackPointerPtr)(NMEA_VTG_data_t *);
  NMEA_VTG_data_t *callbackCopy; // The callback gets its own preserved copy of the complete copy
} NMEA_GPVTG_t;

typedef struct
{
  nmeaAutomaticFlags automaticFlags;
  NMEA_VTG_data_t workingCopy;  // Incoming data is added to the working copy
  NMEA_VTG_data_t completeCopy; // The working copy is copied into the complete copy when all data has been received and the checksum is valid
  void (*callbackPointer)(NMEA_VTG_data_t);
  void (*callbackPointerPtr)(NMEA_VTG_data_t *);
  NMEA_VTG_data_t *callbackCopy; // The callback gets its own preserved copy of the complete copy
} NMEA_GNVTG_t;

const uint8_t NMEA_RMC_MAX_LENGTH = 100;

typedef struct
{
  uint8_t length; // The number of bytes in nmea
  uint8_t nmea[NMEA_RMC_MAX_LENGTH];
} NMEA_RMC_data_t;

typedef struct
{
  nmeaAutomaticFlags automaticFlags;
  NMEA_RMC_data_t workingCopy;  // Incoming data is added to the working copy
  NMEA_RMC_data_t completeCopy; // The working copy is copied into the complete copy when all data has been received and the checksum is valid
  void (*callbackPointer)(NMEA_RMC_data_t);
  void (*callbackPointerPtr)(NMEA_RMC_data_t *);
  NMEA_RMC_data_t *callbackCopy; // The callback gets its own preserved copy of the complete copy
} NMEA_GPRMC_t;

typedef struct
{
  nmeaAutomaticFlags automaticFlags;
  NMEA_RMC_data_t workingCopy;  // Incoming data is added to the working copy
  NMEA_RMC_data_t completeCopy; // The working copy is copied into the complete copy when all data has been received and the checksum is valid
  void (*callbackPointer)(NMEA_RMC_data_t);
  void (*callbackPointerPtr)(NMEA_RMC_data_t *);
  NMEA_RMC_data_t *callbackCopy; // The callback gets its own preserved copy of the complete copy
} NMEA_GNRMC_t;

const uint8_t NMEA_ZDA_MAX_LENGTH = 50;

typedef struct
{
  uint8_t length; // The number of bytes in nmea
  uint8_t nmea[NMEA_ZDA_MAX_LENGTH];
} NMEA_ZDA_data_t;

typedef struct
{
  nmeaAutomaticFlags automaticFlags;
  NMEA_ZDA_data_t workingCopy;  // Incoming data is added to the working copy
  NMEA_ZDA_data_t completeCopy; // The working copy is copied into the complete copy when all data has been received and the checksum is valid
  void (*callbackPointer)(NMEA_ZDA_data_t);
  void (*callbackPointerPtr)(NMEA_ZDA_data_t *);
  NMEA_ZDA_data_t *callbackCopy; // The callback gets its own preserved copy of the complete copy
} NMEA_GPZDA_t;

typedef struct
{
  nmeaAutomaticFlags automaticFlags;
  NMEA_ZDA_data_t workingCopy;  // Incoming data is added to the working copy
  NMEA_ZDA_data_t completeCopy; // The working copy is copied into the complete copy when all data has been received and the checksum is valid
  void (*callbackPointer)(NMEA_ZDA_data_t);
  void (*callbackPointerPtr)(NMEA_ZDA_data_t *);
  NMEA_ZDA_data_t *callbackCopy; // The callback gets its own preserved copy of the complete copy
} NMEA_GNZDA_t;

#endif
