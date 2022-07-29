## How I<sup>2</sup>C (aka DDC) communication works with a u-blox module

When the user calls one of the methods the library will poll the u-blox module for new data.

* Wait for a minimum of 25 ms between polls (configured dynamically when update rate is set)
* Write 0xFD to module
* Read two bytes (0xFD and 0xFE) for bytes available
* Otherwise, read number of bytes and process into NMEA, UBX, or RTCM frame.
* If checksum is valid, flag frame as complete.

This library was originally written to use the I<sup>2</sup>C interface but Serial has been implemented as well.

## How data is processed by this library

In Version 1 of this library, we tried to minimize memory usage by being very careful about how much RAM we allocated to UBX packet storage and processing. We used only three buffers or containers to store the incoming data: **packetBuf** (packetBuffer); **packetCfg** (packetConfiguration); and **packetAck** (packetAcknowledge). Incoming packets were stored in **packetBuf** initially and then diverted into **packetAck** or **packetCfg** as necessary. Once data was received and validated, it would be copied out of **packetCfg** and into 'global' variables with names like ```gpsSecond``` or ```latitude```. We also introduced the concept of _Polling vs. Auto-Reporting_ where messages like PVT (Position, Velocity, Time) could be generated and parsed "automatically". This meant that functions like ```getLatitude``` could be non-blocking, returning the most recent data and requesting fresh data when necessary. But it also meant that _polled_ messages could be _overwritten_ (in **packetCfg**) by any _auto-reported_ messages. The library dealt with this successfully, but it was a headache.

Version 1 had two main drawbacks. As time went on:
- the RAM use increased as we had to add new 'global' storage for each new data type
- the number of messages which needed "auto" processing through **packetCfg** became complex, requiring significant code changes each time a new "auto" message was added. (We started with NAV-PVT. Then came NAV-HPPOSLLH and NAV-DOP. Things got complicated when HNR-ATT, HNR-INS and HNR-PVT were added to the mix.)

Version 2 of the library does things differently. Whilst of course trying to keep the library backward-compatible as much as possible, we have taken a fresh approach:
- We have added **packetAuto** which is used to temporarily buffer expected auto-reported messages and prevents data from being overwritten in **packetCfg**.
  - The payload for **packetAuto** is allocated dynamically in RAM and deleted after use.
  - If insufficient RAM is available, the code falls back to using **packetCfg** to buffer the data instead.
- The library no longer uses 'global' (permanently-allocated) storage for the GNSS data. Instead:
  - Each message type has a **typedef struct** defined which matches the format of the UBX message. (_typedef structs_ are just definitions, they don't occupy memory.) You can find the definitions in [_**u-blox_structs.h**_](./src/u-blox_structs.h).
  - The struct allows each data field (latitude, longitude, etc.) to be read simply and easily using dot notation. Flags etc. are supported by bit definitions in the struct. The field names are as defined in the u-blox interface description.
  - Storage for that message is only _allocated_ in RAM if/when required. The allocation is done using _new_ via a pointer to the struct.
- _Any_ message can be "auto" if required, but can be polled too.
- An optional _callback_ can be associated with the arrival of each message type. A simple scheduler ```checkCallbacks``` triggers the callbacks once I<sup>2</sup>C/Serial data reception is complete.
  - This means that your code no longer needs to wait for the arrival of a message, you are able to request (e.g.) PVT or HNR data and your callback is called once the data arrives.
  - The callbacks are not re-entrant.
  - The callback receives a _copy_ of the data, so data reception and processing can continue while the callback is executing. Data integrity is preserved. You can call ```checkUblox()``` from inside a callback if needed.
- Incoming data can be copied to a separate buffer to allow automatic writing to a file on SD card, which will be useful for (e.g.) RAWX logging.
  - Data is stored in a RingBuffer, the size of which can be set by calling ```setFileBufferSize``` _before_ ```.begin```.
  - The default buffer size is zero - to save memory.
  - To simplify SD card writing, data can be copied from the RingBuffer to a user-defined linear buffer first using ```extractFileBufferData```.
  - Data reception and processing can continue during the SD write.
  - User-defined code does the actual writing of data from the linear buffer to the SD card. The u-blox GNSS library itself does not perform the writing and so is not tied to any particular SD library.
  - The logged files can be played back and analyzed with (e.g.) u-center or RTKLIB.

In terms of RAM, you may find that your total RAM use is lower using v2 compared to v1, but it does of course depend on how many message types are being processed. The downside to this is that it is difficult to know in advance how much RAM is required, since it is only allocated if/when required. If the processor runs out of RAM (i.e. the _new_ fails) then a debug error message is generated.

## "Auto" messages

In v2.0, the full list of messages which can be processed and logged automatically is:
- UBX-NAV-POSECEF (0x01 0x01): Position solution in ECEF
- UBX-NAV-STATUS (0x01 0x03): Receiver navigation status
- UBX-NAV-DOP (0x01 0x04): Dilution of precision
- UBX-NAV-ATT (0x01 0x05): Attitude solution (**only with ADR or UDR products**)
- UBX-NAV-PVT (0x01 0x07): Navigation position velocity time solution
- UBX-NAV-ODO (0x01 0x09): Odometer solution
- UBX-NAV-VELECEF (0x01 0x11): Velocity solution in ECEF
- UBX-NAV-VELNED (0x01 0x12): Velocity solution in NED frame
- UBX-NAV-HPPOSECEF (0x01 0x13): High precision position solution in ECEF
- UBX-NAV-HPPOSLLH (0x01 0x14): High precision geodetic position solution
- UBX-NAV-PVAT (0x01 0x17): Navigation position velocity attitude time solution (**only with ADR or UDR products**)
- UBX-NAV-TIMEUTC (0x01 0x21): UTC time solution
- UBX-NAV-CLOCK (0x01 0x22): Clock solution
- UBX-NAV-SAT (0x01 0x35): Satellite information
- UBX-NAV-SVIN (0x01 0x3B): Survey-in data (**only with High Precision GNSS products**)
- UBX-NAV-RELPOSNED (0x01 0x3C): Relative positioning information in NED frame (**only with High Precision GNSS products**)
- UBX-NAV-AOPSTATUS (0x01 0x60): AssistNow Autonomous status
- UBX-NAV-EOE (0x01 0x61): End of epoch
- UBX-RXM-SFRBX (0x02 0x13): Broadcast navigation data subframe
- UBX-RXM-RAWX (0x02 0x15): Multi-GNSS raw measurement data (**only with ADR or High Precision GNSS or Time Sync products**)
- UBX-TIM-TM2 (0x0D 0x03): Time mark data
- UBX-ESF-ALG (0x10 0x14): IMU alignment information (**only with ADR or UDR products**)
- UBX-ESF-INS (0x10 0x15): Vehicle dynamics information (**only with ADR or UDR products**)
- UBX-ESF-MEAS (0x10 0x02): External sensor fusion measurements (**only with ADR or UDR products**)
- UBX-ESF-RAW (0x10 0x03): Raw sensor measurements (**only with ADR or UDR products**)
- UBX-ESF-STATUS (0x10 0x10): External sensor fusion status (**only with ADR or UDR products**)
- UBX-HNR-PVT (0x28 0x00): High rate output of PVT solution (**only with ADR or UDR products**)
- UBX-HNR-ATT (0x28 0x01): Attitude solution (**only with ADR or UDR products**)
- UBX-HNR-INS (0x28 0x02): Vehicle dynamics information (**only with ADR or UDR products**)

Please see [Adding_New_Messages](./Adding_New_Messages.md) for details on how to add "auto" support for new messages.

Notes:
- UBX-NAV-POSLLH is not supported as UBX-NAV-PVT contains the same information

## Migrating your code to v2.0

Migrating to v2.0 is easy. There are two small changes all users will need to make:

* The name of the library class has changed from ```SFE_UBLOX_GPS``` to ```SFE_UBLOX_GNSS``` to reflect that the library supports all of the Global Navigation Satellite Systems:
  * As a minimum, you need to change: ```SFE_UBLOX_GPS myGPS;```
  * to: ```SFE_UBLOX_GNSS myGPS;```
  * But we would encourage you to use ```SFE_UBLOX_GNSS myGNSS;```. You will see that all of the library examples now use ```myGNSS``` instead of ```myGPS```.
* The name of the library header and C++ files have changed too:
  * Change: ```#include <SparkFun_Ublox_Arduino_Library.h>```
  * to: ```#include <SparkFun_u-blox_GNSS_Arduino_Library.h>```

The biggest change in v2.0 is that data is now stored in a _struct_ which matches the u-blox interface description for that message. For example:
- In v1, the NAV PVT (Position Velocity Time) latitude and longitude were stored in 'global' _int32_t_ variables called ```latitude``` and ```longitude```
  - In v2.0, the data is now stored in <strong>UBX_NAV_PVT_t *packetUBXNAVPVT</strong>
  - ```myGPS.latitude``` becomes ```myGNSS.packetUBXNAVPVT->data.lat```
  - ```myGPS.longitude``` becomes ```myGNSS.packetUBXNAVPVT->data.lon```
  - The helper functions ```myGNSS.getLatitude()``` and ```myGNSS.getLongitude()``` are still available and work in the same way.
- In v1, the ESF Sensor Fusion data for the Dead Reckoning modules was stored in 'global' variables ```imuMeas```, ```ubloxSen``` and ```vehAtt```
  - In v2.0, the data is now stored in:
  - <strong>UBX_ESF_ALG_t *packetUBXESFALG</strong> contains the IMU alignment information (roll, pitch and yaw)
  - <strong>UBX_ESF_INS_t *packetUBXESFINS</strong> contains the vehicle dynamics information (acceleration and angular rate)
  - <strong>UBX_ESF_MEAS_t *packetUBXESFMEAS</strong> contains the sensor fusion measurements
  - <strong>UBX_ESF_RAW_t *packetUBXESFRAW</strong> contains the raw sensor measurements
  - <strong>UBX_ESF_STATUS_t *packetUBXESFSTATUS</strong> contains the sensor fusion status
  - e.g. ```myGPS.imuMeas.fusionMode``` becomes ```myGNSS.packetUBXESFSTATUS->data.fusionMode```
  - The helper functions ```getSensorFusionMeasurement```, ```getRawSensorMeasurement``` and ```getSensorFusionStatus``` can be used to extract the sensor data for an individual sensor
  - "auto" data can be marked as stale by calling (e.g.) ```myGNSS.flushESFALG()```
  - Please see the [**Dead_Reckoning/Example4_vehicleDynamics**](./examples/Dead_Reckoning/Example4_vehicleDynamics/Example4_vehicleDynamics.ino) example for more details
- In v1, the HNR (High Navigation Rate) data for the Dead Reckoning modules was stored in 'global' variables ```hnrAtt```, ```hnrVehDyn``` and ```hnrPVT```
  - In v2.0, e.g.:
  - ```myGPS.hnrAtt.roll``` becomes ```myGNSS.packetUBXHNRATT->data.roll```
  - ```myGPS.hnrVehDyn.xAccel``` becomes ```myGNSS.packetUBXHNRINS->data.xAccel```
  - ```myGPS.hnrPVT.lat``` becomes ```myGNSS.packetUBXHNRPVT->data.lat```
  - "auto" data can be marked as stale by calling (e.g.) ```myGNSS.flushHNRATT()```
  - Please see the [**Dead_Reckoning/Example6_getAutoHNRData**](./examples/Dead_Reckoning/Example6_getAutoHNRData/Example6_getAutoHNRData.ino) example for more details

Other changes include:
- In v1, NAV_RELPOSNED relPosN, relPosE and relPosD were returned as (float)m. In v2.0 they are returned via <strong>packetUBXNAVRELPOSNED->data.relPosN</strong> (etc.) as (int32_t)cm.
  - New helper functions (```getRelPosN```, ```getRelPosE``` and ```getRelPosD```) provide backward-compatibility
  - Please see the [**ZED-F9P/Example5_RelativePositioningInformation**](./examples/ZED-F9P/Example5_RelativePositioningInformation/Example5_RelativePositioningInformation.ino) example for more details
- In v1, NAV_RELPOSNED accN, accE and accD were returned as (float)m. In v2.0 they are returned via <strong>packetUBXNAVRELPOSNED->data.accN</strong> (etc.) as (uint32_t)mm*0.1.
  - New helper functions (```getRelPosAccN```, ```getRelPosAccE``` and ```getRelPosAccD```) provide backward-compatibility
  - Please see the [**ZED-F9P/Example5_RelativePositioningInformation**](./examples/ZED-F9P/Example5_RelativePositioningInformation/Example5_RelativePositioningInformation.ino) example for more details
- getSurveyStatus now returns data via <strong>UBX_NAV_SVIN_t *packetUBXNAVSVIN</strong>
  - ```myGPS.svin.active``` becomes ```myGNSS.packetUBXNAVSVIN->data.active```
  - ```myGPS.svin.valid``` becomes ```myGNSS.packetUBXNAVSVIN->data.valid```
  - ```myGPS.svin.observationTime``` becomes ```myGNSS.packetUBXNAVSVIN->data.dur``` and is now uint32_t (not uint16_t)
  - ```myGPS.svin.MeanAccuracy``` becomes ```myGNSS.packetUBXNAVSVIN->data.meanAcc``` and is now uint32_t * 0.1mm (not float * m)
  - New helper functions (```getSurveyInActive```, ```getSurveyInValid```, ```getSurveyInObservationTime``` and ```getSurveyInMeanAccuracy```) provide backward-compatibility
  - Please see the [**ZED-F9P/Example3_StartRTCMBase**](./examples/ZED-F9P/Example3_StartRTCMBase/Example3_StartRTCMBase.ino) example for more details
