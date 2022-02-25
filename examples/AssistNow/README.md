# SparkFun u-blox Arduino GNSS Library - AssistNow<sup>TM</sup>

v2.1.0 of the library adds support for u-blox [AssistNow<sup>TM</sup> Assisted GNSS (A-GNSS)](https://www.u-blox.com/en/product/assistnow) which can dramatically reduce the time-to-first-fix.

To use AssistNow Online or AssistNow Offline, you will need a token to access the u-blox Thingstream server. See [below](#AssistNow-Service-Token) for details.

## AssistNow<sup>TM</sup> Online

With AssistNow Online, an Internet connected host downloads assistance data from the u-blox AssistNow Online service to the receiver at system start-up. AssistNow Online data is valid for 2 - 4 hours; beyond that fresh data must be downloaded.

Please see the [AssistNow_Online](./AssistNow_Online) examples for more details. These examples were written for the ESP32, but will run on other platforms too.

The new functions we've added to the library to support AssistNow Online are described [Support for AssistNow below](#Support-for-AssistNow).

## AssistNow<sup>TM</sup> Offline

With the AssistNow Offline service, users can download long-term orbit data over the Internet at their convenience. The orbit data can be stored in the memory of the application processor. The function requires no connectivity at system start-up, enabling a position fix within seconds, even when no network is available. AssistNow Offline offers augmentation for up to 35 days.

Please see the [AssistNow_Offline](./AssistNow_Offline) examples for more details. These examples were written for the ESP32, but will run on other platforms too.

**Note: AssistNow Offline is not supported by the ZED-F9P. "The ZED-F9P supports AssistNow Online only."**

The new functions we've added to the library to support AssistNow Offline are described [Support for AssistNow](#Support-for-AssistNow) and [Additional Support for AssistNow Offline](#Additional-Support-for-AssistNow-Offline).

## AssistNow<sup>TM</sup> Autonomous

AssistNow Autonomous provides aiding information without the need for a host or external network connection. Based on previous broadcast satellite ephemeris data downloaded to and stored by the GNSS receiver, AssistNow Autonomous automatically generates accurate predictions of satellite orbital data (“AssistNow Autonomous data”) that is usable for future GNSS position fixes.

The benefits of AssistNow Autonomous are:

* Faster fix in situations where GNSS satellite signals are weak
* No connectivity required
* Compatible with AssistNow Online (can work stand-alone, or in tandem with AssistNow Online service)
* No integration effort; calculations are done in the background, transparent to the user

AssistNow Autonomous offers augmentation for up to 6 days.

Please see the [AssistNow_Autonomous](./AssistNow_Autonomous) examples for more details.

**Note: AssistNow Autonomous does not work on the ZED-F9P. "The ZED-F9P supports AssistNow Online only."**

The new functions we've added to the library to support AssistNow Autonomous are described [Support for AssistNow Autonomous below](#Support-for-AssistNow-Autonomous).

## AssistNow Service Token

To be able to use AssistNow Online or AssistNow Offline, you will need a token to access the u-blox Thingstream server.

The following u-blox resources contain useful information:

* [AssistNow - u-blox A-GNSS services](https://www.u-blox.com/en/product/assistnow)
* [AssistNow Product Summary](https://www.u-blox.com/sites/default/files/products/documents/AssistNow_ProductSummary_UBX-13003352.pdf)
* [AssistNow User Guide](https://www.u-blox.com/sites/default/files/products/documents/MultiGNSS-Assistance_UserGuide_%28UBX-13004360%29.pdf)
* [Thingstream Pricing](https://portal.thingstream.io/pricing)

You can apply for a _free_ AssistNow Service Evaluation Token by completing the request form:

* [AssistNow Service evaluation token request form](https://www.u-blox.com/en/assistnow-service-evaluation-token-request-form)

The _free_ AssistNow Developer token entitles you to:

* AssistNow Online Developer: 100K free location requests per month. Capped.
* AssistNow Offline Developer: 20K free location requests per month. Capped.
* CellLocate Developer: 5K free location requests per month. Capped.

The free token will expire after 90 days, but you can continue to use it beyond that by registering it on [Thingstream](https://portal.thingstream.io/).

## Initial Position Assistance

You can further decrease the time-to-first-fix by providing the receiver's approximate position - if known. There are two ways to do this:

* The position can be specified when requesting AssistNow Online data from the server:
  * include the key name ```lat``` with the approximate user latitude in WGS 84 expressed in degrees and fractional degrees. Must be in range -90 to 90. Example: ```lat=47.2;```
  * include the key name ```lon``` with the approximate user longitude in WGS 84 expressed in degrees and fractional degrees. Must be in range -180 to 180. Example: ```lon=8.55;```
  * include the key name ```alt``` with the approximate user altitude above WGS 84 Ellipsoid in meters. If this value is not provided, the server assumes an altitude of 0 meters. Must be in range -1000 to 50000
  * include the key name ```pacc``` with the approximate accuracy of submitted position in meters. If this value is not provided, the server assumes an accuracy of 300 km. Must be in range 0 to 6000000
  * the position assistance data will then be automatically included in the AssistNow Online data
* Provide initial position assistance data by calling one of:
  * <b>bool setPositionAssistanceXYZ(int32_t ecefX, int32_t ecefY, int32_t ecefZ, uint32_t posAcc, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait);</b>
  * The units for ```ecefX/Y/Z``` and ```posAcc``` (stddev) are cm
  * <b>bool setPositionAssistanceLLH(int32_t lat, int32_t lon, int32_t alt, uint32_t posAcc, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait);</b>
  * The units for ```lat``` and ```lon``` are degrees * 1e-7 (WGS84). The units for ```alt``` (WGS84) and ```posAcc``` (stddev) are cm (not m)

## Support for AssistNow

```pushAssistNowData``` allows AssistNow Online, Offline or Autonomous data to be pushed to the module. As the ESP32 HTTP GET function returns a ```String```, we've included overloaded functions which allow you to pass the data as a ```String``` or as ```const uint8_t *```.

The String-based function declarations are:

* <b>size_t pushAssistNowData(const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait);</b>
* <b>size_t pushAssistNowData(bool skipTime, const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait);</b>
* <b>size_t pushAssistNowData(size_t offset, bool skipTime, const String &dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait);</b>

The const uint8_t * function declarations are:

* <b>size_t pushAssistNowData(const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait);</b>
* <b>size_t pushAssistNowData(bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait);</b>
* <b>size_t pushAssistNowData(size_t offset, bool skipTime, const uint8_t *dataBytes, size_t numDataBytes, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait);</b>

```dataBytes``` is a pointer to the AssistNow data.
<br>
```numDataBytes``` is the length of the AssistNow data.
<br>

```pushAssistNowData``` pushes individual packets of data to the u-blox module. Sending all of the data contiguously would overload the module, so ```pushAssistNowData``` can either:

* insert a small delay between each packet (the default is 7ms)
* or use the ```UBX-MGA-ACK-DATA0``` acknowledgement message to acknowledge each packet

```mgaAck``` controls which method is used.

* if ```mgaAck``` is ```SFE_UBLOX_MGA_ASSIST_ACK_NO``` (**default**), a delay of ```maxWait``` milliseconds is inserted between each packet. ```maxWait``` defaults to 7ms.
* if ```mgaAck``` is ```SFE_UBLOX_MGA_ASSIST_ACK_YES```, acknowledgement messages will be expected with a _timeout_ of ```maxWait``` milliseconds. The default timeout is again 7ms, but you can change this if required by passing a different value.
* if ```mgaAck``` is ```SFE_UBLOX_MGA_ASSIST_ACK_ENQUIRE```, the code will poll the module to enquire if the acknowledgement messages are enabled. If they are, they will be used. If not, a delay is used.

```setAckAiding``` enables or disables the acknowledgement messages. By default they are disabled. ```setAckAiding(1)``` will enable them. ```setAckAiding(0)``` will disable them again.

* <b>bool setAckAiding(uint8_t ackAiding, uint16_t maxWait);</b>

```getAckAiding``` returns 1 if the acknowledgement messages are enabled, 0 if they are disabled. 255 indicates an error or timeout.

* <b>uint8_t getAckAiding(uint16_t maxWait);</b>

```pushAssistNowData``` returns the number of _bytes_ pushed (not the number of _packets_). The return value should be equal to ```numDataBytes``` if all data was valid and pushed successfully.

AssistNow Online data is valid for 2-4 hours. 'Stale' data can be re-used but:

* ```pushAssistNowData``` needs to be told to skip the time information contained in the AssistNow data
* the user needs to provide the module with UTC time separately

The ```skipTime``` parameter tells ```pushAssistNowData``` to skip any time information in the data. ```skipTime``` is bool. Set it to ```true``` to skip the time information.
<br>

UTC time can be pushed to the module first by calling ```setUTCTimeAssistance```:

* <b>bool setUTCTimeAssistance(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint32_t nanos, uint16_t tAccS, uint32_t tAccNs, uint8_t source, sfe_ublox_mga_assist_ack_e mgaAck, uint16_t maxWait);</b>

Only the ```year```, ```month```, ```day```, ```hour```, ```minute``` and ```second``` parameters are mandatory. The others default to sensible values. Again ```mgaAck``` and ```maxWait``` control if a delay is used when configuring the time, or if an acknowledgement message will be expected.
<br>

```nanos``` (nanoseconds), ```tAccS``` (time accuracy estimate (seconds)), ```tAccNs``` (time accuracy estimate (nanoseconds)) and ```source``` (if a clock signal will be provided on EXT_INT) are optional, but are available for advanced users.
<br>

```year``` numbering starts at 0; 2021 is 2021, not 121 (years since 1900). ```month``` and ```day``` numbering starts at 1, not 0.
<br>

Call ```setUTCTimeAssistance``` _before_ ```pushAssistNowData```.

## Additional Support for AssistNow Offline

AssistNow Offline data downloaded from the u-blox server can contain 1-5 weeks of data. However, only the data for _today_ should be pushed the module. Sending data for past or future days will confuse the module.
```findMGAANOForDate``` can be used to find the location of the start of the UBX-MGA-ANO data for the specified date within the offline data. That location can then be passed to ```pushAssistNowData``` using the ```offset``` parameter.

* <b>size_t findMGAANOForDate(const uint8_t *dataBytes, size_t numDataBytes, uint16_t year, uint8_t month, uint8_t day, uint8_t daysIntoFuture);</b>

The sequence of events is:

* call ```findMGAANOForDate``` passing the ```year```, ```month``` and ```day``` for today. ```findMGAANOForDate``` will return the location / offset of the data for today within the offline data.
* call ```findMGAANOForDate``` again passing the ```year```, ```month``` and ```day``` for _today_ but also set ```daysIntoFuture``` to 1. ```findMGAANOForDate``` will then return the location / offset of the data for _tomorrow_ (one day into the future).
* call ```pushAssistNowData``` setting:
  * ```offset``` to the location (offset) of today's data within the offline data
  * ```skipTime``` to ```true```
  * ```numDataBytes``` to ((tomorrow's location) - (today's location)). Only the offline data for today will be pushed.

```findMGAANOForDate``` will return a value of numDataBytes if the data for the chosen day cannot be found.
<br>

Again, call ```setUTCTimeAssistance``` _before_ ```pushAssistNowData```.

## Support for AssistNow Autonomous

AssistNow Autonomous is disabled by default. You can enable it by calling ```setAopCfg``` and check if it is enabled by calling ```getAopCfg```:

* <b>uint8_t getAopCfg(uint16_t maxWait);</b>
* <b>bool SFE_UBLOX_GNSS::setAopCfg(uint8_t aopCfg, uint16_t aopOrbMaxErr, uint16_t maxWait)</b>

```getAopCfg``` will return 1 if AssistNow Autonomous is enabled, 0 if disabled. 255 indicates an error or timeout.

```setAopCfg``` has two parameters:

* set ```aopCfg``` to 1 to enable AssistNow Autonomous, or 0 to disable it
* ```aopOrbMaxErr``` is used to set the 'lifetime' of the AssistNow data. It is recommended to set aopOrbMaxErr to 0 (the default value). This instructs the module to use the firmware default value that corresponds to a default orbit data validity of approximately three days (for GPS satellites observed once) and up to six days (for GPS and GLONASS satellites observed multiple times over a period of at least half a day).

Once AssistNow Autonomous is enabled, you can monitor its status via the ```status``` field in the UBX-NAV-AOPSTATUS message. You can read the ```status``` by calling the helper function ```getAOPSTATUSstatus```. It will return zero when the AssistNow Autonomous data collection is idle. Non-zero values indicate that data collection is in progress. Only power-off the receiver when the subsystem is idle (that is, when the status shows a steady zero).

* <b>uint8_t getAOPSTATUSstatus(uint16_t maxWait);</b>
* <b>uint8_t getAOPSTATUSuseAOP(uint16_t maxWait);</b>

We have included full 'auto' support for UBX-NAV-AOPSTATUS, so you can have the message delivered periodically, add a callback for it, and/or log it to the file buffer:

* <b>bool getAOPSTATUS(uint16_t maxWait);</b>
* <b>bool setAutoAOPSTATUS(bool enabled, uint16_t maxWait);</b>
* <b>bool setAutoAOPSTATUS(bool enabled, bool implicitUpdate, uint16_t maxWait);</b>
* <b>bool setAutoAOPSTATUSrate(uint8_t rate, bool implicitUpdate, uint16_t maxWait);</b>
* <b>bool setAutoAOPSTATUScallback(void (*callbackPointer)(UBX_NAV_AOPSTATUS_data_t), uint16_t maxWait);</b>
* <b>bool assumeAutoAOPSTATUS(bool enabled, bool implicitUpdate);</b>
* <b>void flushAOPSTATUS();</b>
* <b>void logAOPSTATUS(bool enabled);</b>

You can also monitor the AssistNow Autonomous satellite information via the UBX-NAV-SAT message. Again, we have included full 'auto' support for UBX-NAV-SAT. UBX-NAV-SAT contains useful information for each individual satellite which the module has acquired: carrier to noise ratio (signal strength); elevation; azimuth; pseudorange residual; quality indication, health; ephemeris available; almanac available; **AssistNow Offline data availability**; and more. The data can be analyzed using a callback. Please see the AssistNowAutonomous examples for more details.

* <b>bool getNAVSAT(uint16_t maxWait);</b>
* <b>bool setAutoNAVSAT(bool enabled, uint16_t maxWait);</b>
* <b>bool setAutoNAVSAT(bool enabled, bool implicitUpdate, uint16_t maxWait);</b>
* <b>bool setAutoNAVSATrate(uint8_t rate, bool implicitUpdate = true, uint16_t maxWait);</b>
* <b>bool setAutoNAVSATcallback(void (*callbackPointer)(UBX_NAV_NAVSAT_data_t), uint16_t maxWait);</b>
* <b>bool assumeAutoNAVSAT(bool enabled, bool implicitUpdate);</b>
* <b>void flushNAVSAT();</b>
* <b>void logNAVSAT(bool enabled);</b>

The AssistNow Autonomous data is stored in the module's RAM memory. If that RAM is Battery-Backed - all SparkFun GNSS boards include battery back-up - then the data will be available after the module is powered down and powered back up again. However, you can also read (poll) the navigation database and store the contents in processor memory. ```readNavigationDatabase``` allows you to do that:

* <b>size_t readNavigationDatabase(uint8_t *dataBytes, size_t maxNumDataBytes, uint16_t maxWait);</b>

Data is written to ```dataBytes```. Set ```maxNumDataBytes``` to the (maximum) size of dataBytes. If the database exceeds maxNumDataBytes, the excess bytes will be lost.

```readNavigationDatabase``` returns the number of database bytes written to ```dataBytes```. The return value will be equal to ```maxNumDataBytes``` if excess data was received.

```readNavigationDatabase```  will timeout after ```maxWait``` milliseconds - in case the final UBX-MGA-ACK was missed.

You can then write the database back into the module using ```pushAssistNowData```. Don't forget to call ```setUTCTimeAssistance``` _before_ ```pushAssistNowData```.

Note: UBX-MGA-DBD messages are only intended to be sent back to the same receiver that generated them. They are firmware-specific.
