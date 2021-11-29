# SparkFun u-blox Arduino GNSS Library - AssistNow<sup>TM</sup>

v2.1.0 of the library adds support for u-blox [AssistNow<sup>TM</sup> Assisted GNSS (A-GNSS)](https://www.u-blox.com/en/product/assistnow) which can dramatically reduce the time-to-first-fix.

To use AssistNow Online or AssistNow Offline, you will need a token to access the u-blox Thingstream server. See [below](#AssistNow-Service-Token) for details.

## AssistNow<sup>TM</sup> Online

With AssistNow Online, an Internet connected host downloads assistance data from the u-blox AssistNow Online service to the receiver at system start-up. AssistNow Online data is valid for 2 - 4 hours; beyond that fresh data must be downloaded.

Please see the [AssistNow_Online](./AssistNow_Online) examples for more details. These examples were written for the ESP32, but will run on other platforms too.

The new functions we've added to the library to support AssistNow Online are described [Code Support for AssistNow below](#Code-Support-for-AssistNow)

## AssistNow<sup>TM</sup> Offline

With the AssistNow Offline service, users can download long-term orbit data over the Internet at their convenience. The orbit data can be stored in the memory of the application processor. The function requires no connectivity at system start-up, enabling a position fix within seconds, even when no network is available. AssistNow Offline offers augmentation for up to 35 days.

Please see the [AssistNow_Offline](./AssistNow_Offline) examples for more details. These examples were written for the ESP32, but will run on other platforms too.

**Note: AssistNow Offline is not supported by the ZED-F9P. "The ZED-F9P supports AssistNow Online only."**

The new functions we've added to the library to support AssistNow Offline are described [Code Support for AssistNow below](#Code-Support-for-AssistNow)

## AssistNow<sup>TM</sup> Autonomous

AssistNow Autonomous provides aiding information without the need for a host or external network connection. Based on previous broadcast satellite ephemeris data downloaded to and stored by the GNSS receiver, AssistNow Autonomous automatically generates accurate predictions of satellite orbital data (“AssistNow Autonomous data”) that is usable for future GNSS position fixes.

The benefits of AssistNow Autonomous are:

* Faster fix in situations where GNSS satellite signals are weak
* No connectivity required
* Compatible with AssistNow Online (can work stand-alone, or in tandem with AssistNow Online service)
* No integration effort; calculations are done in the background, transparent to the user

AssistNow Autonomous offers augmentation for up to 6 days.

Please see the [AssistNow_Autonomous](./AssistNow_Autonomous) examples for more details.

The new functions we've added to the library to support AssistNow Autonomous are described [Code Support for AssistNow below](#Code-Support-for-AssistNow)

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

## Code Support for AssistNow

```pushAssistNowData``` allows the AssistNow Online data or AssistNow Offline data from the u-blox server to be pushed to the module. As the ESP32 HTTP GET function returns a ```String```, we've included overloaded functions which allow you to pass the data as a ```String``` or as ```const uint8_t *```.

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

## Additional Code Support for AssistNow Offline

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

