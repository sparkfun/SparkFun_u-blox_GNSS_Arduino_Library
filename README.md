# SparkFun u-blox Arduino GNSS Library

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><a href="https://www.sparkfun.com/products/15136"><img src="https://cdn.sparkfun.com//assets/parts/1/3/5/1/4/15136-SparkFun_GPS-RTK2_Board_-_ZED-F9P__Qwiic_-03.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/15005"><img src="https://cdn.sparkfun.com//assets/parts/1/3/3/2/0/15005-SparkFun_GPS-RTK__Qwiic__-_NEO-M8P-2-00.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/15193"><img src="https://cdn.sparkfun.com//assets/parts/1/3/6/1/4/15193-SparkFun_GPS_Breakout_-_U.FL__ZOE-M8__Qwiic_-01.jpg"></a></td>
   <td><a href="https://www.sparkfun.com/products/15210"><img src="https://cdn.sparkfun.com//assets/parts/1/3/6/4/8/15210-SparkFun_GPS_Breakout_-_Chip_Antenna__SAM-M8Q__Qwiic_-01.jpg"></a></td>
    <td><a href="https://www.sparkfun.com/products/15733"><img src="https://cdn.sparkfun.com//assets/parts/1/4/3/2/2/15733-SparkFun_GPS_Breakout_-_NEO-M9N__Chip_Antenna__Qwiic_-01.jpg"></a></td>
  </tr>
  <tr align="center">
    <td><a href="https://www.sparkfun.com/products/15136">SparkFun GPS-RTK2 - ZED-F9P (GPS-15136)</a></td>
    <td><a href="https://www.sparkfun.com/products/15005">SparkFun GPS-RTK - NEO-M8P-2 (GPS-15005)</a></td>
    <td><a href="https://www.sparkfun.com/products/15193">SparkFun ZOE-M8Q Breakout (GPS-15193)</a></td>
    <td><a href="https://www.sparkfun.com/products/15210">SparkFun SAM-M8Q Breakout (GPS-15210)</a></td>
    <td><a href="https://www.sparkfun.com/products/15733">SparkFun NEO-M9N Breakout (GPS-15733)</a></td>
  </tr>
</table>

u-blox makes some incredible GNSS receivers covering everything from low-cost, highly configurable modules such as the SAM-M8Q all the way up to the surveyor grade ZED-F9P with precision of the diameter of a dime. This library supports configuration and control of u-blox devices over I<sup>2</sup>C (called DDC by u-blox), Serial and - as of v2.0.8 (thank you @aberridg) - SPI too! The UBX protocol is a much easier and lighterweight interface to a GNSS module. Stop polling messages and parsing NMEA data! Simply ask for the datums you need and receive an automatic callback when they arrive.

This library can be installed via the Arduino Library manager. Search for **SparkFun u-blox GNSS**.

## Automatic support for correction services like PointPerfect (u-blox), RTK2go, Emlid Caster and Skylark (Swift Navigation)

u-blox's PointPerfect GNSS augmentation service uses the secure MQTT protocol to download SPARTN format correction data, providing "3-6 cm accuracy and convergence within seconds". Please see the new [PointPerfect Client example](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/tree/main/examples/ZED-F9P/Example18_PointPerfectClient) for more details.

v2.2.1 also supports L-band correction services using the new u-blox NEO-D9S correction data receiver. Please see the new [L-band Corrections example](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/tree/main/examples/ZED-F9P/Example19_LBand_Corrections_with_NEO-D9S) for more details.

Other RTK NTRIP corrections services often require you to send them your location in NMEA GPGGA format. v2.2 of the library makes this easy by providing get functions and automatic callbacks
for both GPGGA and GNGGA messages. You can now instruct your module to output GPGGA (e.g.) every 10 seconds and then push it to the correction server directly from the callback. No more polling, no more parsing!

v2.2 also includes two new functions useful for correction services:

* ```setMainTalkerID``` : lets you change the NMEA Talker ID (prefix) from "GN" to "GP" - just in case your correction service really does need GPGGA, not GNGGA
* ```setHighPrecisionMode``` : adds extra decimal places in the GGA messages, increasing the resolution of latitude, longitude and altitude

Please see the new [Automatic_NMEA examples](./examples/Automatic_NMEA) for more details.

We've also added a new [NTRIP Caster Client example](./examples/ZED-F9P/Example17_NTRIPClient_With_GGA_Callback) showing how to use these new features to full effect.

## AssistNow<sup>TM</sup>

v2.1 of the library adds support for u-blox AssistNow<sup>TM</sup> Assisted GNSS (A-GNSS) which can dramatically reduce the time-to-first-fix. You can find further details in the [AssistNow Examples folder](./examples/AssistNow).

## v2 vs. v1

This library is the new and improved version of the very popular SparkFun u-blox GNSS Arduino Library. v2.0 contains some big changes and improvements:

* Seamless support for "automatic" message delivery:
  * In v1.8, you could ask for the NAV PVT (Navigation Position Velocity Time) message to be delivered _automatically_, without polling. v2.0 adds automatic support for [**29 messages**](./Theory.md#auto-messages), covering the full range of: standard and High Precision position, velocity, attitude and time information; relative positioning; event capture with nanosecond time resolution; raw GNSS signal data including carrier phase; Sensor Fusion; and High Navigation Rate data.
  * Don't see the message you really need? [Adding_New_Messages](./Adding_New_Messages.md) provides details on how to add "auto" support for your favourite message.
* Dynamic memory allocation with clearly-defined data storage structs for each message:
  * There are no static 'global' variables to eat up your RAM. v2.0 automatically allocates memory for the automatic messages when they are enabled. You may find your total RAM use is lower with v2.0 than with v1.8.
  * Each "auto" message has a clearly-defined [data storage struct](./src/u-blox_structs.h) which follows the u-blox protocol specification precisely.
* Callbacks:
  * No more polling! Simply request the "auto" messages you need and receive an automatic callback when each message arrives.
  * Please see the [callback examples](./examples/Callbacks) for more details.
* Built-in support for data logging:
  * Want to log RXM SFRBX and RAWX data for Post-Processed Kinematics or Precise Point Positioning? You can absolutely do that! v2.0 provides built-in support for data logging, allowing you to log **any** of the "auto" messages simply and easily.
  * Incoming "auto" data can be stored in a configurable ring buffer. You can then extract the data from the buffer and write it to (e.g.) SD card using your favorite SD library.
  * Data is logged in u-blox UBX format which is compact and efficient. You can replay the data using [u-center](https://www.u-blox.com/en/product/u-center).
  * Please see the [data logging examples](./examples/Data_Logging) for more details.

## Migrating to v2.0

Migrating to v2.0 is easy. There are two small changes all users will need to make:

* The name of the library class has changed from ```SFE_UBLOX_GPS``` to ```SFE_UBLOX_GNSS``` to reflect that the library supports all of the Global Navigation Satellite Systems:
  * As a minimum, you need to change: ```SFE_UBLOX_GPS myGPS;```
  * to: ```SFE_UBLOX_GNSS myGPS;```
  * But we would encourage you to use ```SFE_UBLOX_GNSS myGNSS;```. You will see that all of the library examples now use ```myGNSS``` instead of ```myGPS```.
* The name of the library header and C++ files have changed too:
  * Change: ```#include <SparkFun_Ublox_Arduino_Library.h>```
  * to: ```#include <SparkFun_u-blox_GNSS_Arduino_Library.h>```

If you are using the Dead Reckoning Sensor Fusion or High Dynamic Rate messages, you will need to make more small changes to your code. Please see the [dead reckoning examples](./examples/Dead_Reckoning) for more details. There is more detail available in [Theory.md](./Theory.md#migrating-your-code-to-v20) if you need it.

There is a [new example](./examples/Dead_Reckoning/Example8_getNAVPVAT) showing how to read the UBX-NAV-PVAT (Position, Velocity, Attitude, Time) with a single function call. UBX-NAV-PVAT has full "auto" callback and data-logging support too!

## Memory Usage

The u-blox GNSS library has grown considerably over the years and now exceeds the available program memory on platforms like the ATmega328 (Arduino Uno).
If you want to reduce the amount of memory used by the library, you can edit the header file (_SparkFun_u-blox_GNSS_Arduino_Library.h_) and uncomment lines 60 and 63:

```
#define SFE_UBLOX_REDUCED_PROG_MEM // Uncommenting this line will delete the minor debug messages to save memory
```

```
#define SFE_UBLOX_DISABLE_AUTO_NMEA // Uncommenting this line will disable auto-NMEA support to save memory
```

**Please note:** the debug messages are automatically deleted and auto-NMEA support is automatically disabled on ARDUINO_AVR_UNO platforms. For other platforms, you will need to uncomment those lines manually.

On Windows, you will normally find _SparkFun_u-blox_GNSS_Arduino_Library.h_ in:
- Documents\Arduino\libraries\SparkFun_u-blox_GNSS_Arduino_Library\src

## SPI Support

In v2.0.8 we added support for SPI, based on a contribution by @aberridg. Thank you Andrew!

We have tested the SPI interface on as many platforms and modules as we could pull together. It works perfectly on most but not quite all combinations.
For reasons we don't understand yet, the ZED-F9P and Teensy 3.2 don't seem to get along. But Teensy 3.2 and the ZOE-M8Q do play nicely together.
If you notice a combination that does not seem to work, please raise an [issue](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/issues) and we will investigate.

The SPI examples have their [own folder](./examples/SPI).

Please check the module datasheets for details on what clock speeds and data rates each module supports. The maximum clock speed is typically 5.5MHz and the maximum transfer rate is typically 125kBytes/s.

## I<sup>2</sup>C Support

For I<sup>2</sup>C communication, please be sure to remove all additional pull-ups on the I<sup>2</sup>C bus. u-blox modules include internal pull-ups on the I<sup>2</sup>C lines (sometimes called DDC in their manuals). Cut all I<sup>2</sup>C pull-up jumpers and/or remove them from peripheral boards. Otherwise, various data glitches can occur. See issues [38](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/38) and [40](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40) for more information. We recommend running the I<sup>2</sup>C bus at 100kHz.

## Compatibility

v2 of the library provides support for generation 8, 9 and 10 u-blox GNSS modules. For generation 6 and 7, please see [this example (depricated)](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/tree/master/examples/Series_6_7/Example1_GetPositionAndTime_Series_6_7).

## Contributing

If you would like to contribute to this library: please do, we truly appreciate it, but please follow [these guidelines](./CONTRIBUTING.md). Thanks!

## Repository Contents

* [**/examples**](./examples) - Example sketches for the library (.ino). Run these from the Arduino IDE.
* [**/src**](./src) - Source files for the library (.cpp, .h).
* [**keywords.txt**](./keywords.txt) - Keywords from this library that will be highlighted in the Arduino IDE.
* [**library.properties**](./library.properties) - General library properties for the Arduino package manager.
* [**CONTRIBUTING.md**](./CONTRIBUTING.md) - Guidelines on how to contribute to this library.
* [**Theory.md**](./Theory.md) - provides detail on how data is processed by the library.
* [**/Utils**](./Utils) - contains a Python utility which can check the contents of UBX log files.

## Documentation

* [**Installing an Arduino Library Guide**](https://learn.sparkfun.com/tutorials/installing-an-arduino-library) - Basic information on how to install an Arduino library.

## Theory

If you would like to learn more about how this library works, including the big changes we made in version 2.0, please see [**Theory.md**](./Theory.md) for full details.

## Products That Use This Library

* [GPS-16481](https://www.sparkfun.com/products/16481) - SparkFun GPS-RTK-SMA Breakout - ZED-F9P (Qwiic)
* [GPS-15136](https://www.sparkfun.com/products/15136) - SparkFun GPS-RTK2 Board - ZED-F9P (Qwiic)
* [GPS-16344](https://www.sparkfun.com/products/16344) - SparkFun GPS-RTK Dead Reckoning Breakout - ZED-F9R (Qwiic)
* [GPS-15005](https://www.sparkfun.com/products/15005) - SparkFun GPS-RTK Board - NEO-M8P-2 (Qwiic)
* [GPS-15210](https://www.sparkfun.com/products/15210) - SparkFun GPS Breakout - Chip Antenna, SAM-M8Q (Qwiic)
* [GPS-15193](https://www.sparkfun.com/products/15193) - SparkFun GPS Breakout - Chip Antenna, ZOE-M8Q (Qwiic)
* [GPS-17285](https://www.sparkfun.com/products/17285) - SparkFun GPS Breakout - NEO-M9N, SMA (Qwiic)
* [GPS-15733](https://www.sparkfun.com/products/15733) - SparkFun GPS Breakout - NEO-M9N, Chip Antenna (Qwiic)
* [GPS-15712](https://www.sparkfun.com/products/15712) - SparkFun GPS Breakout - NEO-M9N, U.FL (Qwiic)
* [GPS-16329](https://www.sparkfun.com/products/16329) - SparkFun GPS Dead Reckoning Breakout - NEO-M8U (Qwiic)
* [SPX-14980](https://www.sparkfun.com/products/14980) - SparkX GPS-RTK Black
* [SPX-15106](https://www.sparkfun.com/products/15106) - SparkX SAM-M8Q

## License Information

This product is _**open source**_!

Various bits of the code have different licenses applied. Anything SparkFun wrote is beerware; if you see me (or any other SparkFun employee) at the local, and you've found our code helpful, please buy us a round!

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release anything derivative under the same license.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
