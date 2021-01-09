SparkFun u-blox Arduino Library
===========================================================

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

u-blox makes some incredible GNSS receivers covering everything from low-cost, highly configurable modules such as the SAM-M8Q all the way up to the surveyor grade ZED-F9P with precision of the diameter of a dime. This library focuses on configuration and control of u-blox devices over I2C (called DDC by u-blox) and Serial. The UBX protocol is supported over both I2C and serial, and is a much easier and lighterweight interface to a GNSS module. Stop parsing NMEA data! And simply ask for the datums you need.

This library can be installed via the Arduino Library manager. Search for **SparkFun u-blox GNSS**.

Although not an integrated part of the library, you will find an example of how to communicate with the older series 6 and 7 modules in the [examples folder](./examples/Series_6_7).

Max (400kHz) I2C Support
-------------------

To achieve 400kHz I2C speed please be sure to remove all pull-ups on the I2C bus. Most, if not all, u-blox modules include pull ups on the I2C lines (sometimes called DDC in their manuals). Cut all I2C pull up jumpers and/or remove them from peripheral boards. Otherwise, various data glitches can occur. See issues [38](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/38) and [40](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/issues/40) for more information. If possible, run the I2C bus at 100kHz.

-------------------

Want to help? Please do! We are always looking for ways to improve and build out features of this library.

* We are always interested in adding SPI support with a checkUbloxSPI() function

Thanks to:

* [trycoon](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library/pull/7) for fixing the lack of I2C buffer length defines.
* [tve](https://github.com/tve) for building out serial additions and examples.
* [Redstoned](https://github.com/Redstoned) and [davidallenmann](https://github.com/davidallenmann) for adding PVT date and time.
* [wittend](https://forum.sparkfun.com/viewtopic.php?t=49874) for pointing out the RTCM print bug.
* Big thanks to [PaulZC](https://github.com/PaulZC) for implementing the combined key ValSet method, geofence functions, better saveConfig handling, as well as a bunch of small fixes.
* [RollieRowland](https://github.com/RollieRowland) for adding HPPOSLLH (High Precision Geodetic Position).
* [tedder](https://github.com/tedder) for moving iTOW to PVT instead of HPPOS and comment cleanup.
* [grexjmo](https://github.com/grexjmo) for pushing for a better NMEA sentence configuration method.
* [averywallis](https://github.com/averywallis) for adding good comments to the various constants.
* [blazczak](https://github.com/blazczak) and [geeksville](https://github.com/geeksville) for adding support for the series 6 and 7 modules.
* [bjorn@unsurv](https://github.com/unsurv) for adding powerOff and powerOffWithInterrupt.
* [dotMorten](https://github.com/dotMorten) for the MSGOUT keys, autoHPPOSLLH, autoDOP and upgrades to autoPVT.
* [markuckermann](https://github.com/markuckermann) for spotting the config layer gremlins
* [vid553](https://github.com/vid553) for the Zephyr port
* [balamuruganky](https://github.com/balamuruganky) for the NAV-PVT velocity parameters, getSpeedAccEst, getHeadingAccEst, getInvalidLlh, getHeadVeh, getMagDec and getMagAcc
* [nelarsen](https://github.com/nelarsen) for the buffer overrun improvements
* [mstranne](https://github.com/mstranne) and [shaneperera](https://github.com/shaneperera) for the pushRawData suggestion
* [rubienr](https://github.com/rubienr) for spotting the logical AND issues

Need a Python version for Raspberry Pi? Checkout the [Qwiic Ublox GPS Py module](https://github.com/sparkfun/Qwiic_Ublox_Gps_Py).

Need a library for the u-blox and Particle? Checkout the [Particle library](https://github.com/aseelye/SparkFun_Ublox_Particle_Library) fork.

Contributing
--------------

If you would like to contribute to this library: please do, we truly appreciate it, but please follow [these guidelines](./CONTRIBUTING.md). Thanks!

Repository Contents
-------------------

* **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE.
* **/src** - Source files for the library (.cpp, .h).
* **[keywords.txt](./keywords.txt)** - Keywords from this library that will be highlighted in the Arduino IDE.
* **[library.properties](./library.properties)** - General library properties for the Arduino package manager.
* **[CONTRIBUTING.md](./CONTRIBUTING.md)** - Guidelines on how to contribute to this library.
* **[Theory.md](./Theory.md)** - provides detail on how data is processed by the library.

Documentation
--------------

* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.

Theory
--------------

If you would like to learn more about how this library works, including the big changes we made in version 2.0, please see **[Theory.md](./Theory.md)** for full details.

Products That Use This Library
---------------------------------
* [GPS-16481](https://www.sparkfun.com/products/16481) - SparkFun GPS-RTK-SMA Breakout - ZED-F9P (Qwiic)
* [GPS-15136](https://www.sparkfun.com/products/15136) - SparkFun GPS-RTK2 Board - ZED-F9P (Qwiic)
* [GPS-16344](https://www.sparkfun.com/products/16344) - SparkFun GPS-RTK Dead Reckoning Breakout - ZED-F9R (Qwiic)
* [GPS-15005](https://www.sparkfun.com/products/15005) - SparkFun GPS-RTK Board - NEO-M8P-2 (Qwiic)
* [GPS-15210](https://www.sparkfun.com/products/15210) - SparkFun GPS Breakout - Chip Antenna, SAM-M8Q (Qwiic)
* [GPS-15193](https://www.sparkfun.com/products/15193) - SparkFun GPS Breakout - Chip Antenna, ZOE-M8Q (Qwiic)
* [GPS-15733](https://www.sparkfun.com/products/15733) - SparkFun GPS Breakout - NEO-M9N, Chip Antenna (Qwiic)
* [GPS-15712](https://www.sparkfun.com/products/15712) - SparkFun GPS Breakout - NEO-M9N, U.FL (Qwiic)
* [GPS-16329](https://www.sparkfun.com/products/16329) - SparkFun GPS Dead Reckoning Breakout - NEO-M8U (Qwiic)
* [SPX-14980](https://www.sparkfun.com/products/14980) - SparkX GPS-RTK Black
* [SPX-15106](https://www.sparkfun.com/products/15106) - SparkX SAM-M8Q

License Information
-------------------

This product is _**open source**_!

Various bits of the code have different licenses applied. Anything SparkFun wrote is beerware; if you see me (or any other SparkFun employee) at the local, and you've found our code helpful, please buy us a round!

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release anything derivative under the same license.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
