# SparkFun u-blox Arduino GNSS Library - AssistNow<sup>TM</sup>

v2.1.0 of the library adds support for u-blox [AssistNow<sup>TM</sup> Assisted GNSS (A-GNSS)](https://www.u-blox.com/en/product/assistnow) which can dramatically reduce the time-to-first-fix.

To use AssistNow Online or AssistNow Offline, you will need a token to access the u-blox Thingstream server. See [below](#AssistNow-Service-Token) for details.

## AssistNow<sup>TM</sup> Online

With AssistNow Online, an Internet connected host downloads assistance data from the u-blox AssistNow Online service to the receiver at system start-up. AssistNow Online data is valid for 2 - 4 hours; beyond that fresh data must be downloaded.

Please see the [AssistNow_Online](./AssistNow_Online) examples for more details. These examples were written for the ESP32, but will run on other platforms too.

## AssistNow<sup>TM</sup> Offline

With the AssistNow Offline service, users can download long-term orbit data over the Internet at their convenience. The orbit data can be stored in the memory of the application processor. The function requires no connectivity at system start-up, enabling a position fix within seconds, even when no network is available. AssistNow Offline offers augmentation for up to 35 days.

Please see the [AssistNow_Offline](./AssistNow_Offline) examples for more details. These examples were written for the ESP32, but will run on other platforms too.

## AssistNow<sup>TM</sup> Autonomous

AssistNow Autonomous provides aiding information without the need for a host or external network connection. Based on previous broadcast satellite ephemeris data downloaded to and stored by the GNSS receiver, AssistNow Autonomous automatically generates accurate predictions of satellite orbital data (“AssistNow Autonomous data”) that is usable for future GNSS position fixes.

The benefits of AssistNow Autonomous are:

* Faster fix in situations where GNSS satellite signals are weak
* No connectivity required
* Compatible with AssistNow Online (can work stand-alone, or in tandem with AssistNow Online service)
* No integration effort; calculations are done in the background, transparent to the user

AssistNow Autonomous offers augmentation for up to 6 days.

Please see the [AssistNow_Autonomous](./AssistNow_Autonomous) examples for more details.

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
