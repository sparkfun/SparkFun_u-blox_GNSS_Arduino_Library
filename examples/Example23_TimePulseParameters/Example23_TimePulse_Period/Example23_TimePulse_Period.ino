/*
  Time Pulse Parameters - Period
  By: Paul Clark (PaulZC)
  Date: January 13th, 2021

  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to change the time pulse parameters and configure the TIMEPULSE (PPS)
  pin to produce a 1 second pulse every 30 seconds. What's really cool is that if you run this
  example on two GNSS boards, the pulses are precisely synchronised!

  The SparkFun GPS-RTK-SMA Breakout - ZED-F9P (Qwiic) (https://www.sparkfun.com/products/16481)
  has solder pads which will let you connect an SMA connector to the TIMEPULSE signal. Need an
  accurate timelapse camera shutter signal? This is the product for you!

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include "SparkFun_u-blox_GNSS_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  // Create storage for the time pulse parameters
  UBX_CFG_TP5_data_t timePulseParameters;

  // Get the time pulse parameters
  if (myGNSS.getTimePulseParameters(&timePulseParameters) == false)
  {
    Serial.println(F("getTimePulseParameters failed! Freezing..."));
    while (1) ; // Do nothing more
  }

  // Print the CFG TP5 version
  Serial.print(F("UBX_CFG_TP5 version: "));
  Serial.println(timePulseParameters.version);

  timePulseParameters.tpIdx = 0; // Select the TIMEPULSE pin
  //timePulseParameters.tpIdx = 1; // Or we could select the TIMEPULSE2 pin instead, if the module has one

  // We can configure the time pulse pin to produce a defined frequency or period
  // Here is how to set the period:

  // Let's say that we want our 1 pulse every 30 seconds to be as accurate as possible. So, let's tell the module
  // to generate no signal while it is _locking_ to GNSS time. We want the signal to start only when the module is
  // _locked_ to GNSS time.
  timePulseParameters.freqPeriod = 0; // Set the frequency/period to zero
  timePulseParameters.pulseLenRatio = 0; // Set the pulse ratio to zero

  // When the module is _locked_ to GNSS time, make it generate a 1 second pulse every 30 seconds
  // (Although the period can be a maximum of 2^32 microseconds (over one hour), the upper limit appears to be around 33 seconds)
  timePulseParameters.freqPeriodLock = 30000000; // Set the period to 30,000,000 us
  timePulseParameters.pulseLenRatioLock = 1000000; // Set the pulse length to 1,000,000 us

  timePulseParameters.flags.bits.active = 1; // Make sure the active flag is set to enable the time pulse. (Set to 0 to disable.)
  timePulseParameters.flags.bits.lockedOtherSet = 1; // Tell the module to use freqPeriod while locking and freqPeriodLock when locked to GNSS time
  timePulseParameters.flags.bits.isFreq = 0; // Tell the module that we want to set the period (not the frequency)
  timePulseParameters.flags.bits.isLength = 1; // Tell the module that pulseLenRatio is a length (in us) - not a duty cycle
  timePulseParameters.flags.bits.polarity = 1; // Tell the module that we want the rising edge at the top of second. (Set to 0 for falling edge.)

  // Now set the time pulse parameters
  if (myGNSS.setTimePulseParameters(&timePulseParameters) == false)
  {
    Serial.println(F("setTimePulseParameters failed!"));
  }
  else
  {
    Serial.println(F("Success!"));
  }

  // Finally, save the time pulse parameters in battery-backed memory so the pulse will automatically restart at power on
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_NAVCONF); // Save the configuration
}

void loop()
{
  // Nothing to do here
}
