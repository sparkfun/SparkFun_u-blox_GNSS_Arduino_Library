/*
  Getting Unix Epoch Time and micros using u-blox commands
  By: UT2UH
  Date: March 30th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to query a u-blox module for the current time and date as Unix Epoch uint32_t type to avoid time.h dependency.
  We also turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic dramatically.

  Leave NMEA parsing behind. Now you can simply ask the module for the datums you want!

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

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include <ezTime.h> //https://github.com/ropg/ezTime
#include <WiFi.h>

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

uint32_t us;  //microseconds returned by getUnixEpoch()
  
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();
  
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  //myGNSS.saveConfiguration();        //Optional: Save the current settings to flash and BBR

  Serial.println("Compare Unix Epoch given with reference one from https://www.epochconverter.com/");

}

void loop()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer

    byte SIV = myGNSS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.print("  ");
    Serial.print(myGNSS.getYear());
    Serial.print("-");
    Serial.print(myGNSS.getMonth());
    Serial.print("-");
    Serial.print(myGNSS.getDay());
    Serial.print(" ");
    Serial.print(myGNSS.getHour());
    Serial.print(":");
    Serial.print(myGNSS.getMinute());
    Serial.print(":");
    Serial.print(myGNSS.getSecond());
    Serial.print("  makeTime(tm): ");
    Serial.print(makeTime(myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(), myGNSS.getDay(), myGNSS.getMonth(), myGNSS.getYear()));
    Serial.print(" micros: ");
    Serial.print((int32_t)(myGNSS.getNanosecond() / 1000));
    Serial.print("  getUnixEpoch(micros): ");
    Serial.print(myGNSS.getUnixEpoch(us));
    Serial.print("  micros: ");
    Serial.print(us, DEC);
    
    Serial.print("  Time is ");
    if (myGNSS.getTimeValid() == false)
    {
      Serial.print("not ");
    }
    Serial.print("valid ");
    if (myGNSS.getConfirmedTime() == false)
    {
      Serial.print("but not ");
    } else {
      Serial.print("and ");
    }
    Serial.print("confirmed");

    Serial.println();
  }
}