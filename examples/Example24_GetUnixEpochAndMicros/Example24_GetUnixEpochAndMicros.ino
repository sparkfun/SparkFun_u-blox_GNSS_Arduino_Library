/*
  Getting Unix Epoch Time and micros using u-blox commands
  By: UT2UH
  Date: March 31th, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to query a u-blox module for the current time and date as Unix Epoch uint32_t type to avoid time.h dependency.
  We also turn off the NMEA output on the I2C port. This decreases the amount of I2C traffic dramatically.

  Note: this example works best on modules like the ZED_F9P. Modules like the ZOE_M8Q do not support confirmedTime.

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


long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
  
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

  // Uncomment the next line if you need to completely reset your module
  //myGNSS.factoryDefault(); delay(5000); // Reset everything and wait while the module restarts

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

    // getUnixEpoch marks the PVT data as stale so you will get Unix time and PVT time on alternate seconds

    uint32_t us;  //microseconds returned by getUnixEpoch()
    uint32_t epoch = myGNSS.getUnixEpoch();
    Serial.print("Unix Epoch rounded: ");
    Serial.print(epoch, DEC);    
    epoch = myGNSS.getUnixEpoch(us);
    Serial.print("  Exact Unix Epoch: ");
    Serial.print(epoch, DEC);
    Serial.print("  micros: ");
    Serial.println(us, DEC);

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
    
    Serial.print("  Time is ");
    if (myGNSS.getTimeFullyResolved() == false)
    {
      Serial.print("not fully resolved but ");
    } else {
      Serial.print("fully resolved and ");
    }
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

    byte SIV = myGNSS.getSIV();
    Serial.print(F("  SIV: "));
    Serial.println(SIV);
  }
}