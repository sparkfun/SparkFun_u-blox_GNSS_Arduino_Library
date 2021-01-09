/*
  Debug Output
  By: Nathan Seidle, Adapted from Example3_GetPosition by Thorsten von Eicken
  SparkFun Electronics
  Date: January 28rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the debug output from the library.
  Debug shows various packet and status outputs. These prints can be directed
  towards Serial (as in Serial.print) or any other port (Serial1, SerialUSB, etc).

  You can also limit the debug messages to the "critical" ones by adding an extra argument.

  The debug messages can be disabled again by calling disableDebugging()

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

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

unsigned long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
int counter = 0; // Disable the debug messages when counter reaches 20

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  //myGNSS.saveConfiguration(); //Optional: Save the current settings to flash and BBR

  myGNSS.enableDebugging(); //Enable all the debug messages over Serial (default)
  
  //myGNSS.enableDebugging(SerialUSB); //Enable debug messages over Serial USB

  //myGNSS.enableDebugging(Serial, true); //Enable only the critical debug messages over Serial
  
}

void loop()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = myGNSS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.print(F("   "));
    Serial.print(myGNSS.getYear());
    Serial.print(F("-"));
    Serial.print(myGNSS.getMonth());
    Serial.print(F("-"));
    Serial.print(myGNSS.getDay());
    Serial.print(F(" "));
    Serial.print(myGNSS.getHour());
    Serial.print(F(":"));
    Serial.print(myGNSS.getMinute());
    Serial.print(F(":"));
    Serial.println(myGNSS.getSecond());
    
    Serial.println();

    counter++; // Increment counter
    if (counter == 20)
    {
      myGNSS.disableDebugging(); // Disable the debug messages when counter reaches 20
    }
  }
}
