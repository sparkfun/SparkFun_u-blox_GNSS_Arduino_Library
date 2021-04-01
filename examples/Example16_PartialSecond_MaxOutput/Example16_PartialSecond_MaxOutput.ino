/*
  Getting time and date using u-blox commands
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 16th, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to use the Millisecond and Nanosecond output as well as increase the
  I2C speed (100 to 400kHz), and serial output (115200 to 500kbps).

  Note: you will need to set your Serial Monitor to 500000 Baud to see the output

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
  Serial.begin(500000); //Increase serial speed to maximize
  while (!Serial)
    ; //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();
  Wire.setClock(400000); // Increase I2C clock speed to 400kHz

  //myGNSS.enableDebugging(); //Uncomment this line to enable debug messages over Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  // Note: not all u-blox modules can output solutions at 10Hz - or not while tracking all satellite constellations
  // If the rate drops back to 1Hz, you're asking too much of your module
  myGNSS.setNavigationFrequency(10);           //Set output to 10 times a second
  
  byte rate = myGNSS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate:");
  Serial.println(rate);

  //myGNSS.saveConfiguration(); //Optional: Save the current settings to flash and BBR
}

void loop()
{
  // Calling getPVT returns true if there actually is a fresh navigation solution available.
  if (myGNSS.getPVT())
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

    Serial.print(" ");
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
    Serial.print(".");
    //Pretty print leading zeros
    int mseconds = myGNSS.getMillisecond();
    if (mseconds < 100)
      Serial.print("0");
    if (mseconds < 10)
      Serial.print("0");
    Serial.print(mseconds);

    Serial.print(" nanoseconds: ");
    Serial.print(myGNSS.getNanosecond());

    Serial.println();
  }
}
