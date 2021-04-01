/*
  Demonstrating how to use "end"
  By: Paul Clark
  SparkFun Electronics
  Date: April 1st, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to use the end function.
  End will stop all automatic message processing and free (nearly) all used RAM.
  The file buffer is deleted (if it exists).
  
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

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  //myGNSS.saveConfiguration(); //Optional: Save the current settings to flash and BBR

  myGNSS.end(); // Call end now just because we can - it won't do much as we haven't used any automatic messages
}

void loop()
{
  // Allocate 128 bytes for file storage - this checks that issue #20 has been resolved
  // Allocating only 128 bytes will let this code run on the ATmega328P
  // If your processor has plenty of RAM, you can increase this to something useful like 16KB
  myGNSS.setFileBufferSize(128);
  
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected. Freezing."));
    while (1);
  }

  Serial.print(F("The file buffer size is: "));
  Serial.println(myGNSS.getFileBufferSize());    

  // Request Position, Velocity, Time
  // RAM will be allocated for PVT message processing
  // getPVT will return true is fresh PVT data was received and processed
  if (myGNSS.getPVT())
  {
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
    Serial.println(F(" (mm)"));
  }

  // Calling end will free the RAM allocated for file storage and PVT processing
  // Calling end is optional. You can comment the next line if you want to.
  myGNSS.end();
}
