/*
  Set Dynamic Model
  By: Paul Clark (PaulZC)
  Date: April 22nd, 2020

  Based extensively on Example3_GetPosition
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to change the u-blox module's dynamic platform model and then
  query its lat/long/altitude. We also turn off the NMEA output on the I2C port.
  This decreases the amount of I2C traffic dramatically.

  Possible values for the dynamic model are: PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE,
  SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE

  Note: Long/lat are large numbers because they are * 10^7. To convert lat/long
  to something google maps understands simply divide the numbers by 10,000,000. We
  do this so that we don't have to use floating point numbers.

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

  // If we are going to change the dynamic platform model, let's do it here.
  // Possible values are:
  // PORTABLE, STATIONARY, PEDESTRIAN, AUTOMOTIVE, SEA, AIRBORNE1g, AIRBORNE2g, AIRBORNE4g, WRIST, BIKE

  if (myGNSS.setDynamicModel(DYN_MODEL_PORTABLE) == false) // Set the dynamic model to PORTABLE
  {
    Serial.println(F("*** Warning: setDynamicModel failed ***"));
  }
  else
  {
    Serial.println(F("Dynamic platform model changed successfully!"));
  }

  // Let's read the new dynamic model to see if it worked
  uint8_t newDynamicModel = myGNSS.getDynamicModel();
  if (newDynamicModel == DYN_MODEL_UNKNOWN)
  {
    Serial.println(F("*** Warning: getDynamicModel failed ***"));
  }
  else
  {
    Serial.print(F("The new dynamic model is: "));
    Serial.println(newDynamicModel);
  }

  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_NAVCONF); //Uncomment this line to save only the NAV settings to flash and BBR
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

    Serial.println();
  }
}
