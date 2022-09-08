/*
  Get the high precision ECEF coordinates using double
  By: Paul Clark
  SparkFun Electronics
  Date: September 8th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read the high-precision ECEF
  positional solution. Please see below for information about the units.

  ** This example will only work correctly on platforms which support 64-bit double **

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and (e.g.) a Redboard Artemis https://www.sparkfun.com/products/15444
  or an Artemis Thing Plus https://www.sparkfun.com/products/15574
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> // Needed for I2C to GNSS

#define myWire Wire // This will work on the Redboard Artemis and the Artemis Thing Plus using Qwiic
//#define myWire Wire1 // Uncomment this line if you are using the extra SCL1/SDA1 pins (D17 and D16) on the Thing Plus

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal

  myWire.begin();

  //myGNSS.enableDebugging(Serial); // Uncomment this line to enable debug messages

  if (myGNSS.begin(myWire) == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  // Check that this platform supports 64-bit (8 byte) double
  if (sizeof(double) < 8)
  {
    Serial.println(F("Warning! Your platform does not support 64-bit double."));
    Serial.println(F("The ECEF coordinates will be inaccurate."));
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  //myGNSS.saveConfiguration(); //Save the current settings to flash and BBR
}

void loop()
{
  //Query module only every second.
  //The module only responds when a new position is available.
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer

    // getHighResECEFX: returns the X coordinate from HPPOSECEF as an int32_t in cm
    // getHighResECEFXHp: returns the high resolution component of the X coordinate from HPPOSECEF as an int8_t in mm*10^-1 (0.1mm)
    // getHighResECEFY: returns the Y coordinate from HPPOSECEF as an int32_t in cm
    // getHighResECEFYHp: returns the high resolution component of the Y coordinate from HPPOSECEF as an int8_t in mm*10^-1 (0.1mm)
    // getHighResECEFZ: returns the Z coordinate from HPPOSECEF as an int32_t in cm
    // getHighResECEFZHp: returns the high resolution component of the Z coordinate from HPPOSECEF as an int8_t in mm*10^-1 (0.1mm)
    // getPositionAccuracy: returns the position accuracy estimate from HPPOSLLH as an uint32_t in mm (note: not 0.1mm)

    // First, let's collect the position data
    int32_t ECEFX = myGNSS.getHighResECEFX();
    int8_t ECEFXHp = myGNSS.getHighResECEFXHp();
    int32_t ECEFY = myGNSS.getHighResECEFY();
    int8_t ECEFYHp = myGNSS.getHighResECEFYHp();
    int32_t ECEFZ = myGNSS.getHighResECEFZ();
    int8_t ECEFZHp = myGNSS.getHighResECEFZHp();
    uint32_t accuracy = myGNSS.getPositionAccuracy();

    // Defines storage for the ECEF coordinates as double
    double d_ECEFX;
    double d_ECEFY;
    double d_ECEFZ;

    // Assemble the high precision coordinates
    d_ECEFX = ((double)ECEFX) / 100.0; // Convert from cm to m
    d_ECEFX += ((double)ECEFXHp) / 10000.0; // Now add the high resolution component ( mm * 10^-1 = m * 10^-4 )
    d_ECEFY = ((double)ECEFY) / 100.0; // Convert from cm to m
    d_ECEFY += ((double)ECEFYHp) / 10000.0; // Now add the high resolution component ( mm * 10^-1 = m * 10^-4 )
    d_ECEFZ = ((double)ECEFZ) / 100.0; // Convert from cm to m
    d_ECEFZ += ((double)ECEFZHp) / 10000.0; // Now add the high resolution component ( mm * 10^-1 = m * 10^-4 )

   // Print the coordinates with 4 decimal places (0.1mm)
    Serial.print("X (m): ");
    Serial.print(d_ECEFX, 4);
    Serial.print(", Y (m): ");
    Serial.print(d_ECEFY, 4);
    Serial.print(", Z (m): ");
    Serial.print(d_ECEFZ, 4);

    // Now define float storage for the accuracy
    float f_accuracy;

    // Convert the horizontal accuracy (mm) to a float
    f_accuracy = accuracy;
    // Now convert to m
    f_accuracy = f_accuracy / 1000.0; // Convert from mm to m

    // Finally, do the printing
    Serial.print(", Accuracy (m): ");
    Serial.println(f_accuracy, 3); // Print the accuracy with 3 decimal places
  }
}
