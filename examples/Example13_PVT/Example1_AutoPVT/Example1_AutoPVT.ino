/*
  Configuring the GNSS to automatically send position reports over I2C
  By: Nathan Seidle and Thorsten von Eicken
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the U-Blox GNSS the send navigation reports automatically
  and retrieving the latest one via getPVT. This eliminates the blocking in getPVT while the GNSS
  produces a fresh navigation solution at the expense of returning a slighly old solution.

  This can be used over serial or over I2C, this example shows the I2C use. With serial the GNSS
  simply outputs the UBX_NAV_PVT packet. With I2C it queues it into its internal I2C buffer (4KB in
  size?) where it can be retrieved in the next I2C poll.

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

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(2); //Produce two solutions per second
  myGNSS.setAutoPVT(true); //Tell the GNSS to "send" each solution
  //myGNSS.saveConfiguration(); //Optional: Save the current settings to flash and BBR
}

void loop()
{
  // Calling getPVT returns true if there actually is a fresh navigation solution available.
  // Start the reading only when valid LLH is available
  if (myGNSS.getPVT() && (myGNSS.getInvalidLlh() == false))
  {
    Serial.println();
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

    int PDOP = myGNSS.getPDOP();
    Serial.print(F(" PDOP: "));
    Serial.print(PDOP);
    Serial.print(F(" (10^-2)"));

    int nedNorthVel = myGNSS.getNedNorthVel();
    Serial.print(F(" VelN: "));
    Serial.print(nedNorthVel);
    Serial.print(F(" (mm/s)"));

    int nedEastVel = myGNSS.getNedEastVel();
    Serial.print(F(" VelE: "));
    Serial.print(nedEastVel);
    Serial.print(F(" (mm/s)"));

    int nedDownVel = myGNSS.getNedDownVel();
    Serial.print(F(" VelD: "));
    Serial.print(nedDownVel);
    Serial.print(F(" (mm/s)"));

    int verticalAccEst = myGNSS.getVerticalAccEst();
    Serial.print(F(" VAccEst: "));
    Serial.print(verticalAccEst);
    Serial.print(F(" (mm)"));

    int horizontalAccEst = myGNSS.getHorizontalAccEst();
    Serial.print(F(" HAccEst: "));
    Serial.print(horizontalAccEst);
    Serial.print(F(" (mm)"));

    int speedAccEst = myGNSS.getSpeedAccEst();
    Serial.print(F(" SpeedAccEst: "));
    Serial.print(speedAccEst);
    Serial.print(F(" (mm/s)"));

    int headAccEst = myGNSS.getHeadingAccEst();
    Serial.print(F(" HeadAccEst: "));
    Serial.print(headAccEst);
    Serial.print(F(" (degrees * 10^-5)"));

    if (myGNSS.getHeadVehValid() == true) {
      int headVeh = myGNSS.getHeadVeh();
      Serial.print(F(" HeadVeh: "));
      Serial.print(headVeh);
      Serial.print(F(" (degrees * 10^-5)"));

      int magDec = myGNSS.getMagDec();
      Serial.print(F(" MagDec: "));
      Serial.print(magDec);
      Serial.print(F(" (degrees * 10^-2)"));

      int magAcc = myGNSS.getMagAcc();
      Serial.print(F(" MagAcc: "));
      Serial.print(magAcc);
      Serial.print(F(" (degrees * 10^-2)"));
    }

    Serial.println();
  } else {
    Serial.print(".");
    delay(50);
  }
}
