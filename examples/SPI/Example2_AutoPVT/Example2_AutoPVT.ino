/*
  Configuring the GNSS to automatically send position reports over SPI
  Based on code by: Nathan Seidle and Thorsten von Eicken
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the U-Blox GNSS the send navigation reports automatically
  and retrieving the latest one via getPVT. This eliminates the blocking in getPVT while the GNSS
  produces a fresh navigation solution at the expense of returning a slighly old solution.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  NEO-M8U: https://www.sparkfun.com/products/16329
  NEO-M9N: https://www.sparkfun.com/products/17285

  Hardware Connections:
  You need to connect the SPI pins from your microcontroller to the specific pins on your SparkFun product. 
  Connections will vary based on your microcontroller, but for reference please refer to this tutorial on SPI:
  https://learn.sparkfun.com/tutorials/serial-peripheral-interface-spi/all
  
  Most new boards now use the terms:
  CS - Chip Select
  COPI - Controller Out, Peripheral In
  CIPO - Controller in, Peripheral Out
  SCK - Serial Clock

  You can choose any pin for Chip Select, but the others are likely either defined by the library you are using
  (see here for the standard Arduino one: https://www.arduino.cc/en/reference/SPI) or the microcontroller. The
  ESP32 has two standard, selectable SPI ports, for example.

  To enable SPI communication, you will need to solder the DSEL/SPI jumper closed on your u-blox board.

  IMPORTANT: there have been reports that some u-blox devices do not respond to the UBX protocol over SPI
  with the factory settings. You may find you need to connect to the device via USB and u-center and set
  the incoming protocol to UBX only. Make sure you disable all other protocols as inputs if you can't
  get things to work! Hopefully this is just a bug in the u-blox firmware that will be fixed soon ;-)

*/

#include <SPI.h> //Needed for SPI to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

// #########################################

// Instantiate an instance of the SPI class. 
// Your configuration may be different, depending on the microcontroller you are using!

#define spiPort SPI // This is the SPI port on standard Ardino boards. Comment this line if you want to use a different port.

//SPIClass spiPort (HSPI); // This is the default SPI interface on some ESP32 boards. Uncomment this line if you are using ESP32.

// #########################################

const uint8_t csPin = 10; // On ATmega328 boards, SPI Chip Select is usually pin 10. Change this to match your board.

// #########################################

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  spiPort.begin(); // begin the SPI port

  //myGNSS.enableDebugging(); // Uncomment this line to see helpful debug messages on Serial

  // Connect to the u-blox module using SPI port, csPin and speed setting
  // ublox devices generally work up to 5MHz. We'll use 4MHz for this example:
  if (myGNSS.begin(spiPort, csPin, 4000000) == false) 
  {
    Serial.println(F("u-blox GNSS not detected on SPI bus. Please check wiring. Freezing."));
    while (1);
  }
  
  //myGNSS.factoryDefault(); delay(5000); // Uncomment this line to reset the module back to its factory defaults

  myGNSS.setPortOutput(COM_PORT_SPI, COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  myGNSS.setNavigationFrequency(2); //Produce two solutions per second
  myGNSS.setAutoPVT(true); //Tell the GNSS to "send" each solution
  //myGNSS.saveConfiguration(); //Optional: Save _all_ the current settings to flash and BBR
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
