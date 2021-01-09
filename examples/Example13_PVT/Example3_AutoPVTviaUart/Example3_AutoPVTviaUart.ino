/*
  Configuring the GNSS to automatically send position reports over Serial
  By: Nathan Seidle, Adapted from Example11 by Felix Jirka
  SparkFun Electronics
  Date: July 2nd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the U-Blox GNSS the send navigation reports automatically
  and retrieving the latest one via getPVT. This eliminates the blocking in getPVT while the GNSS
  produces a fresh navigation solution at the expense of returning a slighly old solution.

  This can be used over serial or over I2C, this example shows the Serial use. With serial the GNSS
  simply outputs the UBX_NAV_PVT packet. With I2C it queues it into its internal I2C buffer (4KB in
  size?) where it can be retrieved in the next I2C poll.

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

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include <SoftwareSerial.h>

//#define mySerial Serial2 // Uncomment this line to connect via Serial2
// - or -
SoftwareSerial mySerial(10, 11); // Uncomment this line to connect via SoftwareSerial(RX, TX). Connect pin 10 to GNSS TX pin.

//#define baudRate 9600 // Uncomment this line to select 9600 Baud for the M8
// - or -
#define baudRate 38400 // Uncomment this line to select 38400 Baud for the F9

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  mySerial.begin(baudRate); // Start the Serial port

  if (myGNSS.begin(mySerial) == false) //Connect to the u-blox module using Serial
  {
    Serial.println(F("u-blox GNSS not detected. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART1 port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(2); //Produce two solutions per second
  myGNSS.setAutoPVT(true); //Tell the GNSS to "send" each solution
  //myGNSS.saveConfiguration(); //Optional: Save the current settings to flash and BBR
}

void loop()
{
  // getPVT will return true if there actually is a fresh navigation solution available.
  // Important note: the PVT message is 100 bytes long. We need to call getPVT often enough
  // to prevent serial buffer overflows on boards like the original RedBoard / UNO.
  // At 38400 Baud, the 100 PVT bytes will arrive in 26ms.
  // On the RedBoard, we need to call getPVT every 5ms to keep up.
  if (myGNSS.getPVT())
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

    Serial.println();
  }
  else
  {
    delay(5); // Delay for 5ms only

    static int counter = 0; // Print a dot every 50ms
    counter++;
    if (counter > 10)
    {
      Serial.print(".");
      counter = 0;
    }
  }
}
