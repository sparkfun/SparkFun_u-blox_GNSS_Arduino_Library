/*
  Set update rate to 10Hz
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 3rd, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to increase the output of the module from 1Hz to 4Hz.
  The max output rate various from model to model. RTFM! But you cannot do harm
  to the module.

  We also disable NMEA output on the I2C bus and use only UBX. This dramatically 
  decreases the amount of data that needs to be transmitted.

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

unsigned long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
unsigned long startTime = 0; //Used to calc the actual update rate.
unsigned long updateCount = 0; //Used to calc the actual update rate.

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  // Increase I2C clock speed to 400kHz to cope with the high navigation rate
  // (We normally recommend running the bus at 100kHz)
  Wire.setClock(400000);
  
  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(5); //Set output to 5 times a second

  uint8_t rate = myGNSS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate: ");
  Serial.println(rate);

  startTime = millis();
}

void loop()
{
  //Query module every 25 ms. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available. This is defined
  //by the update freq.
  if (millis() - lastTime > 25)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = myGNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);

    updateCount++;

    //Calculate the actual update rate based on the sketch start time and the 
    //number of updates we've received.
    Serial.print(F(" Rate: "));
    Serial.print( updateCount / ((millis() - startTime) / 1000.0), 2);
    Serial.print(F("Hz"));

    Serial.println();
  }
}
