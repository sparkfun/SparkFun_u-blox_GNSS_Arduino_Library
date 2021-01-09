/*
  By: Elias Santistevan
  SparkFun Electronics
  Date: May, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  NEO-M8U: https://www.sparkfun.com/products/16329
  ZED-F9R: https://www.sparkfun.com/products/16344  

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a Redboard Qwiic
  If you don't have a platform with a Qwiic connection use the 
	SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output

	After calibrating the module, also known as "Fusion Mode", you can get
	data directly from the IMU. This example code walks you through trouble
  shooting or identifying the different states of any individual  
  "external" (which include internal) sensors you've hooked up (vehicle speed
  sensor) or the internal IMU used by the modules. You can see if the sensor is
  being used, if it's calibrated, ready, what data type it returns, the state
  of the measurement etc.

*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

	// GetEsfInfo also gets the number of sensors used by the ublox module, this
	// includes (in the case of the ZED-F9R) wheel tick input from the vehicle
	// speed sensor attached to the module. 
  if (myGNSS.getEsfInfo()){

    Serial.print(F("Fusion Mode: "));  
    Serial.println(myGNSS.packetUBXESFSTATUS->data.fusionMode);  

    if (myGNSS.packetUBXESFSTATUS->data.fusionMode == 1){
      Serial.println(F("Fusion Mode is Initialized!"));  
		}
		else {
      Serial.println(F("Fusion Mode is either disabled or not initialized!"));  
			Serial.println(F("Please see the previous example for more information."));
		}
  }
}

void loop()
{
  // ESF data is produced at the navigation rate, so by default we'll get fresh data once per second
  if (myGNSS.getEsfInfo()) // Poll new ESF STATUS data
  {
    UBX_ESF_STATUS_sensorStatus_t sensorStatus; // Create storage for the individual sensor status

    //See ublox receiver description or our hookup guide for information on the return values

    Serial.println(F("                              "));
    Serial.println(F("              C               "));
    Serial.println(F("              a               "));
    Serial.println(F("              l               "));
    Serial.println(F("              i               "));
    Serial.println(F("              b           M   "));
    Serial.println(F("              r           i  N"));
    Serial.println(F("              a        B  s  o"));
    Serial.println(F("    S         t  T  B  a  s  i"));
    Serial.println(F("S   e   B     i  i  a  d  e  s"));
    Serial.println(F("e   n   e     o  m  d     d  y"));
    Serial.println(F("n   s   i  I  n  e     T      "));
    Serial.println(F("s   o   n  s        M  i  M  M"));
    Serial.println(F("o   r   g     S  S  e  m  e  e"));
    Serial.println(F("r          R  t  t  a  e  a  a"));
    Serial.println(F("    T   U  e  a  a  s     s  s"));
    Serial.println(F("N   y   s  a  t  t  u  T  u  u"));
    Serial.println(F("o   p   e  d  u  u  r  a  r  r"));
    Serial.println(F(".   e   d  y  s  s  e  g  e  e"));
    Serial.println(F("                              "));

  	for(uint8_t i = 0; i < myGNSS.packetUBXESFSTATUS->data.numSens; i++)
  	{
      myGNSS.getSensorFusionStatus(&sensorStatus, i); // Extract the individual sensor data for this sensor
      
      Serial.print(i); Serial.print(F("   ")); // Print the sensor number

      // Print the sensor type
  		Serial.print(sensorStatus.sensStatus1.bits.type);
      if (sensorStatus.sensStatus1.bits.type < 10) Serial.print(F(" "));
  		Serial.print(F("  "));

      Serial.print(sensorStatus.sensStatus1.bits.used); Serial.print(F("  ")); // Print the used flag
      Serial.print(sensorStatus.sensStatus1.bits.ready); Serial.print(F("  ")); // Print the ready flag

      Serial.print(sensorStatus.sensStatus2.bits.calibStatus); Serial.print(F("  ")); // Print the calibration status
      Serial.print(sensorStatus.sensStatus2.bits.timeStatus); Serial.print(F("  ")); // Print the time status

      Serial.print(sensorStatus.faults.bits.badMeas); Serial.print(F("  ")); // Print the bad measurement flag
      Serial.print(sensorStatus.faults.bits.badTTag); Serial.print(F("  ")); // Print the time tag flag
      Serial.print(sensorStatus.faults.bits.missingMeas); Serial.print(F("  ")); // Print the missing measurement flag
      Serial.print(sensorStatus.faults.bits.noisyMeas); // Print the noisy measure flag

      Serial.println(); 
  	}
  }

  delay(250);
}
