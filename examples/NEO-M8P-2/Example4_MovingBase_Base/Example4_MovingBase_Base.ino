/*
  Configure the NEO-M8P-2 as a "moving base"
  By: Paul Clark
  SparkFun Electronics
  Date: March 20th, 2023
  License: MIT. See license file for more information.

  This example configures the NEO-M8P-2 as a moving base.
  It will output RTCM3 1077, 1087, 1230 and 4072.0 messages on UART1.
  Connect UART1 TX to UART1 RX on a NEO-M8P rover. Also connect GND to GND.
  The next example shows how to display the relative position of the rover.
 
  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  NEO-M8P RTK: https://www.sparkfun.com/products/15005

  Hardware Connections:
  Plug a Qwiic cable into the GNSS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Connect UART1 TX to UART1 RX on a NEO-M8P rover. Also connect GND to GND.
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GNSS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  while(!Serial); //Wait for user to open terminal
  Serial.println(F("u-blox NEO-M8P Moving Base Example"));

  Wire.begin();

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring."));
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  //myGNSS.factoryDefault(); delay(5000);

  myGNSS.setUART1Output(COM_TYPE_RTCM3); // Configure UART1 to output RTCM3 only. Disable NMEA.
  myGNSS.saveConfiguration(); //Save the current settings to flash and BBR

  bool response = true;
  
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1077, COM_PORT_UART1, 1); //Enable message 1077 on UART1
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1087, COM_PORT_UART1, 1); //Enable message 1087 on UART1
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_UART1, 1); //Enable message 1230 on UART1
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_4072_0, COM_PORT_UART1, 1); //Enable message 4072.0 on UART1
  
  response &= myGNSS.disableSurveyMode(); //Essential - we need to set TMODE3 to Disabled to allow 4072.0 to be output

  if (response == true)
  {
    Serial.println(F("RTCM messages enabled"));
  }
  else
  {
    Serial.println(F("RTCM failed to enable. Are you sure you have an NEO-M8P?"));
  }
}

void loop()
{
  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

  delay(250); //Don't pound too hard on the I2C bus
}
