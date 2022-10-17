/*
  Send UBX binary commands to enable RTCM sentences on u-blox ZED-F9P module
  By: Nathan Seidle
  SparkFun Electronics
  Date: January 9th, 2019
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example does all steps to configure and enable a ZED-F9P as a base station:
    Begin Survey-In
    Once we've achieved 2m accuracy and 300s have passed, survey is complete
    Enable six RTCM messages
    Begin outputting RTCM bytes

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

//#define USE_SERIAL1 // Uncomment this line to push the RTCM data to Serial1

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("u-blox Base Station example"));

#ifdef USE_SERIAL1
  // If our board supports it, we can output the RTCM data on Serial1
  Serial1.begin(115200);
#endif

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  // Uncomment the next line if you want to reset your module back to the default settings with 1Hz navigation rate
  //myGNSS.factoryDefault(); delay(5000);

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // Ensure RTCM3 is enabled
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR

  while (Serial.available()) Serial.read(); //Clear any latent chars in serial buffer
  Serial.println(F("Press any key to send commands to begin Survey-In"));
  while (Serial.available() == 0) ; //Wait for user to press a key

  bool response = true;
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1); //Enable message 1005 to output through I2C port, message every second
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10); //Enable message every 10 seconds

  //Use COM_PORT_UART1 for the above six messages to direct RTCM messages out UART1
  //COM_PORT_UART2, COM_PORT_USB, COM_PORT_SPI are also available
  //For example: response &= myGNSS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_UART1, 10);

  if (response == true)
  {
    Serial.println(F("RTCM messages enabled"));
  }
  else
  {
    Serial.println(F("RTCM failed to enable. Are you sure you have an ZED-F9P?"));
    while (1); //Freeze
  }

  //Check if Survey is in Progress before initiating one
  // From v2.0, the data from getSurveyStatus (UBX-NAV-SVIN) is returned in UBX_NAV_SVIN_t packetUBXNAVSVIN
  // Please see u-blox_structs.h for the full definition of UBX_NAV_SVIN_t
  // You can either read the data from packetUBXNAVSVIN directly
  // or can use the helper functions: getSurveyInActive; getSurveyInValid; getSurveyInObservationTime; and getSurveyInMeanAccuracy
  response = myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)
  
  if (response == false) // Check if fresh data was received
  {
    Serial.println(F("Failed to get Survey In status"));
    while (1); //Freeze
  }

  if (myGNSS.getSurveyInActive() == true) // Use the helper function
  //if (myGNSS.packetUBXNAVSVIN->data.active > 0) // Or we could read active directly
  {
    Serial.print(F("Survey already in progress."));
  }
  else
  {
    //Start survey
    //The ZED-F9P is slightly different than the NEO-M8P. See the Integration manual 3.5.8 for more info.
    //response = myGNSS.enableSurveyMode(300, 2.000); //Enable Survey in on NEO-M8P, 300 seconds, 2.0m
    response = myGNSS.enableSurveyMode(60, 5.000); //Enable Survey in, 60 seconds, 5.0m
    //response = myGNSS.enableSurveyModeFull(86400, 2.000); //Enable Survey in, 24 hours, 2.0m
    if (response == false)
    {
      Serial.println(F("Survey start failed. Freezing..."));
      while (1);
    }
    Serial.println(F("Survey started. This will run until 60s has passed and less than 5m accuracy is achieved."));
  }

  while(Serial.available()) Serial.read(); //Clear buffer
  
  //Begin waiting for survey to complete
  while (myGNSS.getSurveyInValid() == false) // Call the helper function
  //while (myGNSS.packetUBXNAVSVIN->data.valid == 0) // Or we could read valid directly
  {
    if(Serial.available())
    {
      byte incoming = Serial.read();
      if(incoming == 'x')
      {
        //Stop survey mode
        response = myGNSS.disableSurveyMode(); //Disable survey
        Serial.println(F("Survey stopped"));
        break;
      }
    }

    // From v2.0, the data from getSurveyStatus (UBX-NAV-SVIN) is returned in UBX_NAV_SVIN_t packetUBXNAVSVIN
    // Please see u-blox_structs.h for the full definition of UBX_NAV_SVIN_t
    // You can either read the data from packetUBXNAVSVIN directly
    // or can use the helper functions: getSurveyInActive; getSurveyInValid; getSurveyInObservationTime; getSurveyInObservationTimeFull; and getSurveyInMeanAccuracy
    response = myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)
    
    if (response == true) // Check if fresh data was received
    {
      Serial.print(F("Press x to end survey - "));
      Serial.print(F("Time elapsed: "));
      Serial.print((String)myGNSS.getSurveyInObservationTimeFull()); // Call the helper function
      Serial.print(F(" ("));
      Serial.print((String)myGNSS.packetUBXNAVSVIN->data.dur); // Read the survey-in duration directly from packetUBXNAVSVIN

      Serial.print(F(") Accuracy: "));
      Serial.print((String)myGNSS.getSurveyInMeanAccuracy()); // Call the helper function
      Serial.print(F(" ("));
      // Read the mean accuracy directly from packetUBXNAVSVIN and manually convert from mm*0.1 to m
      float meanAcc = ((float)myGNSS.packetUBXNAVSVIN->data.meanAcc) / 10000.0;
      Serial.print((String)meanAcc); 
      Serial.println(F(")"));
    }
    else
    {
      Serial.println(F("SVIN request failed"));
    }

    delay(1000);
  }
  Serial.println(F("Survey valid!"));

  Serial.println(F("Base survey complete! RTCM now broadcasting."));

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3); //Set the I2C port to output UBX and RTCM sentences (not really an option, turns on NMEA as well)
}

void loop()
{
  myGNSS.checkUblox(); //See if new data is available. Process bytes as they come in.

  delay(250); //Don't pound too hard on the I2C bus
}

//This function gets called from the SparkFun u-blox Arduino Library.
//As each RTCM byte comes in you can specify what to do with it
//Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void SFE_UBLOX_GNSS::processRTCM(uint8_t incoming)
{
#ifdef USE_SERIAL1
  //Push the RTCM data to Serial1
  Serial1.write(incoming);
#endif

  //Pretty-print the HEX values to Serial
  if (myGNSS.rtcmFrameCounter % 16 == 0) Serial.println();
  Serial.print(F(" "));
  if (incoming < 0x10) Serial.print(F("0"));
  Serial.print(incoming, HEX);
}
