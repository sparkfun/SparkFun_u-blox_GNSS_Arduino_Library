/*
  u-blox Example: ESF MEAS (Wheel Ticks)
  By: Paul Clark
  SparkFun Electronics
  Date: September 8th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example configures the External Sensor Fusion MEAS sensor messages on the NEO-M8U / ZED-F9R and
  shows how to access the ESF data in the loop - without using the callback.

  Please make sure your NEO-M8U is running UDR firmware >= 1.31. Please update using u-center if necessary:
  https://www.u-blox.com/en/product/neo-m8u-module#tab-documentation-resources

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  NEO-M8U: https://www.sparkfun.com/products/16329

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a Redboard Qwiic
  If you don't have a platform with a Qwiic connection use the
	SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output

*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

// Callback: printESFMEASdata will be called when new ESF MEAS data arrives
// See u-blox_structs.h for the full definition of UBX_ESF_MEAS_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoESFMEAScallback
//        /                  _____  This _must_ be UBX_ESF_MEAS_data_t
//        |                 /                   _____ You can use any name you like for the struct
//        |                 |                  /
//        |                 |                  |
void printESFMEASdata(UBX_ESF_MEAS_data_t *ubxDataStruct)
{
  Serial.println(F("Hey! The ESF MEAS callback has been called!"));
}

void setup()
{
  Serial.begin(230400); // <-- Use a fast baud rate to avoid the Serial prints slowing the code
  
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();
  Wire.setClock(400000); // <-- Use 400kHz I2C

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  if (myGNSS.setAutoESFMEAScallbackPtr(&printESFMEASdata) == true) // Enable automatic ESF MEAS messages with callback to printESFMEASdata
    Serial.println(F("setAutoESFMEAScallback successful"));
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.

  // Check if new ESF MEAS data has arrived:
  // If myGNSS.packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid is true, it indicates new ESF MEAS data has been received and has been copied.
  // automaticFlags.flags.bits.callbackCopyValid will be cleared automatically when the callback is called.

  if (myGNSS.packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid == true)
  {
    // But, we can manually clear the callback flag too. This will prevent the callback from being called!
    myGNSS.packetUBXESFMEAS->automaticFlags.flags.bits.callbackCopyValid = false; // Comment this line if you still want the callback to be called
  
    // Print the timeTag
    Serial.print(F("Time:        "));
    Serial.println(myGNSS.packetUBXESFMEAS->callbackData->timeTag);

    // myGNSS.packetUBXESFMEAS->callbackData->flags.bits.numMeas indicates how many sensor groups the UBX_ESF_MEAS_data_t contains.
    for (uint8_t i = 0; i < myGNSS.packetUBXESFMEAS->callbackData->flags.bits.numMeas; i++)
    {  
      // Print the sensor data type
      // From the M8 interface description:
      //   0: None
      // 1-4: Reserved
      //   5: z-axis gyroscope angular rate deg/s * 2^-12 signed
      //   6: front-left wheel ticks: Bits 0-22: unsigned tick value. Bit 23: direction indicator (0=forward, 1=backward)
      //   7: front-right wheel ticks: Bits 0-22: unsigned tick value. Bit 23: direction indicator (0=forward, 1=backward)
      //   8: rear-left wheel ticks: Bits 0-22: unsigned tick value. Bit 23: direction indicator (0=forward, 1=backward)
      //   9: rear-right wheel ticks: Bits 0-22: unsigned tick value. Bit 23: direction indicator (0=forward, 1=backward)
      //  10: speed ticks: Bits 0-22: unsigned tick value. Bit 23: direction indicator (0=forward, 1=backward)
      //  11: speed m/s * 1e-3 signed
      //  12: gyroscope temperature deg Celsius * 1e-2 signed
      //  13: y-axis gyroscope angular rate deg/s * 2^-12 signed
      //  14: x-axis gyroscope angular rate deg/s * 2^-12 signed
      //  16: x-axis accelerometer specific force m/s^2 * 2^-10 signed
      //  17: y-axis accelerometer specific force m/s^2 * 2^-10 signed
      //  18: z-axis accelerometer specific force m/s^2 * 2^-10 signed
      switch (myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataType)
      {
        case 5:
          Serial.print(F("Z Gyro:      "));
          break;
        case 6:
          Serial.print(F("Front Left:  "));
          break;
        case 7:
          Serial.print(F("Front Right: "));
          break;
        case 8:
          Serial.print(F("Rear Left:   "));
          break;
        case 9:
          Serial.print(F("Rear Right:  "));
          break;
        case 10:
          Serial.print(F("Speed Ticks: "));
          break;
        case 11:
          Serial.print(F("Speed:       "));
          break;
        case 12:
          Serial.print(F("Temp:        "));
          break;
        case 13:
          Serial.print(F("Y Gyro:      "));
          break;
        case 14:
          Serial.print(F("X Gyro:      "));
          break;
        case 16:
          Serial.print(F("X Accel:     "));
          break;
        case 17:
          Serial.print(F("Y Accel:     "));
          break;
        case 18:
          Serial.print(F("Z Accel:     "));
          break;
        default:
          break;
      }
      
      // Tick data
      if ((myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataType >= 6) && (myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataType <= 10))
      {
        if ((myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataField & (1 << 23)) > 0)
          Serial.print(F("-")); // Backward
        else
          Serial.print(F("+")); // Forward
        Serial.println(myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataField & 0x007FFFFF);
      }
      // Speed
      else if (myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataType == 11)
      {
        union
        {
          int32_t signed32;
          uint32_t unsigned32;
        } signedUnsigned; // Avoid any ambiguity casting uint32_t to int32_t
        // The dataField is 24-bit signed, stored in the 24 LSBs of a uint32_t
        signedUnsigned.unsigned32 = myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataField  << 8; // Shift left by 8 bits to correctly align the data
        float speed =  signedUnsigned.signed32; // Extract the signed data. Convert to float
        speed /= 256.0; // Divide by 256 to undo the shift
        speed *= 0.001; // Convert from m/s * 1e-3 to m/s
        Serial.println(speed, 3);     
      }
      // Gyro data
      else if ((myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataType == 5) || (myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataType == 13) || (myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataType == 14))
      {
        union
        {
          int32_t signed32;
          uint32_t unsigned32;
        } signedUnsigned; // Avoid any ambiguity casting uint32_t to int32_t
        // The dataField is 24-bit signed, stored in the 24 LSBs of a uint32_t
        signedUnsigned.unsigned32 = myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataField  << 8; // Shift left by 8 bits to correctly align the data
        float rate =  signedUnsigned.signed32; // Extract the signed data. Convert to float
        rate /= 256.0; // Divide by 256 to undo the shift
        rate *= 0.000244140625; // Convert from deg/s * 2^-12 to deg/s
        Serial.println(rate);     
      }
      // Accelerometer data
      else if ((myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataType == 16) || (myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataType == 17) || (myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataType == 18))
      {
        union
        {
          int32_t signed32;
          uint32_t unsigned32;
        } signedUnsigned; // Avoid any ambiguity casting uint32_t to int32_t
        // The dataField is 24-bit signed, stored in the 24 LSBs of a uint32_t
        signedUnsigned.unsigned32 = myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataField  << 8; // Shift left by 8 bits to correctly align the data
        float force =  signedUnsigned.signed32; // Extract the signed data. Convert to float
        force /= 256.0; // Divide by 256 to undo the shift
        force *= 0.0009765625; // Convert from m/s^2 * 2^-10 to m/s^2
        Serial.println(force);     
      }
      // Gyro Temperature
      else if (myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataType == 12)
      {
        union
        {
          int32_t signed32;
          uint32_t unsigned32;
        } signedUnsigned; // Avoid any ambiguity casting uint32_t to int32_t
        // The dataField is 24-bit signed, stored in the 24 LSBs of a uint32_t
        signedUnsigned.unsigned32 = myGNSS.packetUBXESFMEAS->callbackData->data[i].data.bits.dataField  << 8; // Shift left by 8 bits to correctly align the data
        float temperature =  signedUnsigned.signed32; // Extract the signed data. Convert to float
        temperature /= 256.0; // Divide by 256 to undo the shift
        temperature *= 0.01; // Convert from C * 1e-2 to C
        Serial.println(temperature);     
      }
    }
  }

  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed. There will not be any in this example, unless you commented the line above  
}
