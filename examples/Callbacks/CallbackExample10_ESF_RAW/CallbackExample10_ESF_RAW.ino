/*
  By: Paul Clark
  SparkFun Electronics
  Date: September 8th, 2022
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example configures the External Sensor Fusion RAW IMU sensor messages on the NEO-M8U/ZED-F9R and
  uses callbacks to process and display the ESF data automatically. No more polling!

  Notes:
    On the ZED-F9R, each ESF RAW message contains _one_ set of IMU sensor data, seven readings in total (3 x Accel, 3 x Gyro, 1 x Temperature).
    However, on the NEO-M8U, each message contains _ten_ sets of IMU sensor data, seventy readings in total.
    The NEO-M8U data is all timestamped and it is possible to reconstruct the full data stream, you just need to do it
    ten at a time...
    Also, note that the sensor data is 24-bit signed (two's complement). You need to be careful when converting to int32_t.
    Data will arrive at 100Hz. 10Hz x 10 on the NEO-M8U.
    400kHz I2C is essential.
    Serial printing needs to be kept short and the baud rate needs to be around 500000.

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

// Callback: printESFRAWdata will be called when new ESF RAW data arrives
// See u-blox_structs.h for the full definition of UBX_ESF_RAW_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoESFRAWcallback
//        /                  _____  This _must_ be UBX_ESF_RAW_data_t
//        |                 /                   _____ You can use any name you like for the struct
//        |                 |                  /
//        |                 |                  |
void printESFRAWdata(UBX_ESF_RAW_data_t *ubxDataStruct)
{
  Serial.print(F("New ESF RAW data received. Number of sensor readings is: "));
  Serial.print(ubxDataStruct->numEsfRawBlocks);
  if (ubxDataStruct->numEsfRawBlocks > 7)
    Serial.println(F(". (Only the first 7 will be printed.)"));
  else
    Serial.println(F("."));

  for (uint8_t i = 0; (i < ubxDataStruct->numEsfRawBlocks) && (i < 7); i++)
  {
    switch (ubxDataStruct->data[i].data.bits.dataType)
    {
      case 5:
        Serial.print(F("z-axis gyro: "));
        break;
      case 12:
        Serial.print(F("gyro temperature: "));
        break;
      case 13:
        Serial.print(F("y-axis gyro: "));
        break;
      case 14:
        Serial.print(F("x-axis gyro: "));
        break;
      case 16:
        Serial.print(F("x-axis accel: "));
        break;
      case 17:
        Serial.print(F("y-axis accel: "));
        break;
      case 18:
        Serial.print(F("z-axis accel: "));
        break;
      default:
        break;
    }
    if ((ubxDataStruct->data[i].data.bits.dataType == 5) || (ubxDataStruct->data[i].data.bits.dataType == 13) || (ubxDataStruct->data[i].data.bits.dataType == 14))
    {
      union
      {
        int32_t signed32;
        uint32_t unsigned32;
      } signedUnsigned; // Avoid any ambiguity casting uint32_t to int32_t
      // The dataField is 24-bit signed, stored in the 24 LSBs of a uint32_t
      signedUnsigned.unsigned32 = ubxDataStruct->data[i].data.bits.dataField  << 8; // Shift left by 8 bits to correctly align the data
      float rate =  signedUnsigned.signed32; // Extract the signed data. Convert to float
      rate /= 256.0; // Divide by 256 to undo the shift
      rate *= 0.000244140625; // Convert from deg/s*2^-12 to deg/s
      Serial.println(rate);     
    }
    else if ((ubxDataStruct->data[i].data.bits.dataType == 16) || (ubxDataStruct->data[i].data.bits.dataType == 17) || (ubxDataStruct->data[i].data.bits.dataType == 18))
    {
      union
      {
        int32_t signed32;
        uint32_t unsigned32;
      } signedUnsigned; // Avoid any ambiguity casting uint32_t to int32_t
      // The dataField is 24-bit signed, stored in the 24 LSBs of a uint32_t
      signedUnsigned.unsigned32 = ubxDataStruct->data[i].data.bits.dataField  << 8; // Shift left by 8 bits to correctly align the data
      float force =  signedUnsigned.signed32; // Extract the signed data. Convert to float
      force /= 256.0; // Divide by 256 to undo the shift
      force *= 0.0009765625; // Convert from m/s*2^-10 to m/s
      Serial.println(force);     
    }
    else if (ubxDataStruct->data[i].data.bits.dataType == 12)
    {
      union
      {
        int32_t signed32;
        uint32_t unsigned32;
      } signedUnsigned; // Avoid any ambiguity casting uint32_t to int32_t
      // The dataField is 24-bit signed, stored in the 24 LSBs of a uint32_t
      signedUnsigned.unsigned32 = ubxDataStruct->data[i].data.bits.dataField  << 8; // Shift left by 8 bits to correctly align the data
      float temperature =  signedUnsigned.signed32; // Extract the signed data. Convert to float
      temperature /= 256.0; // Divide by 256 to undo the shift
      temperature *= 0.01; // Convert from C*1e-2 to C
      Serial.println(temperature);     
    }
  }
}

void setup()
{
  Serial.begin(500000);
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();
  Wire.setClock(400000); // Use 400kHz I2C

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  myGNSS.setI2CpollingWait(5); //Allow checkUblox to poll I2C data every 5ms to keep up with the ESF RAW messages

  if (myGNSS.setAutoESFRAWcallbackPtr(&printESFRAWdata) == true) // Enable automatic ESF RAW messages with callback to printESFRAWdata
    Serial.println(F("setAutoESFRAWcallback successful"));
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
}
