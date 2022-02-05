/*
  By: Paul Clark
  SparkFun Electronics
  Date: December, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example configures the External Sensor Fusion messages on the NEO-M8U and
  uses callbacks to process and display the ESF data automatically. No more polling!

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

// Callback: printESFALGdata will be called when new ESF ALG data arrives
// See u-blox_structs.h for the full definition of UBX_ESF_ALG_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoESFALGcallback
//        /                  _____  This _must_ be UBX_ESF_ALG_data_t
//        |                 /                   _____ You can use any name you like for the struct
//        |                 |                  /
//        |                 |                  |
void printESFALGdata(UBX_ESF_ALG_data_t *ubxDataStruct)
{
  Serial.println();

  Serial.print(F("TOW: ")); // Print the Time Of Week
  unsigned long iTOW = ubxDataStruct->iTOW; // iTOW is in milliseconds
  Serial.print(iTOW);
  Serial.print(F(" (ms)"));

  Serial.print(F(" Roll: ")); // Print selected data
  Serial.print((float)ubxDataStruct->roll / 100.0, 2); // Convert roll to degrees

  Serial.print(F(" Pitch: "));
  Serial.print((float)ubxDataStruct->pitch / 100.0, 2); // Convert pitch to degrees

  Serial.print(F(" Yaw: "));
  Serial.print((float)ubxDataStruct->yaw / 100.0, 2); // Convert yaw to degrees

  Serial.println(F(" (Degrees)"));
}

// Callback: printESFINSdata will be called when new ESF INS data arrives
// See u-blox_structs.h for the full definition of UBX_ESF_INS_data_t
void printESFINSdata(UBX_ESF_INS_data_t *ubxDataStruct)
{
  Serial.print(F("xAccel: ")); // Print selected data
  Serial.print(ubxDataStruct->xAccel);

  Serial.print(F(" yAccel: "));
  Serial.print(ubxDataStruct->yAccel);

  Serial.print(F(" zAccel: "));
  Serial.print(ubxDataStruct->zAccel);

  Serial.println(F(" (m/s^2)"));
}

// Callback: printESFMEASdata will be called when new ESF MEAS data arrives
// See u-blox_structs.h for the full definition of UBX_ESF_MEAS_data_t
// and UBX_ESF_MEAS_sensorData_t
void printESFMEASdata(UBX_ESF_MEAS_data_t *ubxDataStruct)
{
  Serial.println();

  Serial.print(F("id: ")); // Print selected data
  Serial.print(ubxDataStruct->id);

  Serial.print(F(" numMeas: "));
  Serial.println(ubxDataStruct->flags.bits.numMeas);

  for (uint8_t num = 0; num < ubxDataStruct->flags.bits.numMeas; num++) // For each sensor
  {
    Serial.print(F("Sensor "));
    Serial.print(num);

    UBX_ESF_MEAS_sensorData_t sensorData;
    myGNSS.getSensorFusionMeasurement(&sensorData, *ubxDataStruct, num); // Extract the data for one sensor

    Serial.print(F(": Type: "));
    Serial.print(sensorData.data.bits.dataType);
    Serial.print(F(" Data: "));
    Serial.println(sensorData.data.bits.dataField);
  }
}

// Callback: printESFSTATUSdata will be called when new ESF STATUS data arrives
// See u-blox_structs.h for the full definition of UBX_ESF_STATUS_data_t
// and UBX_ESF_STATUS_sensorStatus_t
void printESFSTATUSdata(UBX_ESF_STATUS_data_t *ubxDataStruct)
{
  Serial.print(F("fusionMode: ")); // Print selected data
  Serial.print(ubxDataStruct->fusionMode);

  Serial.print(F(" numSens: "));
  Serial.println(ubxDataStruct->numSens);

  for (uint8_t num = 0; num < ubxDataStruct->numSens; num++) // For each sensor
  {
    Serial.print(F("Sensor "));
    Serial.print(num);

    UBX_ESF_STATUS_sensorStatus_t sensorStatus;
    myGNSS.getSensorFusionStatus(&sensorStatus, *ubxDataStruct, num); // Extract the data for one sensor

    Serial.print(F(": Type: "));
    Serial.print(sensorStatus.sensStatus1.bits.type);
    Serial.print(F(" Used: "));
    Serial.print(sensorStatus.sensStatus1.bits.used);
    Serial.print(F(" Ready: "));
    Serial.print(sensorStatus.sensStatus1.bits.ready);
    Serial.print(F(" Calib Status: "));
    Serial.print(sensorStatus.sensStatus2.bits.calibStatus);
    Serial.print(F(" Noisy: "));
    Serial.println(sensorStatus.faults.bits.noisyMeas);
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println(F("SparkFun u-blox Example"));

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  myGNSS.setNavigationFrequency(1); //Produce one solution per second
  myGNSS.setHNRNavigationRate(1); //Set the High Navigation Rate to 1Hz

  myGNSS.setI2CpollingWait(50); //Allow checkUblox to poll I2C data every 50ms to keep up with the ESF MEAS messages

  if (myGNSS.setAutoESFALGcallbackPtr(&printESFALGdata) == true) // Enable automatic ESF ALG messages with callback to printESFALGdata
    Serial.println(F("setAutoESFALGcallback successful"));

  if (myGNSS.setAutoESFINScallbackPtr(&printESFINSdata) == true) // Enable automatic ESF INS messages with callback to printESFINSdata
    Serial.println(F("setAutoESFINScallback successful"));

  if (myGNSS.setAutoESFMEAScallbackPtr(&printESFMEASdata) == true) // Enable automatic ESF MEAS messages with callback to printESFMEASdata
    Serial.println(F("setAutoESFMEAScallback successful"));

  if (myGNSS.setAutoESFSTATUScallbackPtr(&printESFSTATUSdata) == true) // Enable automatic ESF STATUS messages with callback to printESFSTATUSdata
    Serial.println(F("setAutoESFSTATUScallback successful"));
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  Serial.print(".");
  delay(25);
}
