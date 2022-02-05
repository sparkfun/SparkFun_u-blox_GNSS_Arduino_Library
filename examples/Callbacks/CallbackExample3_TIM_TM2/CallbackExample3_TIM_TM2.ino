/*
  Configuring the GNSS to automatically send TIM TM2 reports over I2C and display the data using a callback
  By: Paul Clark
  SparkFun Electronics
  Date: December 30th, 2020
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to configure the u-blox GNSS to send TIM TM2 reports automatically
  and display the data via a callback. No more polling!

  Connecting the PPS (Pulse Per Second) breakout pin to the INT (Interrupt) pin with a jumper wire
  will cause a TIM TM2 message to be produced once per second. You can then study the timing of the
  pulse edges with nanosecond resolution!

  Note: TIM TM2 can only capture the timing of one rising edge and one falling edge per
  navigation solution. So with setNavigationFrequency set to 1Hz, we can only see the timing
  of one rising and one falling edge per second. If the frequency of the signal on the INT pin
  is higher than 1Hz, we will only be able to see the timing of the most recent edges.
  However, the module can count the number of rising edges too, at rates faster than the navigation rate.

  TIM TM2 messages are only produced when a rising or falling edge is detected on the INT pin.
  If you disconnect your PPS to INT jumper wire, the messages will stop.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  NEO-M9N: https://www.sparkfun.com/products/17285

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
*/

#include <Wire.h> //Needed for I2C to GPS

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

int dotsPrinted = 0; // Print dots in rows of 50 while waiting for a TIM TM2 message

// Callback: printTIMTM2data will be called when new TIM TM2 data arrives
// See u-blox_structs.h for the full definition of UBX_TIM_TM2_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoTIMTM2callback
//        /                  _____  This _must_ be UBX_TIM_TM2_data_t
//        |                 /                   _____ You can use any name you like for the struct
//        |                 |                  /
//        |                 |                  |
void printTIMTM2data(UBX_TIM_TM2_data_t *ubxDataStruct)
{
    Serial.println();

    Serial.print(F("newFallingEdge: ")); // 1 if a new falling edge was detected
    Serial.print(ubxDataStruct->flags.bits.newFallingEdge);

    Serial.print(F(" newRisingEdge: ")); // 1 if a new rising edge was detected
    Serial.print(ubxDataStruct->flags.bits.newRisingEdge);

    Serial.print(F(" Rising Edge Counter: ")); // Rising edge counter
    Serial.print(ubxDataStruct->count);

    Serial.print(F(" towMsR: ")); // Time Of Week of rising edge (ms)
    Serial.print(ubxDataStruct->towMsR);

    Serial.print(F(" towSubMsR: ")); // Millisecond fraction of Time Of Week of rising edge in nanoseconds
    Serial.print(ubxDataStruct->towSubMsR);

    Serial.print(F(" towMsF: ")); // Time Of Week of falling edge (ms)
    Serial.print(ubxDataStruct->towMsF);

    Serial.print(F(" towSubMsF: ")); // Millisecond fraction of Time Of Week of falling edge in nanoseconds
    Serial.println(ubxDataStruct->towSubMsF);

    dotsPrinted = 0; // Reset dotsPrinted
}

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun u-blox Example");

  Wire.begin();

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  if (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

  myGNSS.setNavigationFrequency(1); //Produce one solution per second

  myGNSS.setAutoTIMTM2callbackPtr(&printTIMTM2data); // Enable automatic TIM TM2 messages with callback to printTIMTM2data
}

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  Serial.print(".");
  delay(50);
  if (++dotsPrinted > 50)
  {
    Serial.println();
    dotsPrinted = 0;
  }
}
