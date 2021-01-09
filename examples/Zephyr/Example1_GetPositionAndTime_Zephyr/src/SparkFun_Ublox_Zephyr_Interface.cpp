/*
    This is an interface that connects the CPP u-blox library with the main C code.
    Added to make it possible to run u-blox lib on Zephyr (NCS)

    This port was made by Vid Rajtmajer <vid@irnas.eu>, www.irnas.eu
*/
#include "SparkFun_Ublox_Zephyr_Interface.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "SparkFun_Ublox_Zephyr_Library.h"


SFE_UBLOX_GPS myGPS; // driver class instance
long lastTime = 0;   // Simple local timer. Limits amount if I2C traffic to u-blox module.

// init GPIO checksumFailurePin and load GPIO device pointer to the driver
uint8_t set_gpio_dev(struct device *gpio_dev, uint8_t enable_debug)
{
    if (myGPS.init_gpio_pins(*gpio_dev) == false)
    {
        return -EIO;
    }
    // turn on debugging if enable_debug is set
    if (enable_debug)
    {
        myGPS.enableDebugging();
    }
    return 0;
}

// initialize I2C and check if GPS device respons
uint8_t gps_begin(struct device *i2c_dev)
{
    if (myGPS.begin(*i2c_dev) == false)
    {
        return -EIO;
    }
    return 0;
}

// This will pipe all NMEA sentences to UART so we can see them
void pipe_nmea_sentences(void)
{
    myGPS.setNMEAOutputPort();
}

// Check for available bytes from the device
void check_ublox(void)
{
    myGPS.checkUblox();
}

// Get position information when requested, also display number of satellites used in the fix
int get_position(void)
{
    //Query module only every second. Doing it more often will just cause I2C traffic.
    //The module only responds when a new position is available, print it to console
    if (k_uptime_get_32() - lastTime > 1000)
    {
        lastTime = k_uptime_get_32(); //Update the timer

        long latitude = myGPS.getLatitude();
        long longitude = myGPS.getLongitude();
        long altitude = myGPS.getAltitude();
        uint8_t SIV = myGPS.getSIV();

        printk("Position: Lat: %ld, Lon: %ld, Alt: %ld, SIV: %d", latitude, longitude, altitude, SIV);
        return 0;
    }
    return -EBUSY;
}

// Get date and time information when requested, check if they are valid and print info to console, it returns UNIX time
void get_datetime(void)
{
    int year = myGPS.getYear();
    int month = myGPS.getMonth();
    int day = myGPS.getDay();
    int hour = myGPS.getHour();
    int minute = myGPS.getMinute();
    int second = myGPS.getSecond();

    printk("DateTime: %d-%d-%d %d:%d:%d\n", year, month, day, hour, minute, second);

    printk("Time is ");
    if (myGPS.getTimeValid() == false)
    {
      printk("not ");
    }
    printk("valid. Date is ");
    if (myGPS.getDateValid() == false)
    {
      printk("not ");
    }
    printk("valid.\n");
}
