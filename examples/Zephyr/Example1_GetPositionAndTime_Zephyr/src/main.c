/*
    Reading lat, long and UTC time via UBX binary commands
    By: Nathan Seidle
    SparkFun Electronics
    Date: August 22nd, 2018
    License: MIT. See license file for more information but you can
    basically do whatever you want with this code.

    This example reads the NMEA setences from the u-blox module over I2c and outputs
    them to the serial port

    Open the serial monitor at 115200 baud to see the output
    I2C clock speed: 100 kHz

    Ported to Zephyr by Vid Rajtmajer <vid@irnas.eu>, www.irnas.eu

    Development environment specifics: NCS v1.0.3 release

    To build: west build -b <BOARD_NAME> -p    Can also read NMEA sentences over I2C with check_ublox function
    To flash: west flash --erase
*/
#include <device.h>
#include <drivers/i2c.h>
#include <errno.h>
#include <zephyr.h>
#include <zephyr/types.h>

#include "SparkFun_Ublox_Zephyr_Interface.h"


#define I2C_DEV "I2C_0"

struct device *gpio_dev;
struct device *i2c_dev;
/* I2C pins used are defaults for I2C_0 on nrf52840
	SDA: 26
	SCL: 27
*/

uint8_t init_gpio(void) {
	const char* gpioName = "GPIO_0";
	gpio_dev = device_get_binding(gpioName);
	if (gpio_dev == NULL) {
		printk("Error: Could not get %s device\n", gpioName);
		return -EIO;
	}
    int err = set_gpio_dev(gpio_dev, true);     // set GPIO_0 device and enable debugging
    if (err) {
        return -EIO;
    }
    return 0;
}

uint8_t init_i2c(void) {
	i2c_dev = device_get_binding(I2C_DEV);
    if (!i2c_dev)
    {
        printk("I2C_0 error\n");
        return -1;
    }
    else
    {
        printk("I2C_0 Init OK\n");
        return 0;
    }
}

uint8_t init_gps(void) {
	if (gps_begin(i2c_dev) != 0)
  	{
    	printk("u-blox GPS init error!\n");
        return -1;
  	}
    return 0;
}


void main(void) {
	printk("Zephyr u-blox example\n");

	int err;
    err = init_gpio();
    if (err) {
        return;
    }
    err = init_i2c();
    if (err) {
        return;
    }

	err = init_gps();
    if (err) {
        return;
    }

	while(1) {
        //check_ublox(); // See if new data is available. Process bytes as they come in.
        get_position();
        get_datetime();
        k_msleep(250); // Don't pound too hard on the I2C bus
	}
}