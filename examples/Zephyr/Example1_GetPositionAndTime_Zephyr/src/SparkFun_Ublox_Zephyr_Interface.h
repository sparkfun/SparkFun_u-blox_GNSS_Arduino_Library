/*
    This is an interface that connects the CPP u-blox library with the main C code.
    Added to make it possible to run u-blox lib on Zephyr (NCS)

    This port was made by Vid Rajtmajer <vid@irnas.eu>, www.irnas.eu
*/
#include <time.h>
#include <zephyr.h>

#ifndef _UBLOX_LIB_INTERFACE_H_
#define _UBLOX_LIB_INTERFACE_H_

#ifdef __cplusplus
extern "C" {
#endif

    uint8_t set_gpio_dev(struct device *gpio_dev, uint8_t enable_debug); // init GPIO
    uint8_t gps_begin(struct device *i2c_dev);                           // initialize I2C and check if GPS device respons
    void pipe_nmea_sentences(void);                                      // print NMEA sentences

    void check_ublox(void);  // Check for available bytes from the device
    int get_position(void); // Get position information
    void get_datetime(void); // Get date and time information

#ifdef __cplusplus
}
#endif

#endif  //UBLOX_LIB_INTERFACE_H_