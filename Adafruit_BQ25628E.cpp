/*!
 *  @file Adafruit_BQ25628E.cpp
 *
 *  @mainpage Adafruit BQ25628E I2C Battery Charger
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the BQ25628E I2C Battery Charger
 *
 *  Designed specifically to work with the Adafruit BQ25628E Breakout
 *  ----> http://www.adafruit.com/products/
 *
 *  Pick one up today in the adafruit shop!
 *
 *  These chips use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor 'ladyada' Fried with assistance from Claude Code
 *
 *  @section license License
 *
 *  MIT (see license.txt)
 */

#include "Adafruit_BQ25628E.h"

/*!
 *    @brief  Instantiates a new BQ25628E class
 */
Adafruit_BQ25628E::Adafruit_BQ25628E() {}

/*!
 *    @brief  Cleans up the BQ25628E
 */
Adafruit_BQ25628E::~Adafruit_BQ25628E() {
  if (i2c_dev) {
    delete i2c_dev;
  }
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_addr
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_BQ25628E::begin(uint8_t i2c_addr, TwoWire *wire) {
  if (i2c_dev) {
    delete i2c_dev;
  }

  i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  // Verify chip connection by reading part information register
  Adafruit_BusIO_Register part_info_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_PART_INFORMATION);
  uint8_t part_info = part_info_reg.read();
  
  // Check for valid BQ25628E part ID (0x22)
  if (part_info != 0x22) {
    return false;
  }
  
  return true;
}