/*!
 *  @file Adafruit_BQ25628E.h
 *
 *  This is a library for the BQ25628E I2C Battery Charger
 *
 *  Designed specifically to work with the Adafruit BQ25628E Breakout
 *  ----> http://www.adafruit.com/products/
 *
 *  Pick one up today in the adafruit shop!
 *
 *  These displays use I2C to communicate, 2 pins are required to interface.
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @author   Limor 'ladyada' Fried with assistance from Claude Code
 *  @license  MIT (see license.txt)
 */

#ifndef _ADAFRUIT_BQ25628E_H
#define _ADAFRUIT_BQ25628E_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>

/*! Default I2C address for the BQ25628E */
#define BQ25628E_DEFAULT_ADDR 0x6A

/*! Register addresses for the BQ25628E */
#define BQ25628E_REG_CHARGE_CURRENT_LIMIT 0x02
#define BQ25628E_REG_CHARGE_VOLTAGE_LIMIT 0x04
#define BQ25628E_REG_INPUT_CURRENT_LIMIT 0x06
#define BQ25628E_REG_INPUT_VOLTAGE_LIMIT 0x08
#define BQ25628E_REG_MINIMAL_SYSTEM_VOLTAGE 0x0E
#define BQ25628E_REG_PRECHARGE_CONTROL 0x10
#define BQ25628E_REG_TERMINATION_CONTROL 0x12
#define BQ25628E_REG_CHARGE_CONTROL 0x14
#define BQ25628E_REG_CHARGE_TIMER_CONTROL 0x15
#define BQ25628E_REG_CHARGER_CONTROL_0 0x16
#define BQ25628E_REG_CHARGER_CONTROL_1 0x17
#define BQ25628E_REG_CHARGER_CONTROL_2 0x18
#define BQ25628E_REG_CHARGER_CONTROL_3 0x19
#define BQ25628E_REG_NTC_CONTROL_0 0x1A
#define BQ25628E_REG_NTC_CONTROL_1 0x1B
#define BQ25628E_REG_NTC_CONTROL_2 0x1C
#define BQ25628E_REG_CHARGER_STATUS_0 0x1D
#define BQ25628E_REG_CHARGER_STATUS_1 0x1E
#define BQ25628E_REG_FAULT_STATUS_0 0x1F
#define BQ25628E_REG_CHARGER_FLAG_0 0x20
#define BQ25628E_REG_CHARGER_FLAG_1 0x21
#define BQ25628E_REG_FAULT_FLAG_0 0x22
#define BQ25628E_REG_CHARGER_MASK_0 0x23
#define BQ25628E_REG_CHARGER_MASK_1 0x24
#define BQ25628E_REG_FAULT_MASK_0 0x25
#define BQ25628E_REG_ADC_CONTROL 0x26
#define BQ25628E_REG_ADC_FUNCTION_DISABLE_0 0x27
#define BQ25628E_REG_IBUS_ADC 0x28
#define BQ25628E_REG_IBAT_ADC 0x2A
#define BQ25628E_REG_VBUS_ADC 0x2C
#define BQ25628E_REG_VPMID_ADC 0x2E
#define BQ25628E_REG_VBAT_ADC 0x30
#define BQ25628E_REG_VSYS_ADC 0x32
#define BQ25628E_REG_TS_ADC 0x34
#define BQ25628E_REG_TDIE_ADC 0x36
#define BQ25628E_REG_PART_INFORMATION 0x38

/*!
 * @brief Class that stores state and functions for interacting with
 *        the BQ25628E I2C Battery Charger
 */
class Adafruit_BQ25628E {
public:
  Adafruit_BQ25628E();
  ~Adafruit_BQ25628E();
  bool begin(uint8_t i2c_addr = BQ25628E_DEFAULT_ADDR, TwoWire *wire = &Wire);

private:
  Adafruit_I2CDevice *i2c_dev; /*!< Pointer to I2C bus interface */
};

#endif