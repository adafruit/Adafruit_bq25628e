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

/*!
 *    @brief  Sets the charge current limit
 *    @param  current_a
 *            Current in Amps (0.04A to 2.0A in 0.04A steps)
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setChargeCurrentLimitA(float current_a) {
  // Convert current to register value: ICHG = (current_a - 0.04) / 0.04 + 1
  // Range: 40mA-2000mA (1h-32h), so offset by 1
  uint8_t ichg_value = (uint8_t)((current_a - 0.04f) / 0.04f + 1);
  
  // Clamp to valid range (1-50, where 50 = 32h)
  if (ichg_value < 1) {
    ichg_value = 1;
  }
  if (ichg_value > 50) {
    ichg_value = 50;
  }
  
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register charge_current_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_CURRENT_LIMIT, 2, LSBFIRST);
  
  // Create register bits object for ICHG field (bits 10:5, 6 bits, shift by 5)
  Adafruit_BusIO_RegisterBits ichg_bits = Adafruit_BusIO_RegisterBits(&charge_current_reg, 6, 5);
  
  return ichg_bits.write(ichg_value);
}

/*!
 *    @brief  Gets the charge current limit
 *    @return Current limit in Amps
 */
float Adafruit_BQ25628E::getChargeCurrentLimitA() {
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register charge_current_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_CURRENT_LIMIT, 2, LSBFIRST);
  
  // Create register bits object for ICHG field (bits 10:5, 6 bits, shift by 5)
  Adafruit_BusIO_RegisterBits ichg_bits = Adafruit_BusIO_RegisterBits(&charge_current_reg, 6, 5);
  
  uint8_t ichg_value = ichg_bits.read();
  
  // Convert register value to current: current_a = (ICHG - 1) * 0.04 + 0.04
  return (ichg_value - 1) * 0.04f + 0.04f;
}

/*!
 *    @brief  Sets the charge voltage limit
 *    @param  voltage_v
 *            Voltage in Volts (3.5V to 4.8V in 0.01V steps)
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setChargeVoltageLimitV(float voltage_v) {
  // Convert voltage to mV and then to register value
  uint16_t voltage_mv = (uint16_t)(voltage_v * 1000);
  uint16_t vreg_value = voltage_mv / 10;
  
  // Clamp to valid range (350-480 for 3.5V-4.8V)
  if (vreg_value < 350) {
    vreg_value = 350;
  }
  if (vreg_value > 480) {
    vreg_value = 480;
  }
  
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register charge_voltage_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_VOLTAGE_LIMIT, 2, LSBFIRST);
  
  // Create register bits object for VREG field (bits 11:3, 9 bits, shift by 3)
  Adafruit_BusIO_RegisterBits vreg_bits = Adafruit_BusIO_RegisterBits(&charge_voltage_reg, 9, 3);
  
  return vreg_bits.write(vreg_value);
}

/*!
 *    @brief  Gets the charge voltage limit
 *    @return Voltage limit in Volts
 */
float Adafruit_BQ25628E::getChargeVoltageLimitV() {
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register charge_voltage_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_VOLTAGE_LIMIT, 2, LSBFIRST);
  
  // Create register bits object for VREG field (bits 11:3, 9 bits, shift by 3)
  Adafruit_BusIO_RegisterBits vreg_bits = Adafruit_BusIO_RegisterBits(&charge_voltage_reg, 9, 3);
  
  uint16_t vreg_value = vreg_bits.read();
  
  // Convert register value to voltage: voltage_v = vreg_value * 0.01
  return vreg_value * 0.01f;
}

/*!
 *    @brief  Sets the input current limit
 *    @param  current_a
 *            Current in Amps (0.1A to 3.2A in 0.02A steps)
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setInputCurrentLimitA(float current_a) {
  // Convert current to mA and then to register value
  uint16_t current_ma = (uint16_t)(current_a * 1000);
  uint8_t iindpm_value = current_ma / 20;
  
  // Clamp to valid range (5-160 for 0.1A-3.2A)
  if (iindpm_value < 5) {
    iindpm_value = 5;
  }
  if (iindpm_value > 160) {
    iindpm_value = 160;
  }
  
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register input_current_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_INPUT_CURRENT_LIMIT, 2, LSBFIRST);
  
  // Create register bits object for IINDPM field (8 bits, shift by 4)
  Adafruit_BusIO_RegisterBits iindpm_bits = Adafruit_BusIO_RegisterBits(&input_current_reg, 8, 4);
  
  return iindpm_bits.write(iindpm_value);
}

/*!
 *    @brief  Gets the input current limit
 *    @return Current limit in Amps
 */
float Adafruit_BQ25628E::getInputCurrentLimitA() {
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register input_current_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_INPUT_CURRENT_LIMIT, 2, LSBFIRST);
  
  // Create register bits object for IINDPM field (8 bits, shift by 4)
  Adafruit_BusIO_RegisterBits iindpm_bits = Adafruit_BusIO_RegisterBits(&input_current_reg, 8, 4);
  
  uint8_t iindpm_value = iindpm_bits.read();
  
  // Convert register value to current: current_a = iindpm_value * 0.02
  return iindpm_value * 0.02f;
}

/*!
 *    @brief  Sets the input voltage limit
 *    @param  voltage_v
 *            Voltage in Volts (3.8V to 16.8V in 0.04V steps)
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setInputVoltageLimitV(float voltage_v) {
  // Convert voltage to mV and then to register value
  uint16_t voltage_mv = (uint16_t)(voltage_v * 1000);
  uint16_t vindpm_value = voltage_mv / 40;
  
  // Clamp to valid range (95-420 for 3.8V-16.8V)
  if (vindpm_value < 95) {
    vindpm_value = 95;
  }
  if (vindpm_value > 420) {
    vindpm_value = 420;
  }
  
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register input_voltage_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_INPUT_VOLTAGE_LIMIT, 2, LSBFIRST);
  
  // Create register bits object for VINDPM field (9 bits, shift by 5)
  Adafruit_BusIO_RegisterBits vindpm_bits = Adafruit_BusIO_RegisterBits(&input_voltage_reg, 9, 5);
  
  return vindpm_bits.write(vindpm_value);
}

/*!
 *    @brief  Gets the input voltage limit
 *    @return Voltage limit in Volts
 */
float Adafruit_BQ25628E::getInputVoltageLimitV() {
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register input_voltage_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_INPUT_VOLTAGE_LIMIT, 2, LSBFIRST);
  
  // Create register bits object for VINDPM field (9 bits, shift by 5)
  Adafruit_BusIO_RegisterBits vindpm_bits = Adafruit_BusIO_RegisterBits(&input_voltage_reg, 9, 5);
  
  uint16_t vindpm_value = vindpm_bits.read();
  
  // Convert register value to voltage: voltage_v = vindpm_value * 0.04
  return vindpm_value * 0.04f;
}

/*!
 *    @brief  Sets the minimal system voltage
 *    @param  voltage_v
 *            Voltage in Volts (2.56V to 3.84V in 0.08V steps)
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setMinimalSystemVoltageV(float voltage_v) {
  // Convert voltage to mV and then to register value
  uint16_t voltage_mv = (uint16_t)(voltage_v * 1000);
  uint8_t vsysmin_value = voltage_mv / 80;
  
  // Clamp to valid range (32-48 for 2.56V-3.84V)
  if (vsysmin_value < 32) {
    vsysmin_value = 32;
  }
  if (vsysmin_value > 48) {
    vsysmin_value = 48;
  }
  
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register system_voltage_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_MINIMAL_SYSTEM_VOLTAGE, 2, LSBFIRST);
  
  // Create register bits object for VSYSMIN field (6 bits, shift by 6)
  Adafruit_BusIO_RegisterBits vsysmin_bits = Adafruit_BusIO_RegisterBits(&system_voltage_reg, 6, 6);
  
  return vsysmin_bits.write(vsysmin_value);
}

/*!
 *    @brief  Gets the minimal system voltage
 *    @return Voltage in Volts
 */
float Adafruit_BQ25628E::getMinimalSystemVoltageV() {
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register system_voltage_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_MINIMAL_SYSTEM_VOLTAGE, 2, LSBFIRST);
  
  // Create register bits object for VSYSMIN field (6 bits, shift by 6)
  Adafruit_BusIO_RegisterBits vsysmin_bits = Adafruit_BusIO_RegisterBits(&system_voltage_reg, 6, 6);
  
  uint8_t vsysmin_value = vsysmin_bits.read();
  
  // Convert register value to voltage: voltage_v = vsysmin_value * 0.08
  return vsysmin_value * 0.08f;
}