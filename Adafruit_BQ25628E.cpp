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

/*!
 *    @brief  Sets the precharge current limit
 *    @param  current_a
 *            Current in Amps (0.01A to 0.31A in 0.01A steps)
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setPrechargeCurrentLimitA(float current_a) {
  // Convert current to mA and then to register value
  uint16_t current_ma = (uint16_t)(current_a * 1000);
  uint8_t iprechg_value = current_ma / 10;
  
  // Clamp to valid range (1-31 for 0.01A-0.31A)
  if (iprechg_value < 1) {
    iprechg_value = 1;
  }
  if (iprechg_value > 31) {
    iprechg_value = 31;
  }
  
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register precharge_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_PRECHARGE_CONTROL, 2, LSBFIRST);
  
  // Create register bits object for IPRECHG field (5 bits, shift by 3)
  Adafruit_BusIO_RegisterBits iprechg_bits = Adafruit_BusIO_RegisterBits(&precharge_reg, 5, 3);
  
  return iprechg_bits.write(iprechg_value);
}

/*!
 *    @brief  Gets the precharge current limit
 *    @return Current limit in Amps
 */
float Adafruit_BQ25628E::getPrechargeCurrentLimitA() {
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register precharge_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_PRECHARGE_CONTROL, 2, LSBFIRST);
  
  // Create register bits object for IPRECHG field (5 bits, shift by 3)
  Adafruit_BusIO_RegisterBits iprechg_bits = Adafruit_BusIO_RegisterBits(&precharge_reg, 5, 3);
  
  uint8_t iprechg_value = iprechg_bits.read();
  
  // Convert register value to current: current_a = iprechg_value * 0.01
  return iprechg_value * 0.01f;
}

/*!
 *    @brief  Sets the termination current threshold
 *    @param  current_a
 *            Current in Amps (0.005A to 0.31A in 0.005A steps)
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setTerminationCurrentThresholdA(float current_a) {
  // Convert current to mA and then to register value
  uint16_t current_ma = (uint16_t)(current_a * 1000);
  uint8_t iterm_value = current_ma / 5;
  
  // Clamp to valid range (1-62 for 0.005A-0.31A)
  if (iterm_value < 1) {
    iterm_value = 1;
  }
  if (iterm_value > 62) {
    iterm_value = 62;
  }
  
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register termination_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_TERMINATION_CONTROL, 2, LSBFIRST);
  
  // Create register bits object for ITERM field (6 bits, shift by 2)
  Adafruit_BusIO_RegisterBits iterm_bits = Adafruit_BusIO_RegisterBits(&termination_reg, 6, 2);
  
  return iterm_bits.write(iterm_value);
}

/*!
 *    @brief  Gets the termination current threshold
 *    @return Current threshold in Amps
 */
float Adafruit_BQ25628E::getTerminationCurrentThresholdA() {
  // Create register object (16-bit register, little endian)
  Adafruit_BusIO_Register termination_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_TERMINATION_CONTROL, 2, LSBFIRST);
  
  // Create register bits object for ITERM field (6 bits, shift by 2)
  Adafruit_BusIO_RegisterBits iterm_bits = Adafruit_BusIO_RegisterBits(&termination_reg, 6, 2);
  
  uint8_t iterm_value = iterm_bits.read();
  
  // Convert register value to current: current_a = iterm_value * 0.005
  return iterm_value * 0.005f;
}

/*!
 *    @brief  Sets the trickle charging current
 *    @param  use_40ma
 *            True for 40mA trickle current, false for 10mA trickle current
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setTrickleCurrent(bool use_40ma) {
  Adafruit_BusIO_Register charge_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_CONTROL, 1);
  Adafruit_BusIO_RegisterBits itrickle_bit = Adafruit_BusIO_RegisterBits(&charge_control_reg, 1, 5);
  
  return itrickle_bit.write(use_40ma ? 1 : 0);
}

/*!
 *    @brief  Gets the trickle charging current setting
 *    @return True if 40mA trickle current, false if 10mA trickle current
 */
bool Adafruit_BQ25628E::getTrickleCurrent() {
  Adafruit_BusIO_Register charge_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_CONTROL, 1);
  Adafruit_BusIO_RegisterBits itrickle_bit = Adafruit_BusIO_RegisterBits(&charge_control_reg, 1, 5);
  
  return itrickle_bit.read() == 1;
}

/*!
 *    @brief  Sets charge termination enable/disable
 *    @param  enable
 *            True to enable termination, false to disable
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setEnableTermination(bool enable) {
  Adafruit_BusIO_Register charge_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_CONTROL, 1);
  Adafruit_BusIO_RegisterBits en_term_bit = Adafruit_BusIO_RegisterBits(&charge_control_reg, 1, 2);
  
  return en_term_bit.write(enable ? 1 : 0);
}

/*!
 *    @brief  Gets charge termination enable/disable status
 *    @return True if termination enabled, false if disabled
 */
bool Adafruit_BQ25628E::getEnableTermination() {
  Adafruit_BusIO_Register charge_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_CONTROL, 1);
  Adafruit_BusIO_RegisterBits en_term_bit = Adafruit_BusIO_RegisterBits(&charge_control_reg, 1, 2);
  
  return en_term_bit.read() == 1;
}

/*!
 *    @brief  Sets VINDPM battery voltage tracking
 *    @param  enable
 *            True for VBAT + 400mV tracking, false for register-only VINDPM
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setVINDPMbatTrack(bool enable) {
  Adafruit_BusIO_Register charge_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_CONTROL, 1);
  Adafruit_BusIO_RegisterBits vindpm_bat_track_bit = Adafruit_BusIO_RegisterBits(&charge_control_reg, 1, 1);
  
  return vindpm_bat_track_bit.write(enable ? 1 : 0);
}

/*!
 *    @brief  Gets VINDPM battery voltage tracking status
 *    @return True if VBAT + 400mV tracking enabled, false if register-only VINDPM
 */
bool Adafruit_BQ25628E::getVINDPMbatTrack() {
  Adafruit_BusIO_Register charge_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_CONTROL, 1);
  Adafruit_BusIO_RegisterBits vindpm_bat_track_bit = Adafruit_BusIO_RegisterBits(&charge_control_reg, 1, 1);
  
  return vindpm_bat_track_bit.read() == 1;
}

/*!
 *    @brief  Sets charge timer enable/disable
 *    @param  enable
 *            True to enable safety timers, false to disable
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setEnableSafetyTimers(bool enable) {
  Adafruit_BusIO_Register timer_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_TIMER_CONTROL, 1);
  Adafruit_BusIO_RegisterBits en_safety_tmrs_bit = Adafruit_BusIO_RegisterBits(&timer_control_reg, 1, 2);
  
  return en_safety_tmrs_bit.write(enable ? 1 : 0);
}

/*!
 *    @brief  Gets charge timer enable/disable status
 *    @return True if safety timers enabled, false if disabled
 */
bool Adafruit_BQ25628E::getEnableSafetyTimers() {
  Adafruit_BusIO_Register timer_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_TIMER_CONTROL, 1);
  Adafruit_BusIO_RegisterBits en_safety_tmrs_bit = Adafruit_BusIO_RegisterBits(&timer_control_reg, 1, 2);
  
  return en_safety_tmrs_bit.read() == 1;
}

/*!
 *    @brief  Sets precharge timer setting
 *    @param  short_timer
 *            True for 0.62 hours, false for 2.5 hours (default)
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setPrechargeTimer(bool short_timer) {
  Adafruit_BusIO_Register timer_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_TIMER_CONTROL, 1);
  Adafruit_BusIO_RegisterBits prechg_tmr_bit = Adafruit_BusIO_RegisterBits(&timer_control_reg, 1, 1);
  
  return prechg_tmr_bit.write(short_timer ? 1 : 0);
}

/*!
 *    @brief  Gets precharge timer setting
 *    @return True if 0.62 hours, false if 2.5 hours
 */
bool Adafruit_BQ25628E::getPrechargeTimer() {
  Adafruit_BusIO_Register timer_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_TIMER_CONTROL, 1);
  Adafruit_BusIO_RegisterBits prechg_tmr_bit = Adafruit_BusIO_RegisterBits(&timer_control_reg, 1, 1);
  
  return prechg_tmr_bit.read() == 1;
}

/*!
 *    @brief  Sets fast charge timer setting
 *    @param  long_timer
 *            True for 28 hours, false for 14.5 hours (default)
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setFastchargeTimer(bool long_timer) {
  Adafruit_BusIO_Register timer_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_TIMER_CONTROL, 1);
  Adafruit_BusIO_RegisterBits chg_tmr_bit = Adafruit_BusIO_RegisterBits(&timer_control_reg, 1, 0);
  
  return chg_tmr_bit.write(long_timer ? 1 : 0);
}

/*!
 *    @brief  Gets fast charge timer setting
 *    @return True if 28 hours, false if 14.5 hours
 */
bool Adafruit_BQ25628E::getFastchargeTimer() {
  Adafruit_BusIO_Register timer_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGE_TIMER_CONTROL, 1);
  Adafruit_BusIO_RegisterBits chg_tmr_bit = Adafruit_BusIO_RegisterBits(&timer_control_reg, 1, 0);
  
  return chg_tmr_bit.read() == 1;
}

/*!
 *    @brief  Sets auto battery discharge during battery OVP
 *    @param  enable
 *            True to enable auto discharge during battery OVP, false to disable
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setAutoBatteryDischarge(bool enable) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits en_auto_ibatdis_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 7);
  
  return en_auto_ibatdis_bit.write(enable ? 1 : 0);
}

/*!
 *    @brief  Gets auto battery discharge setting
 *    @return True if auto discharge enabled, false if disabled
 */
bool Adafruit_BQ25628E::getAutoBatteryDischarge() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits en_auto_ibatdis_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 7);
  
  return en_auto_ibatdis_bit.read() == 1;
}

/*!
 *    @brief  Forces battery discharge current (~30mA)
 *    @param  enable
 *            True to force discharge current, false for idle
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setForceBatteryDischarge(bool enable) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits force_ibatdis_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 6);
  
  return force_ibatdis_bit.write(enable ? 1 : 0);
}

/*!
 *    @brief  Gets forced battery discharge status
 *    @return True if discharge current forced, false if idle
 */
bool Adafruit_BQ25628E::getForceBatteryDischarge() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits force_ibatdis_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 6);
  
  return force_ibatdis_bit.read() == 1;
}

/*!
 *    @brief  Sets charger enable/disable
 *    @param  enable
 *            True to enable charging, false to disable
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setEnableCharging(bool enable) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits en_chg_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 5);
  
  return en_chg_bit.write(enable ? 1 : 0);
}

/*!
 *    @brief  Gets charger enable status
 *    @return True if charging enabled, false if disabled
 */
bool Adafruit_BQ25628E::getEnableCharging() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits en_chg_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 5);
  
  return en_chg_bit.read() == 1;
}

/*!
 *    @brief  Sets HIZ mode enable/disable
 *    @param  enable
 *            True to enable HIZ mode, false to disable
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setHighZ(bool enable) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits en_hiz_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 4);
  
  return en_hiz_bit.write(enable ? 1 : 0);
}

/*!
 *    @brief  Gets HIZ mode status
 *    @return True if HIZ mode enabled, false if disabled
 */
bool Adafruit_BQ25628E::getHighZ() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits en_hiz_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 4);
  
  return en_hiz_bit.read() == 1;
}

/*!
 *    @brief  Forces PMID discharge current (~30mA)
 *    @param  enable
 *            True to force PMID discharge, false to disable
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setForcePMIDDischarge(bool enable) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits force_pmid_dis_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 3);
  
  return force_pmid_dis_bit.write(enable ? 1 : 0);
}

/*!
 *    @brief  Gets forced PMID discharge status
 *    @return True if PMID discharge forced, false if disabled
 */
bool Adafruit_BQ25628E::getForcePMIDDischarge() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits force_pmid_dis_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 3);
  
  return force_pmid_dis_bit.read() == 1;
}

/*!
 *    @brief  Resets the I2C watchdog timer
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::resetWatchdog() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits wd_rst_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 2);
  
  return wd_rst_bit.write(1);
}

/*!
 *    @brief  Sets watchdog timer setting
 *    @param  setting
 *            Watchdog timer setting from bq25628e_watchdog_t enum
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setWatchdog(bq25628e_watchdog_t setting) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits watchdog_bits = Adafruit_BusIO_RegisterBits(&charger_control_reg, 2, 0);
  
  return watchdog_bits.write((uint8_t)setting);
}

/*!
 *    @brief  Gets watchdog timer setting
 *    @return Current watchdog timer setting
 */
bq25628e_watchdog_t Adafruit_BQ25628E::getWatchdog() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits watchdog_bits = Adafruit_BusIO_RegisterBits(&charger_control_reg, 2, 0);
  
  return (bq25628e_watchdog_t)watchdog_bits.read();
}