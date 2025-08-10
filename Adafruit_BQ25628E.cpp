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

/*!
 *    @brief  Resets registers to default values and resets timer
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::reset() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_1, 1);
  Adafruit_BusIO_RegisterBits reg_rst_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 7);
  
  // Set reset bit
  if (!reg_rst_bit.write(1)) {
    return false;
  }
  
  // Wait for bit to clear (indicates reset complete)
  uint32_t timeout = millis() + 1000; // 1 second timeout
  while (millis() < timeout) {
    if (reg_rst_bit.read() == 0) {
      return true; // Reset completed
    }
    delay(1);
  }
  
  return false; // Timeout - reset may have failed
}

/*!
 *    @brief  Sets thermal regulation threshold
 *    @param  temp_120c
 *            True for 120°C threshold, false for 60°C threshold
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setThermalRegulation(bool temp_120c) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_1, 1);
  Adafruit_BusIO_RegisterBits treg_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 6);
  
  return treg_bit.write(temp_120c ? 1 : 0);
}

/*!
 *    @brief  Gets thermal regulation threshold setting
 *    @return True if 120°C threshold, false if 60°C threshold
 */
bool Adafruit_BQ25628E::getThermalRegulation() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_1, 1);
  Adafruit_BusIO_RegisterBits treg_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 6);
  
  return treg_bit.read() == 1;
}

/*!
 *    @brief  Sets converter switching frequency
 *    @param  frequency
 *            Frequency setting from bq25628e_conv_freq_t enum
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setConverterFrequency(bq25628e_conv_freq_t frequency) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_1, 1);
  Adafruit_BusIO_RegisterBits conv_freq_bits = Adafruit_BusIO_RegisterBits(&charger_control_reg, 2, 4);
  
  return conv_freq_bits.write((uint8_t)frequency);
}

/*!
 *    @brief  Gets converter switching frequency setting
 *    @return Current frequency setting
 */
bq25628e_conv_freq_t Adafruit_BQ25628E::getConverterFrequency() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_1, 1);
  Adafruit_BusIO_RegisterBits conv_freq_bits = Adafruit_BusIO_RegisterBits(&charger_control_reg, 2, 4);
  
  return (bq25628e_conv_freq_t)conv_freq_bits.read();
}

/*!
 *    @brief  Sets VBUS overvoltage protection threshold
 *    @param  high_threshold
 *            True for 18.5V threshold, false for 6.3V threshold
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setVBUSOvervoltage(bool high_threshold) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_1, 1);
  Adafruit_BusIO_RegisterBits vbus_ovp_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 0);
  
  return vbus_ovp_bit.write(high_threshold ? 1 : 0);
}

/*!
 *    @brief  Gets VBUS overvoltage protection threshold setting
 *    @return True if 18.5V threshold, false if 6.3V threshold
 */
bool Adafruit_BQ25628E::getVBUSOvervoltage() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_1, 1);
  Adafruit_BusIO_RegisterBits vbus_ovp_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 0);
  
  return vbus_ovp_bit.read() == 1;
}

/*!
 *    @brief  Sets BATFET control mode
 *    @param  control
 *            BATFET control setting from bq25628e_batfet_ctrl_t enum
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setBATFETcontrol(bq25628e_batfet_ctrl_t control) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_2, 1);
  Adafruit_BusIO_RegisterBits batfet_ctrl_bits = Adafruit_BusIO_RegisterBits(&charger_control_reg, 2, 0);
  
  return batfet_ctrl_bits.write((uint8_t)control);
}

/*!
 *    @brief  Gets BATFET control mode setting
 *    @return Current BATFET control setting
 */
bq25628e_batfet_ctrl_t Adafruit_BQ25628E::getBATFETcontrol() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_2, 1);
  Adafruit_BusIO_RegisterBits batfet_ctrl_bits = Adafruit_BusIO_RegisterBits(&charger_control_reg, 2, 0);
  
  return (bq25628e_batfet_ctrl_t)batfet_ctrl_bits.read();
}

/*!
 *    @brief  Sets battery discharge peak current protection
 *    @param  peak_12a
 *            True for 12A peak current, false for 6A peak current
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setPeakBattDischarge(bool peak_12a) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_3, 1);
  Adafruit_BusIO_RegisterBits ibat_pk_bits = Adafruit_BusIO_RegisterBits(&charger_control_reg, 2, 6);
  
  return ibat_pk_bits.write(peak_12a ? 0b11 : 0b10);
}

/*!
 *    @brief  Gets battery discharge peak current protection setting
 *    @return True if 12A peak current, false if 6A peak current
 */
bool Adafruit_BQ25628E::getPeakBattDischarge() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_3, 1);
  Adafruit_BusIO_RegisterBits ibat_pk_bits = Adafruit_BusIO_RegisterBits(&charger_control_reg, 2, 6);
  
  uint8_t value = ibat_pk_bits.read();
  return (value == 0b11); // 11b = 12A, 10b = 6A
}

/*!
 *    @brief  Sets VBAT UVLO threshold
 *    @param  low_threshold
 *            True for 1.8V UVLO/1.85V SHORT, false for 2.2V UVLO/2.05V SHORT
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setVBatUVLO(bool low_threshold) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_3, 1);
  Adafruit_BusIO_RegisterBits vbat_uvlo_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 5);
  
  return vbat_uvlo_bit.write(low_threshold ? 1 : 0);
}

/*!
 *    @brief  Gets VBAT UVLO threshold setting
 *    @return True if 1.8V UVLO/1.85V SHORT, false if 2.2V UVLO/2.05V SHORT
 */
bool Adafruit_BQ25628E::getVBatUVLO() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_3, 1);
  Adafruit_BusIO_RegisterBits vbat_uvlo_bit = Adafruit_BusIO_RegisterBits(&charger_control_reg, 1, 5);
  
  return vbat_uvlo_bit.read() == 1;
}

/*!
 *    @brief  Sets charge rate for fast charge stage
 *    @param  rate
 *            Charge rate setting from bq25628e_charge_rate_t enum
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setChargeRate(bq25628e_charge_rate_t rate) {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_3, 1);
  Adafruit_BusIO_RegisterBits chg_rate_bits = Adafruit_BusIO_RegisterBits(&charger_control_reg, 2, 0);
  
  return chg_rate_bits.write((uint8_t)rate);
}

/*!
 *    @brief  Gets charge rate setting
 *    @return Current charge rate setting
 */
bq25628e_charge_rate_t Adafruit_BQ25628E::getChargeRate() {
  Adafruit_BusIO_Register charger_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_CONTROL_3, 1);
  Adafruit_BusIO_RegisterBits chg_rate_bits = Adafruit_BusIO_RegisterBits(&charger_control_reg, 2, 0);
  
  return (bq25628e_charge_rate_t)chg_rate_bits.read();
}

/*!
 *    @brief  Sets thermistor feedback ignore
 *    @param  ignore
 *            True to ignore TS feedback, false to use TS feedback
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setIgnoreThermistor(bool ignore) {
  Adafruit_BusIO_Register ntc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_NTC_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits ts_ignore_bit = Adafruit_BusIO_RegisterBits(&ntc_control_reg, 1, 7);
  
  return ts_ignore_bit.write(ignore ? 1 : 0);
}

/*!
 *    @brief  Gets thermistor feedback ignore setting
 *    @return True if TS feedback ignored, false if TS feedback used
 */
bool Adafruit_BQ25628E::getIgnoreThermistor() {
  Adafruit_BusIO_Register ntc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_NTC_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits ts_ignore_bit = Adafruit_BusIO_RegisterBits(&ntc_control_reg, 1, 7);
  
  return ts_ignore_bit.read() == 1;
}

/*!
 *    @brief  Sets thermistor cool zone current setting
 *    @param  setting
 *            Current setting from bq25628e_therm_curr_t enum
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setCoolThermistorCurrent(bq25628e_therm_curr_t setting) {
  Adafruit_BusIO_Register ntc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_NTC_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits ts_iset_cool_bits = Adafruit_BusIO_RegisterBits(&ntc_control_reg, 2, 0);
  
  return ts_iset_cool_bits.write((uint8_t)setting);
}

/*!
 *    @brief  Gets thermistor cool zone current setting
 *    @return Current cool zone setting
 */
bq25628e_therm_curr_t Adafruit_BQ25628E::getCoolThermistorCurrent() {
  Adafruit_BusIO_Register ntc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_NTC_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits ts_iset_cool_bits = Adafruit_BusIO_RegisterBits(&ntc_control_reg, 2, 0);
  
  return (bq25628e_therm_curr_t)ts_iset_cool_bits.read();
}

/*!
 *    @brief  Sets thermistor warm zone current setting
 *    @param  setting
 *            Current setting from bq25628e_therm_curr_t enum
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setWarmThermistorCurrent(bq25628e_therm_curr_t setting) {
  Adafruit_BusIO_Register ntc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_NTC_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits ts_iset_warm_bits = Adafruit_BusIO_RegisterBits(&ntc_control_reg, 2, 2);
  
  return ts_iset_warm_bits.write((uint8_t)setting);
}

/*!
 *    @brief  Gets thermistor warm zone current setting
 *    @return Current warm zone setting
 */
bq25628e_therm_curr_t Adafruit_BQ25628E::getWarmThermistorCurrent() {
  Adafruit_BusIO_Register ntc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_NTC_CONTROL_0, 1);
  Adafruit_BusIO_RegisterBits ts_iset_warm_bits = Adafruit_BusIO_RegisterBits(&ntc_control_reg, 2, 2);
  
  return (bq25628e_therm_curr_t)ts_iset_warm_bits.read();
}

/*!
 *    @brief  Gets combined charger status flags from both status registers
 *    @return 16-bit status flags: bits 15:8 = REG0x1E, bits 7:0 = REG0x1D
 */
uint16_t Adafruit_BQ25628E::getChargerStatusFlags() {
  // Read REG0x1D (Charger Status 0)
  Adafruit_BusIO_Register status0_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_STATUS_0, 1);
  uint8_t status0 = status0_reg.read();
  
  // Read REG0x1E (Charger Status 1)
  Adafruit_BusIO_Register status1_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_STATUS_1, 1);
  uint8_t status1 = status1_reg.read();
  
  // Combine into 16-bit value: high byte = status1, low byte = status0
  return ((uint16_t)status1 << 8) | status0;
}

/*!
 *    @brief  Gets fault status flags from REG0x1F
 *    @return 8-bit fault status flags from REG0x1F_FAULT_Status_0
 */
uint8_t Adafruit_BQ25628E::getFaultStatusFlags() {
  // Read REG0x1F (FAULT Status 0)
  Adafruit_BusIO_Register fault_status_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_FAULT_STATUS_0, 1);
  
  return fault_status_reg.read();
}

/*!
 *    @brief  Gets combined charger flag registers (clears flags on read)
 *    @return 16-bit flag values: bits 15:8 = REG0x21, bits 7:0 = REG0x20
 *    @note   Reading this function clears all flag bits automatically
 */
uint16_t Adafruit_BQ25628E::getChargerFlags() {
  // Read REG0x20 (Charger Flag 0) - clears flags on read
  Adafruit_BusIO_Register flag0_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_FLAG_0, 1);
  uint8_t flag0 = flag0_reg.read();
  
  // Read REG0x21 (Charger Flag 1) - clears flags on read
  Adafruit_BusIO_Register flag1_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_FLAG_1, 1);
  uint8_t flag1 = flag1_reg.read();
  
  // Combine into 16-bit value: high byte = flag1, low byte = flag0
  return ((uint16_t)flag1 << 8) | flag0;
}

/*!
 *    @brief  Gets fault flag register (clears flags on read)
 *    @return 8-bit fault flag values from REG0x22_FAULT_Flag_0
 *    @note   Reading this function clears all fault flag bits automatically
 */
uint8_t Adafruit_BQ25628E::getFaultFlags() {
  // Read REG0x22 (FAULT Flag 0) - clears flags on read
  Adafruit_BusIO_Register fault_flag_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_FAULT_FLAG_0, 1);
  
  return fault_flag_reg.read();
}

/*!
 *    @brief  Sets interrupt mask for all interrupt sources
 *    @param  mask
 *            32-bit mask value (1 = disable interrupt, 0 = enable interrupt)
 *    @return True if successful, otherwise false.
 *    @note   Use BQ25628E_INT_MASK_* defines to construct the mask
 */
bool Adafruit_BQ25628E::setInterruptMask(uint32_t mask) {
  // Extract individual register values from 32-bit mask
  // Mask0 (REG0x23): bits 6:0 (ADC_DONE, TREG, VSYS, IINDPM, VINDPM, SAFETY_TMR, WD)
  uint8_t mask0 = (mask >> 0) & 0x7F;
  
  // Mask1 (REG0x24): bits 3,0 (CHG=bit11, VBUS=bit8)
  uint8_t mask1 = 0;
  if (mask & BQ25628E_INT_MASK_VBUS) mask1 |= BQ25628E_MASK1_VBUS_MASK;
  if (mask & BQ25628E_INT_MASK_CHG) mask1 |= BQ25628E_MASK1_CHG_MASK;
  
  // FMask (REG0x25): bits 7,6,5,3,0 (VBUS_FAULT, BAT_FAULT, SYS_FAULT, TSHUT, TS)
  uint8_t fmask = 0;
  if (mask & BQ25628E_INT_MASK_TS) fmask |= BQ25628E_FMASK_TS_MASK;
  if (mask & BQ25628E_INT_MASK_TSHUT) fmask |= BQ25628E_FMASK_TSHUT_MASK;
  if (mask & BQ25628E_INT_MASK_SYS_FAULT) fmask |= BQ25628E_FMASK_SYS_FAULT_MASK;
  if (mask & BQ25628E_INT_MASK_BAT_FAULT) fmask |= BQ25628E_FMASK_BAT_FAULT_MASK;
  if (mask & BQ25628E_INT_MASK_VBUS_FAULT) fmask |= BQ25628E_FMASK_VBUS_FAULT_MASK;
  
  // Write to all three mask registers
  Adafruit_BusIO_Register mask0_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_MASK_0, 1);
  Adafruit_BusIO_Register mask1_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_MASK_1, 1);
  Adafruit_BusIO_Register fmask_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_FAULT_MASK_0, 1);
  
  return mask0_reg.write(mask0) && mask1_reg.write(mask1) && fmask_reg.write(fmask);
}

/*!
 *    @brief  Gets interrupt mask for all interrupt sources
 *    @return 32-bit mask value (1 = interrupt disabled, 0 = interrupt enabled)
 */
uint32_t Adafruit_BQ25628E::getInterruptMask() {
  // Read all three mask registers
  Adafruit_BusIO_Register mask0_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_MASK_0, 1);
  Adafruit_BusIO_Register mask1_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_CHARGER_MASK_1, 1);
  Adafruit_BusIO_Register fmask_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_FAULT_MASK_0, 1);
  
  uint8_t mask0 = mask0_reg.read();
  uint8_t mask1 = mask1_reg.read();
  uint8_t fmask = fmask_reg.read();
  
  // Combine into 32-bit mask
  uint32_t combined_mask = 0;
  
  // Mask0 (REG0x23): bits 6:0 map directly to bits 6:0
  combined_mask |= (mask0 & 0x7F);
  
  // Mask1 (REG0x24): bits 3,0 map to specific bit positions
  if (mask1 & BQ25628E_MASK1_VBUS_MASK) combined_mask |= BQ25628E_INT_MASK_VBUS;
  if (mask1 & BQ25628E_MASK1_CHG_MASK) combined_mask |= BQ25628E_INT_MASK_CHG;
  
  // FMask (REG0x25): bits 7,6,5,3,0 map to specific bit positions  
  if (fmask & BQ25628E_FMASK_TS_MASK) combined_mask |= BQ25628E_INT_MASK_TS;
  if (fmask & BQ25628E_FMASK_TSHUT_MASK) combined_mask |= BQ25628E_INT_MASK_TSHUT;
  if (fmask & BQ25628E_FMASK_SYS_FAULT_MASK) combined_mask |= BQ25628E_INT_MASK_SYS_FAULT;
  if (fmask & BQ25628E_FMASK_BAT_FAULT_MASK) combined_mask |= BQ25628E_INT_MASK_BAT_FAULT;
  if (fmask & BQ25628E_FMASK_VBUS_FAULT_MASK) combined_mask |= BQ25628E_INT_MASK_VBUS_FAULT;
  
  return combined_mask;
}

/*!
 *    @brief  Enables or disables ADC conversion
 *    @param  enable
 *            True to enable ADC, false to disable
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setADCEnable(bool enable) {
  Adafruit_BusIO_Register adc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_ADC_CONTROL, 1);
  Adafruit_BusIO_RegisterBits adc_enable = Adafruit_BusIO_RegisterBits(&adc_control_reg, 1, 7);
  
  return adc_enable.write(enable);
}

/*!
 *    @brief  Gets ADC enable status
 *    @return True if ADC is enabled, false otherwise
 */
bool Adafruit_BQ25628E::getADCEnable() {
  Adafruit_BusIO_Register adc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_ADC_CONTROL, 1);
  Adafruit_BusIO_RegisterBits adc_enable = Adafruit_BusIO_RegisterBits(&adc_control_reg, 1, 7);
  
  return adc_enable.read();
}

/*!
 *    @brief  Sets ADC conversion mode
 *    @param  one_shot
 *            True for one-shot conversion, false for continuous conversion
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setADCOneShot(bool one_shot) {
  Adafruit_BusIO_Register adc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_ADC_CONTROL, 1);
  Adafruit_BusIO_RegisterBits adc_rate = Adafruit_BusIO_RegisterBits(&adc_control_reg, 1, 6);
  
  return adc_rate.write(one_shot);
}

/*!
 *    @brief  Gets ADC conversion mode
 *    @return True if one-shot mode, false if continuous mode
 */
bool Adafruit_BQ25628E::getADCOneShot() {
  Adafruit_BusIO_Register adc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_ADC_CONTROL, 1);
  Adafruit_BusIO_RegisterBits adc_rate = Adafruit_BusIO_RegisterBits(&adc_control_reg, 1, 6);
  
  return adc_rate.read();
}

/*!
 *    @brief  Sets ADC sample rate (bit resolution)
 *    @param  sample_rate
 *            Sample rate setting (see bq25628e_adc_sample_t)
 *    @return True if successful, otherwise false.
 */
bool Adafruit_BQ25628E::setADCSampleRate(bq25628e_adc_sample_t sample_rate) {
  Adafruit_BusIO_Register adc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_ADC_CONTROL, 1);
  Adafruit_BusIO_RegisterBits adc_sample = Adafruit_BusIO_RegisterBits(&adc_control_reg, 2, 4);
  
  return adc_sample.write(sample_rate);
}

/*!
 *    @brief  Gets ADC sample rate setting
 *    @return Current sample rate setting (see bq25628e_adc_sample_t)
 */
bq25628e_adc_sample_t Adafruit_BQ25628E::getADCSampleRate() {
  Adafruit_BusIO_Register adc_control_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_ADC_CONTROL, 1);
  Adafruit_BusIO_RegisterBits adc_sample = Adafruit_BusIO_RegisterBits(&adc_control_reg, 2, 4);
  
  return (bq25628e_adc_sample_t)adc_sample.read();
}

/*!
 *    @brief  Sets ADC function disable flags
 *    @param  disable_flags
 *            Bitfield of ADC functions to disable (use BQ25628E_ADC_DIS_* flags)
 *            Setting a bit to 1 disables that ADC function
 *            Setting a bit to 0 enables that ADC function
 *    @return True if successful, otherwise false.
 *    @note   Use BQ25628E_ADC_DIS_* defines to construct the disable_flags
 */
bool Adafruit_BQ25628E::setDisableADC(uint8_t disable_flags) {
  Adafruit_BusIO_Register adc_func_disable_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_ADC_FUNCTION_DISABLE_0, 1);
  
  return adc_func_disable_reg.write(disable_flags);
}

/*!
 *    @brief  Gets ADC function disable flags
 *    @return Bitfield of disabled ADC functions (1 = disabled, 0 = enabled)
 *    @note   Use BQ25628E_ADC_DIS_* defines to check individual flags
 */
uint8_t Adafruit_BQ25628E::getDisableADC() {
  Adafruit_BusIO_Register adc_func_disable_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_ADC_FUNCTION_DISABLE_0, 1);
  
  return adc_func_disable_reg.read();
}

/*!
 *    @brief  Gets IBUS current measurement from ADC
 *    @return Current in Amperes (positive = from VBUS to PMID, negative = reverse)
 *    @note   Requires ADC to be enabled. Returns 2's complement 15-bit value
 *            converted to float with 2mA resolution
 */
float Adafruit_BQ25628E::getIBUScurrent() {
  // Read 16-bit IBUS ADC register (little endian)
  Adafruit_BusIO_Register ibus_adc_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_IBUS_ADC, 2);
  
  uint16_t raw_value = ibus_adc_reg.read();
  
  // Extract 15-bit ADC value from bits 15:1 (shift right by 1)
  uint16_t adc_15bit = raw_value >> 1;
  
  // Convert from 15-bit 2's complement to signed 16-bit
  int16_t signed_value;
  if (adc_15bit & 0x4000) {
    // Negative value - extend sign bit
    signed_value = (int16_t)(adc_15bit | 0x8000);
  } else {
    // Positive value
    signed_value = (int16_t)adc_15bit;
  }
  
  // Convert to Amperes: 2mA per step
  return (float)signed_value * 0.002f;
}

/*!
 *    @brief  Gets IBAT current measurement from ADC
 *    @return Current in Amperes (positive = charging, negative = discharging)
 *    @note   Requires ADC to be enabled. Returns 2's complement 14-bit value
 *            converted to float with 4mA resolution. Range: -7.5A to +4.0A
 */
float Adafruit_BQ25628E::getIBATcurrent() {
  // Read 16-bit IBAT ADC register (little endian)
  Adafruit_BusIO_Register ibat_adc_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_IBAT_ADC, 2);
  
  uint16_t raw_value = ibat_adc_reg.read();
  
  // Extract 14-bit ADC value from bits 15:2 (shift right by 2)
  uint16_t adc_14bit = raw_value >> 2;
  
  // Convert from 14-bit 2's complement to signed 16-bit
  int16_t signed_value;
  if (adc_14bit & 0x2000) {
    // Negative value - extend sign bit
    signed_value = (int16_t)(adc_14bit | 0xC000);
  } else {
    // Positive value
    signed_value = (int16_t)adc_14bit;
  }
  
  // Convert to Amperes: 4mA per step
  return (float)signed_value * 0.004f;
}

/*!
 *    @brief  Gets VBUS voltage measurement from ADC
 *    @return Voltage in Volts. Range: 0V to 18V
 *    @note   Requires ADC to be enabled. 3.97mV resolution
 */
float Adafruit_BQ25628E::getVBUSvoltage() {
  // Read 16-bit VBUS ADC register (little endian)
  Adafruit_BusIO_Register vbus_adc_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_VBUS_ADC, 2);
  
  uint16_t raw_value = vbus_adc_reg.read();
  
  // Extract voltage value from bits 15:2 (shift right by 2)
  uint16_t voltage_value = raw_value >> 2;
  
  // Convert to Volts: 3.97mV per step
  return (float)voltage_value * 0.00397f;
}

/*!
 *    @brief  Gets VPMID voltage measurement from ADC
 *    @return Voltage in Volts. Range: 0V to 18V
 *    @note   Requires ADC to be enabled. 3.97mV resolution
 */
float Adafruit_BQ25628E::getVPMIDvoltage() {
  // Read 16-bit VPMID ADC register (little endian)
  Adafruit_BusIO_Register vpmid_adc_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_VPMID_ADC, 2);
  
  uint16_t raw_value = vpmid_adc_reg.read();
  
  // Extract voltage value from bits 15:2 (shift right by 2)
  uint16_t voltage_value = raw_value >> 2;
  
  // Convert to Volts: 3.97mV per step
  return (float)voltage_value * 0.00397f;
}

/*!
 *    @brief  Gets VBAT voltage measurement from ADC
 *    @return Voltage in Volts. Range: 0V to 5.572V
 *    @note   Requires ADC to be enabled. 1.99mV resolution
 */
float Adafruit_BQ25628E::getVBATvoltage() {
  // Read 16-bit VBAT ADC register (little endian)
  Adafruit_BusIO_Register vbat_adc_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_VBAT_ADC, 2);
  
  uint16_t raw_value = vbat_adc_reg.read();
  
  // Extract voltage value from bits 15:1 (shift right by 1)
  uint16_t voltage_value = raw_value >> 1;
  
  // Convert to Volts: 1.99mV per step
  return (float)voltage_value * 0.00199f;
}

/*!
 *    @brief  Gets VSYS voltage measurement from ADC
 *    @return Voltage in Volts. Range: 0V to 5.572V
 *    @note   Requires ADC to be enabled. 1.99mV resolution
 */
float Adafruit_BQ25628E::getVSYSvoltage() {
  // Read 16-bit VSYS ADC register (little endian)
  Adafruit_BusIO_Register vsys_adc_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_VSYS_ADC, 2);
  
  uint16_t raw_value = vsys_adc_reg.read();
  
  // Extract voltage value from bits 15:1 (shift right by 1)
  uint16_t voltage_value = raw_value >> 1;
  
  // Convert to Volts: 1.99mV per step
  return (float)voltage_value * 0.00199f;
}

/*!
 *    @brief  Gets thermistor reading as percentage of bias reference
 *    @return Percentage (0-100%). Range: 0% to 98.31%
 *    @note   Requires ADC to be enabled and TS pin bias reference active
 *            Uses bits 11:0 with 0.0961% resolution
 */
float Adafruit_BQ25628E::getThermistorPercent() {
  // Read 16-bit TS ADC register (little endian)
  Adafruit_BusIO_Register ts_adc_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_TS_ADC, 2);
  
  uint16_t raw_value = ts_adc_reg.read();
  
  // Extract 12-bit value from bits 11:0 (no shift needed)
  uint16_t ts_value = raw_value & 0x0FFF;
  
  // Convert to percentage: 0.0961% per step
  return (float)ts_value * 0.0961f;
}

/*!
 *    @brief  Gets die temperature measurement from ADC
 *    @return Temperature in Celsius. Range: -40°C to +140°C
 *    @note   Requires ADC to be enabled. Returns 2's complement 12-bit value
 *            with 0.5°C resolution using bits 11:0
 */
float Adafruit_BQ25628E::getDieTempC() {
  // Read 16-bit TDIE ADC register (little endian)
  Adafruit_BusIO_Register tdie_adc_reg = Adafruit_BusIO_Register(i2c_dev, BQ25628E_REG_TDIE_ADC, 2);
  
  uint16_t raw_value = tdie_adc_reg.read();
  
  // Extract 12-bit value from bits 11:0 (no shift needed)
  uint16_t temp_12bit = raw_value & 0x0FFF;
  
  // Convert from 12-bit 2's complement to signed 16-bit
  int16_t signed_value;
  if (temp_12bit & 0x0800) {
    // Negative value - extend sign bit
    signed_value = (int16_t)(temp_12bit | 0xF000);
  } else {
    // Positive value
    signed_value = (int16_t)temp_12bit;
  }
  
  // Convert to Celsius: 0.5°C per step
  return (float)signed_value * 0.5f;
}