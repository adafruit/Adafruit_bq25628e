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

/*!
 * @brief Watchdog timer settings
 */
typedef enum {
  BQ25628E_WATCHDOG_DISABLED = 0b00, /*!< Watchdog disabled */
  BQ25628E_WATCHDOG_50S = 0b01,      /*!< 50 second watchdog */
  BQ25628E_WATCHDOG_100S = 0b10,     /*!< 100 second watchdog */
  BQ25628E_WATCHDOG_200S = 0b11      /*!< 200 second watchdog */
} bq25628e_watchdog_t;

/*!
 * @brief Converter switching frequency settings
 */
typedef enum {
  BQ25628E_CONV_FREQ_1500KHZ = 0b00, /*!< 1.5 MHz nominal */
  BQ25628E_CONV_FREQ_1350KHZ = 0b01, /*!< 1.35 MHz (-10%) */
  BQ25628E_CONV_FREQ_1650KHZ = 0b10, /*!< 1.65 MHz (+10%) */
  BQ25628E_CONV_FREQ_RESERVED = 0b11 /*!< Reserved */
} bq25628e_conv_freq_t;

/*!
 * @brief BATFET control settings
 */
typedef enum {
  BQ25628E_BATFET_NORMAL = 0b00,    /*!< Normal operation */
  BQ25628E_BATFET_SHUTDOWN = 0b01,  /*!< Shutdown mode */
  BQ25628E_BATFET_SHIP = 0b10,      /*!< Ship mode */
  BQ25628E_BATFET_RESET = 0b11      /*!< System power reset */
} bq25628e_batfet_ctrl_t;

/*!
 * @brief Charge rate settings
 */
typedef enum {
  BQ25628E_CHARGE_RATE_1C = 0b00, /*!< 1C charge rate */
  BQ25628E_CHARGE_RATE_2C = 0b01, /*!< 2C charge rate */
  BQ25628E_CHARGE_RATE_4C = 0b10, /*!< 4C charge rate */
  BQ25628E_CHARGE_RATE_6C = 0b11  /*!< 6C charge rate */
} bq25628e_charge_rate_t;

/*!
 * @brief Thermistor current settings for cool/warm zones
 */
typedef enum {
  BQ25628E_THERM_CURR_SUSPEND = 0b00, /*!< Charge suspended */
  BQ25628E_THERM_CURR_20PCT = 0b01,   /*!< Set ICHG to 20% */
  BQ25628E_THERM_CURR_40PCT = 0b10,   /*!< Set ICHG to 40% */
  BQ25628E_THERM_CURR_UNCHANGED = 0b11 /*!< ICHG unchanged */
} bq25628e_therm_curr_t;

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

  bool setChargeCurrentLimitA(float current_a);
  float getChargeCurrentLimitA();

  bool setChargeVoltageLimitV(float voltage_v);
  float getChargeVoltageLimitV();

  bool setInputCurrentLimitA(float current_a);
  float getInputCurrentLimitA();

  bool setInputVoltageLimitV(float voltage_v);
  float getInputVoltageLimitV();

  bool setMinimalSystemVoltageV(float voltage_v);
  float getMinimalSystemVoltageV();

  bool setPrechargeCurrentLimitA(float current_a);
  float getPrechargeCurrentLimitA();

  bool setTerminationCurrentThresholdA(float current_a);
  float getTerminationCurrentThresholdA();

  bool setTrickleCurrent(bool use_40ma);
  bool getTrickleCurrent();

  bool setEnableTermination(bool enable);
  bool getEnableTermination();

  bool setVINDPMbatTrack(bool enable);
  bool getVINDPMbatTrack();

  bool setEnableSafetyTimers(bool enable);
  bool getEnableSafetyTimers();

  bool setPrechargeTimer(bool short_timer);
  bool getPrechargeTimer();

  bool setFastchargeTimer(bool long_timer);
  bool getFastchargeTimer();

  bool setAutoBatteryDischarge(bool enable);
  bool getAutoBatteryDischarge();

  bool setForceBatteryDischarge(bool enable);
  bool getForceBatteryDischarge();

  bool setEnableCharging(bool enable);
  bool getEnableCharging();

  bool setHighZ(bool enable);
  bool getHighZ();

  bool setForcePMIDDischarge(bool enable);
  bool getForcePMIDDischarge();

  bool resetWatchdog();

  bool setWatchdog(bq25628e_watchdog_t setting);
  bq25628e_watchdog_t getWatchdog();

  bool reset();

  bool setThermalRegulation(bool temp_120c);
  bool getThermalRegulation();

  bool setConverterFrequency(bq25628e_conv_freq_t frequency);
  bq25628e_conv_freq_t getConverterFrequency();

  bool setVBUSOvervoltage(bool high_threshold);
  bool getVBUSOvervoltage();

  bool setBATFETcontrol(bq25628e_batfet_ctrl_t control);
  bq25628e_batfet_ctrl_t getBATFETcontrol();

  bool setPeakBattDischarge(bool peak_12a);
  bool getPeakBattDischarge();

  bool setVBatUVLO(bool low_threshold);
  bool getVBatUVLO();

  bool setChargeRate(bq25628e_charge_rate_t rate);
  bq25628e_charge_rate_t getChargeRate();

  bool setIgnoreThermistor(bool ignore);
  bool getIgnoreThermistor();

  bool setCoolThermistorCurrent(bq25628e_therm_curr_t setting);
  bq25628e_therm_curr_t getCoolThermistorCurrent();

  bool setWarmThermistorCurrent(bq25628e_therm_curr_t setting);
  bq25628e_therm_curr_t getWarmThermistorCurrent();

private:
  Adafruit_I2CDevice *i2c_dev; /*!< Pointer to I2C bus interface */
};

#endif