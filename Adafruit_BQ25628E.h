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

/*! Status register bit flags for REG0x1D_Charger_Status_0 */
#define BQ25628E_STATUS0_WD_STAT        (1 << 0)  /*!< WD timer expired */
#define BQ25628E_STATUS0_SAFETY_TMR_STAT (1 << 1) /*!< Safety timer expired */
#define BQ25628E_STATUS0_VINDPM_STAT    (1 << 2)  /*!< In VINDPM regulation */
#define BQ25628E_STATUS0_IINDPM_STAT    (1 << 3)  /*!< In IINDPM/ILIM regulation */
#define BQ25628E_STATUS0_VSYS_STAT      (1 << 4)  /*!< In VSYSMIN regulation */
#define BQ25628E_STATUS0_TREG_STAT      (1 << 5)  /*!< In thermal regulation */
#define BQ25628E_STATUS0_ADC_DONE_STAT  (1 << 6)  /*!< ADC conversion complete */

/*! Status register bit masks for REG0x1E_Charger_Status_1 */
#define BQ25628E_STATUS1_VBUS_STAT_MASK  (0x07)    /*!< VBUS status mask bits 2:0 */
#define BQ25628E_STATUS1_CHG_STAT_MASK   (0x18)    /*!< Charge status mask bits 4:3 */
#define BQ25628E_STATUS1_CHG_STAT_SHIFT  (3)       /*!< Charge status bit shift */

/*! VBUS Status values */
#define BQ25628E_VBUS_STAT_NOT_POWERED   (0x00)    /*!< Not powered from VBUS */
#define BQ25628E_VBUS_STAT_UNKNOWN_ADAPTER (0x04)  /*!< Unknown adapter */

/*! Charge Status values */
#define BQ25628E_CHG_STAT_NOT_CHARGING   (0x00)    /*!< Not charging or terminated */
#define BQ25628E_CHG_STAT_CHARGING       (0x01)    /*!< Trickle/Pre/Fast charge */
#define BQ25628E_CHG_STAT_TAPER          (0x02)    /*!< Taper charge (CV mode) */
#define BQ25628E_CHG_STAT_TOPOFF         (0x03)    /*!< Top-off timer active */

/*! Fault status register bit flags for REG0x1F_FAULT_Status_0 */
#define BQ25628E_FAULT_VBUS_FAULT_STAT   (1 << 7)  /*!< VBUS fault (OVP/sleep) */
#define BQ25628E_FAULT_BAT_FAULT_STAT    (1 << 6)  /*!< Battery fault (OCP/OVP) */
#define BQ25628E_FAULT_SYS_FAULT_STAT    (1 << 5)  /*!< System fault (short/OVP) */
#define BQ25628E_FAULT_TSHUT_STAT        (1 << 3)  /*!< Thermal shutdown */
#define BQ25628E_FAULT_TS_STAT_MASK      (0x07)    /*!< TS status mask bits 2:0 */

/*! TS Status values */
#define BQ25628E_TS_STAT_NORMAL          (0x00)    /*!< TS Normal */
#define BQ25628E_TS_STAT_COLD            (0x01)    /*!< TS Cold */
#define BQ25628E_TS_STAT_HOT             (0x02)    /*!< TS Hot */
#define BQ25628E_TS_STAT_COOL            (0x03)    /*!< TS Cool */
#define BQ25628E_TS_STAT_WARM            (0x04)    /*!< TS Warm */
#define BQ25628E_TS_STAT_PRECOOL         (0x05)    /*!< TS Pre-cool */
#define BQ25628E_TS_STAT_PREWARM         (0x06)    /*!< TS Pre-warm */
#define BQ25628E_TS_STAT_BIAS_FAULT      (0x07)    /*!< TS bias reference fault */

/*! Charger flag register bit flags for REG0x20_Charger_Flag_0 */
#define BQ25628E_FLAG0_WD_FLAG           (1 << 0)  /*!< WD timer expired flag */
#define BQ25628E_FLAG0_SAFETY_TMR_FLAG   (1 << 1)  /*!< Safety timer expired flag */
#define BQ25628E_FLAG0_VINDPM_FLAG       (1 << 2)  /*!< VINDPM regulation flag */
#define BQ25628E_FLAG0_IINDPM_FLAG       (1 << 3)  /*!< IINDPM/ILIM regulation flag */
#define BQ25628E_FLAG0_VSYS_FLAG         (1 << 4)  /*!< VSYSMIN regulation flag */
#define BQ25628E_FLAG0_TREG_FLAG         (1 << 5)  /*!< Thermal regulation flag */
#define BQ25628E_FLAG0_ADC_DONE_FLAG     (1 << 6)  /*!< ADC conversion complete flag */

/*! Charger flag register bit flags for REG0x21_Charger_Flag_1 */
#define BQ25628E_FLAG1_VBUS_FLAG         (1 << 0)  /*!< VBUS status changed flag */
#define BQ25628E_FLAG1_CHG_FLAG          (1 << 3)  /*!< Charge status changed flag */

/*! Fault flag register bit flags for REG0x22_FAULT_Flag_0 */
#define BQ25628E_FAULT_FLAG_VBUS_FAULT   (1 << 7)  /*!< VBUS OVP/sleep fault flag */
#define BQ25628E_FAULT_FLAG_BAT_FAULT    (1 << 6)  /*!< Battery OCP/OVP fault flag */
#define BQ25628E_FAULT_FLAG_SYS_FAULT    (1 << 5)  /*!< System OVP/short fault flag */
#define BQ25628E_FAULT_FLAG_TSHUT        (1 << 3)  /*!< Thermal shutdown fault flag */
#define BQ25628E_FAULT_FLAG_TS_CHANGED   (1 << 0)  /*!< TS status changed flag */

/*! Interrupt mask register bit flags for REG0x23_Charger_Mask_0 */
#define BQ25628E_MASK0_ADC_DONE_MASK     (1 << 6)  /*!< ADC conversion mask */
#define BQ25628E_MASK0_TREG_MASK         (1 << 5)  /*!< Thermal regulation mask */
#define BQ25628E_MASK0_VSYS_MASK         (1 << 4)  /*!< VSYSMIN regulation mask */
#define BQ25628E_MASK0_IINDPM_MASK       (1 << 3)  /*!< IINDPM/ILIM regulation mask */
#define BQ25628E_MASK0_VINDPM_MASK       (1 << 2)  /*!< VINDPM regulation mask */
#define BQ25628E_MASK0_SAFETY_TMR_MASK   (1 << 1)  /*!< Safety timer mask */
#define BQ25628E_MASK0_WD_MASK           (1 << 0)  /*!< Watchdog timer mask */

/*! Interrupt mask register bit flags for REG0x24_Charger_Mask_1 */
#define BQ25628E_MASK1_CHG_MASK          (1 << 3)  /*!< Charge status change mask */
#define BQ25628E_MASK1_VBUS_MASK         (1 << 0)  /*!< VBUS status change mask */

/*! Interrupt mask register bit flags for REG0x25_FAULT_Mask_0 */
#define BQ25628E_FMASK_VBUS_FAULT_MASK   (1 << 7)  /*!< VBUS fault mask */
#define BQ25628E_FMASK_BAT_FAULT_MASK    (1 << 6)  /*!< Battery fault mask */
#define BQ25628E_FMASK_SYS_FAULT_MASK    (1 << 5)  /*!< System fault mask */
#define BQ25628E_FMASK_TSHUT_MASK        (1 << 3)  /*!< Thermal shutdown mask */
#define BQ25628E_FMASK_TS_MASK           (1 << 0)  /*!< TS status change mask */

/*! Combined interrupt mask positions for 32-bit interface */
#define BQ25628E_INT_MASK_WD             (1UL << 0)   /*!< Watchdog timer interrupt */
#define BQ25628E_INT_MASK_SAFETY_TMR     (1UL << 1)   /*!< Safety timer interrupt */
#define BQ25628E_INT_MASK_VINDPM         (1UL << 2)   /*!< VINDPM regulation interrupt */
#define BQ25628E_INT_MASK_IINDPM         (1UL << 3)   /*!< IINDPM/ILIM regulation interrupt */
#define BQ25628E_INT_MASK_VSYS           (1UL << 4)   /*!< VSYSMIN regulation interrupt */
#define BQ25628E_INT_MASK_TREG           (1UL << 5)   /*!< Thermal regulation interrupt */
#define BQ25628E_INT_MASK_ADC_DONE       (1UL << 6)   /*!< ADC conversion interrupt */
#define BQ25628E_INT_MASK_VBUS           (1UL << 8)   /*!< VBUS status change interrupt */
#define BQ25628E_INT_MASK_CHG            (1UL << 11)  /*!< Charge status change interrupt */
#define BQ25628E_INT_MASK_TS             (1UL << 16)  /*!< TS status change interrupt */
#define BQ25628E_INT_MASK_TSHUT          (1UL << 19)  /*!< Thermal shutdown interrupt */
#define BQ25628E_INT_MASK_SYS_FAULT      (1UL << 21)  /*!< System fault interrupt */
#define BQ25628E_INT_MASK_BAT_FAULT      (1UL << 22)  /*!< Battery fault interrupt */
#define BQ25628E_INT_MASK_VBUS_FAULT     (1UL << 23)  /*!< VBUS fault interrupt */

/*! Default interrupt mask: Enable only CHG and VBUS interrupts, disable all others */
#define BQ25628E_INT_MASK_DEFAULT        (~(BQ25628E_INT_MASK_CHG | BQ25628E_INT_MASK_VBUS))

/*!
 * @brief ADC sample rate settings
 */
typedef enum {
  BQ25628E_ADC_SAMPLE_12BIT = 0b00, /*!< 12-bit effective resolution */
  BQ25628E_ADC_SAMPLE_11BIT = 0b01, /*!< 11-bit effective resolution */
  BQ25628E_ADC_SAMPLE_10BIT = 0b10, /*!< 10-bit effective resolution */
  BQ25628E_ADC_SAMPLE_9BIT = 0b11   /*!< 9-bit effective resolution */
} bq25628e_adc_sample_t;

/*! ADC function disable flags for REG0x27_ADC_Function_Disable_0 */
#define BQ25628E_ADC_DIS_IBUS        (1 << 7)  /*!< Disable IBUS ADC */
#define BQ25628E_ADC_DIS_IBAT        (1 << 6)  /*!< Disable IBAT ADC */
#define BQ25628E_ADC_DIS_VBUS        (1 << 5)  /*!< Disable VBUS ADC */
#define BQ25628E_ADC_DIS_VBAT        (1 << 4)  /*!< Disable VBAT ADC */
#define BQ25628E_ADC_DIS_VSYS        (1 << 3)  /*!< Disable VSYS ADC */
#define BQ25628E_ADC_DIS_TS          (1 << 2)  /*!< Disable TS ADC */
#define BQ25628E_ADC_DIS_TDIE        (1 << 1)  /*!< Disable TDIE ADC */
#define BQ25628E_ADC_DIS_VPMID       (1 << 0)  /*!< Disable VPMID ADC */

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

  uint16_t getChargerStatusFlags();
  uint8_t getFaultStatusFlags();
  uint16_t getChargerFlags();
  uint8_t getFaultFlags();

  bool setInterruptMask(uint32_t mask);
  uint32_t getInterruptMask();

  bool setADCEnable(bool enable);
  bool getADCEnable();

  bool setADCOneShot(bool one_shot);
  bool getADCOneShot();

  bool setADCSampleRate(bq25628e_adc_sample_t sample_rate);
  bq25628e_adc_sample_t getADCSampleRate();

  bool setDisableADC(uint8_t disable_flags);
  uint8_t getDisableADC();

  float getIBUScurrent();
  float getIBATcurrent();
  float getVBUSvoltage();
  float getVPMIDvoltage();
  float getVBATvoltage();
  float getVSYSvoltage();
  float getThermistorPercent();
  float getDieTempC();

private:
  Adafruit_I2CDevice *i2c_dev; /*!< Pointer to I2C bus interface */
};

#endif