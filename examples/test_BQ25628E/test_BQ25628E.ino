/*!
 * Test sketch for the Adafruit BQ25628E I2C Battery Charger library
 * 
 * Designed specifically to work with the Adafruit BQ25628E Breakout
 * Pick one up today in the adafruit shop!
 * 
 * Author: Limor 'ladyada' Fried with assistance from Claude Code
 * License: MIT
 */

#include "Adafruit_BQ25628E.h"

Adafruit_BQ25628E bq;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println(F("Adafruit BQ25628E Test!"));
  
  if (!bq.begin()) {
    Serial.println(F("Failed to find BQ25628E chip"));
    while (1) delay(10);
  }
  
  Serial.println(F("BQ25628E Found!"));
  
  // Reset chip to default values
  if (bq.reset()) {
    Serial.println(F("Reset successful"));
  } else {
    Serial.println(F("Reset failed"));
  }
  
  // Uncomment to set charge current limit to 1.0A
  // bq.setChargeCurrentLimitA(1.0);
  
  // Test charge current limit functions
  float current = bq.getChargeCurrentLimitA();
  Serial.print(F("Current charge limit: "));
  Serial.print(current);
  Serial.println(F(" A"));
  
  // Uncomment to set charge voltage limit to 4.1V
  // bq.setChargeVoltageLimitV(4.1);
  
  // Test charge voltage limit functions
  float voltage = bq.getChargeVoltageLimitV();
  Serial.print(F("Current voltage limit: "));
  Serial.print(voltage);
  Serial.println(F(" V"));
  
  // Uncomment to set input current limit to 2.0A
  // bq.setInputCurrentLimitA(2.0);
  
  // Test input current limit functions
  float input_current = bq.getInputCurrentLimitA();
  Serial.print(F("Current input limit: "));
  Serial.print(input_current);
  Serial.println(F(" A"));
  
  // Uncomment to set input voltage limit to 5.0V
  // bq.setInputVoltageLimitV(5.0);
  
  // Test input voltage limit functions
  float input_voltage = bq.getInputVoltageLimitV();
  Serial.print(F("Current input voltage limit: "));
  Serial.print(input_voltage);
  Serial.println(F(" V"));
  
  // Uncomment to set minimal system voltage to 3.0V
  // bq.setMinimalSystemVoltageV(3.0);
  
  // Test minimal system voltage functions
  float min_sys_voltage = bq.getMinimalSystemVoltageV();
  Serial.print(F("Current minimal system voltage: "));
  Serial.print(min_sys_voltage);
  Serial.println(F(" V"));
  
  // Uncomment to set precharge current limit to 0.05A
  // bq.setPrechargeCurrentLimitA(0.05);
  
  // Test precharge current limit functions
  float precharge_current = bq.getPrechargeCurrentLimitA();
  Serial.print(F("Current precharge limit: "));
  Serial.print(precharge_current);
  Serial.println(F(" A"));
  
  // Uncomment to set termination current threshold to 0.025A
  // bq.setTerminationCurrentThresholdA(0.025);
  
  // Test termination current threshold functions
  float termination_current = bq.getTerminationCurrentThresholdA();
  Serial.print(F("Current termination threshold: "));
  Serial.print(termination_current);
  Serial.println(F(" A"));
  
  // Uncomment to set trickle current to 40mA
  // bq.setTrickleCurrent(true);
  
  // Test trickle current functions
  bool trickle_40ma = bq.getTrickleCurrent();
  Serial.print(F("Trickle current (40mA mode): "));
  Serial.println(trickle_40ma ? F("true") : F("false"));
  
  // Uncomment to disable termination
  // bq.setEnableTermination(false);
  
  // Test termination enable functions
  bool term_enabled = bq.getEnableTermination();
  Serial.print(F("Termination enabled: "));
  Serial.println(term_enabled ? F("true") : F("false"));
  
  // Uncomment to disable VINDPM battery tracking
  // bq.setVINDPMbatTrack(false);
  
  // Test VINDPM battery tracking functions
  bool vindpm_track = bq.getVINDPMbatTrack();
  Serial.print(F("VINDPM battery tracking: "));
  Serial.println(vindpm_track ? F("true") : F("false"));
  
  // Uncomment to disable safety timers
  // bq.setEnableSafetyTimers(false);
  
  // Test safety timer enable functions
  bool safety_timers = bq.getEnableSafetyTimers();
  Serial.print(F("Safety timers enabled: "));
  Serial.println(safety_timers ? F("true") : F("false"));
  
  // Uncomment to set short precharge timer (0.62 hours)
  // bq.setPrechargeTimer(true);
  
  // Test precharge timer functions
  bool precharge_short = bq.getPrechargeTimer();
  Serial.print(F("Precharge timer (0.62h mode): "));
  Serial.println(precharge_short ? F("true") : F("false"));
  
  // Uncomment to set long fast charge timer (28 hours)
  // bq.setFastchargeTimer(true);
  
  // Test fast charge timer functions
  bool fastcharge_long = bq.getFastchargeTimer();
  Serial.print(F("Fast charge timer (28h mode): "));
  Serial.println(fastcharge_long ? F("true") : F("false"));
  
  // Uncomment to disable auto battery discharge
  // bq.setAutoBatteryDischarge(false);
  
  // Test auto battery discharge functions
  bool auto_bat_discharge = bq.getAutoBatteryDischarge();
  Serial.print(F("Auto battery discharge: "));
  Serial.println(auto_bat_discharge ? F("true") : F("false"));
  
  // Uncomment to force battery discharge
  // bq.setForceBatteryDischarge(true);
  
  // Test force battery discharge functions
  bool force_bat_discharge = bq.getForceBatteryDischarge();
  Serial.print(F("Force battery discharge: "));
  Serial.println(force_bat_discharge ? F("true") : F("false"));
  
  // Uncomment to disable charging
  // bq.setEnableCharging(false);
  
  // Test charging enable functions
  bool charging_enabled = bq.getEnableCharging();
  Serial.print(F("Charging enabled: "));
  Serial.println(charging_enabled ? F("true") : F("false"));
  
  // Uncomment to enable HIZ mode
  // bq.setHighZ(true);
  
  // Test HIZ mode functions
  bool hiz_enabled = bq.getHighZ();
  Serial.print(F("HIZ mode: "));
  Serial.println(hiz_enabled ? F("true") : F("false"));
  
  // Uncomment to force PMID discharge
  // bq.setForcePMIDDischarge(true);
  
  // Test force PMID discharge functions
  bool force_pmid_discharge = bq.getForcePMIDDischarge();
  Serial.print(F("Force PMID discharge: "));
  Serial.println(force_pmid_discharge ? F("true") : F("false"));
  
  // Uncomment to reset watchdog
  // bq.resetWatchdog();
  
  // Uncomment to set watchdog to 100s
  // bq.setWatchdog(BQ25628E_WATCHDOG_100S);
  
  // Test watchdog setting
  bq25628e_watchdog_t watchdog_setting = bq.getWatchdog();
  Serial.print(F("Watchdog setting: "));
  switch (watchdog_setting) {
    case BQ25628E_WATCHDOG_DISABLED:
      Serial.println(F("Disabled"));
      break;
    case BQ25628E_WATCHDOG_50S:
      Serial.println(F("50s"));
      break;
    case BQ25628E_WATCHDOG_100S:
      Serial.println(F("100s"));
      break;
    case BQ25628E_WATCHDOG_200S:
      Serial.println(F("200s"));
      break;
  }
  
  // Uncomment to set thermal regulation to 60°C
  // bq.setThermalRegulation(false);
  
  // Test thermal regulation setting
  bool thermal_120c = bq.getThermalRegulation();
  Serial.print(F("Thermal regulation: "));
  Serial.println(thermal_120c ? F("120°C") : F("60°C"));
  
  // Uncomment to set converter frequency to 1.35MHz
  // bq.setConverterFrequency(BQ25628E_CONV_FREQ_1350KHZ);
  
  // Test converter frequency setting
  bq25628e_conv_freq_t conv_freq = bq.getConverterFrequency();
  Serial.print(F("Converter frequency: "));
  switch (conv_freq) {
    case BQ25628E_CONV_FREQ_1500KHZ:
      Serial.println(F("1.5MHz"));
      break;
    case BQ25628E_CONV_FREQ_1350KHZ:
      Serial.println(F("1.35MHz"));
      break;
    case BQ25628E_CONV_FREQ_1650KHZ:
      Serial.println(F("1.65MHz"));
      break;
    case BQ25628E_CONV_FREQ_RESERVED:
      Serial.println(F("Reserved"));
      break;
  }
  
  // Uncomment to set VBUS OVP to 6.3V
  // bq.setVBUSOvervoltage(false);
  
  // Test VBUS overvoltage setting
  bool vbus_high_ovp = bq.getVBUSOvervoltage();
  Serial.print(F("VBUS OVP threshold: "));
  Serial.println(vbus_high_ovp ? F("18.5V") : F("6.3V"));
  
  // Uncomment to set BATFET to ship mode (WARNING: will disconnect battery)
  // bq.setBATFETcontrol(BQ25628E_BATFET_SHIP);
  
  // Test BATFET control setting
  bq25628e_batfet_ctrl_t batfet_ctrl = bq.getBATFETcontrol();
  Serial.print(F("BATFET control: "));
  switch (batfet_ctrl) {
    case BQ25628E_BATFET_NORMAL:
      Serial.println(F("Normal"));
      break;
    case BQ25628E_BATFET_SHUTDOWN:
      Serial.println(F("Shutdown"));
      break;
    case BQ25628E_BATFET_SHIP:
      Serial.println(F("Ship"));
      break;
    case BQ25628E_BATFET_RESET:
      Serial.println(F("Reset"));
      break;
  }
  
  // Uncomment to set peak discharge current to 6A
  // bq.setPeakBattDischarge(false);
  
  // Test peak battery discharge setting
  bool peak_12a = bq.getPeakBattDischarge();
  Serial.print(F("Peak discharge current: "));
  Serial.println(peak_12a ? F("12A") : F("6A"));
  
  // Uncomment to set VBAT UVLO to 1.8V threshold
  // bq.setVBatUVLO(true);
  
  // Test VBAT UVLO setting
  bool vbat_uvlo_low = bq.getVBatUVLO();
  Serial.print(F("VBAT UVLO threshold: "));
  Serial.println(vbat_uvlo_low ? F("1.8V") : F("2.2V"));
  
  // Uncomment to set charge rate to 2C
  // bq.setChargeRate(BQ25628E_CHARGE_RATE_2C);
  
  // Test charge rate setting
  bq25628e_charge_rate_t charge_rate = bq.getChargeRate();
  Serial.print(F("Charge rate: "));
  switch (charge_rate) {
    case BQ25628E_CHARGE_RATE_1C:
      Serial.println(F("1C"));
      break;
    case BQ25628E_CHARGE_RATE_2C:
      Serial.println(F("2C"));
      break;
    case BQ25628E_CHARGE_RATE_4C:
      Serial.println(F("4C"));
      break;
    case BQ25628E_CHARGE_RATE_6C:
      Serial.println(F("6C"));
      break;
  }
  
  // Uncomment to ignore thermistor
  // bq.setIgnoreThermistor(true);
  
  // Test thermistor ignore setting
  bool ignore_thermistor = bq.getIgnoreThermistor();
  Serial.print(F("Ignore thermistor: "));
  Serial.println(ignore_thermistor ? F("true") : F("false"));
  
  // Uncomment to set cool thermistor current to 40%
  // bq.setCoolThermistorCurrent(BQ25628E_THERM_CURR_40PCT);
  
  // Test cool thermistor current setting
  bq25628e_therm_curr_t cool_current = bq.getCoolThermistorCurrent();
  Serial.print(F("Cool thermistor current: "));
  switch (cool_current) {
    case BQ25628E_THERM_CURR_SUSPEND:
      Serial.println(F("Suspended"));
      break;
    case BQ25628E_THERM_CURR_20PCT:
      Serial.println(F("20%"));
      break;
    case BQ25628E_THERM_CURR_40PCT:
      Serial.println(F("40%"));
      break;
    case BQ25628E_THERM_CURR_UNCHANGED:
      Serial.println(F("Unchanged"));
      break;
  }
  
  // Uncomment to set warm thermistor current to 20%
  // bq.setWarmThermistorCurrent(BQ25628E_THERM_CURR_20PCT);
  
  // Test warm thermistor current setting
  bq25628e_therm_curr_t warm_current = bq.getWarmThermistorCurrent();
  Serial.print(F("Warm thermistor current: "));
  switch (warm_current) {
    case BQ25628E_THERM_CURR_SUSPEND:
      Serial.println(F("Suspended"));
      break;
    case BQ25628E_THERM_CURR_20PCT:
      Serial.println(F("20%"));
      break;
    case BQ25628E_THERM_CURR_40PCT:
      Serial.println(F("40%"));
      break;
    case BQ25628E_THERM_CURR_UNCHANGED:
      Serial.println(F("Unchanged"));
      break;
  }
  
  // Enable ADC and set to 12-bit mode by default
  Serial.println(F("Enabling ADC with 12-bit resolution..."));
  if (bq.setADCEnable(true) && bq.setADCSampleRate(BQ25628E_ADC_SAMPLE_12BIT)) {
    Serial.println(F("ADC configured successfully"));
  } else {
    Serial.println(F("Failed to configure ADC"));
  }
  
  // Test ADC configuration functions
  bool adc_enabled = bq.getADCEnable();
  Serial.print(F("ADC enabled: "));
  Serial.println(adc_enabled ? F("true") : F("false"));
  
  bool adc_oneshot = bq.getADCOneShot();
  Serial.print(F("ADC one-shot mode: "));
  Serial.println(adc_oneshot ? F("true") : F("false"));
  
  bq25628e_adc_sample_t sample_rate = bq.getADCSampleRate();
  Serial.print(F("ADC sample rate: "));
  switch (sample_rate) {
    case BQ25628E_ADC_SAMPLE_12BIT:
      Serial.println(F("12-bit"));
      break;
    case BQ25628E_ADC_SAMPLE_11BIT:
      Serial.println(F("11-bit"));
      break;
    case BQ25628E_ADC_SAMPLE_10BIT:
      Serial.println(F("10-bit"));
      break;
    case BQ25628E_ADC_SAMPLE_9BIT:
      Serial.println(F("9-bit"));
      break;
  }
  
  // Ensure all ADC functions are enabled (disable_flags = 0x00)
  Serial.println(F("Ensuring all ADC functions are enabled..."));
  if (bq.setDisableADC(0x00)) {
    Serial.println(F("ADC functions configured successfully"));
  } else {
    Serial.println(F("Failed to configure ADC functions"));
  }
  
  // Display ADC function enable status
  uint8_t adc_disable_flags = bq.getDisableADC();
  Serial.print(F("ADC Function Status (0x"));
  Serial.print(adc_disable_flags, HEX);
  Serial.println(F("):"));
  
  Serial.print(F("  IBUS ADC: "));
  Serial.println((adc_disable_flags & BQ25628E_ADC_DIS_IBUS) ? F("Disabled") : F("Enabled"));
  
  Serial.print(F("  IBAT ADC: "));
  Serial.println((adc_disable_flags & BQ25628E_ADC_DIS_IBAT) ? F("Disabled") : F("Enabled"));
  
  Serial.print(F("  VBUS ADC: "));
  Serial.println((adc_disable_flags & BQ25628E_ADC_DIS_VBUS) ? F("Disabled") : F("Enabled"));
  
  Serial.print(F("  VBAT ADC: "));
  Serial.println((adc_disable_flags & BQ25628E_ADC_DIS_VBAT) ? F("Disabled") : F("Enabled"));
  
  Serial.print(F("  VSYS ADC: "));
  Serial.println((adc_disable_flags & BQ25628E_ADC_DIS_VSYS) ? F("Disabled") : F("Enabled"));
  
  Serial.print(F("  TS ADC: "));
  Serial.println((adc_disable_flags & BQ25628E_ADC_DIS_TS) ? F("Disabled") : F("Enabled"));
  
  Serial.print(F("  TDIE ADC: "));
  Serial.println((adc_disable_flags & BQ25628E_ADC_DIS_TDIE) ? F("Disabled") : F("Enabled"));
  
  Serial.print(F("  VPMID ADC: "));
  Serial.println((adc_disable_flags & BQ25628E_ADC_DIS_VPMID) ? F("Disabled") : F("Enabled"));
  
  Serial.println(F("All tests completed!"));
  
  // Set interrupt mask to enable only CHG and VBUS interrupts, disable all others
  Serial.println(F("Setting interrupt mask (CHG + VBUS enabled, others disabled)..."));
  uint32_t intMask = BQ25628E_INT_MASK_WD | BQ25628E_INT_MASK_SAFETY_TMR | 
                     BQ25628E_INT_MASK_VINDPM | BQ25628E_INT_MASK_IINDPM |
                     BQ25628E_INT_MASK_VSYS | BQ25628E_INT_MASK_TREG |
                     BQ25628E_INT_MASK_ADC_DONE | BQ25628E_INT_MASK_TS |
                     BQ25628E_INT_MASK_TSHUT | BQ25628E_INT_MASK_SYS_FAULT |
                     BQ25628E_INT_MASK_BAT_FAULT | BQ25628E_INT_MASK_VBUS_FAULT;
  // CHG and VBUS interrupts NOT included in mask = they remain enabled
  
  if (bq.setInterruptMask(intMask)) {
    Serial.println(F("Interrupt mask set successfully"));
  } else {
    Serial.println(F("Failed to set interrupt mask"));
  }
  
  // Read back and display interrupt mask
  uint32_t readMask = bq.getInterruptMask();
  Serial.print(F("Current interrupt mask: 0x"));
  Serial.println(readMask, HEX);
}

void printChargerStatus() {
  uint16_t statusFlags = bq.getChargerStatusFlags();
  uint8_t faultFlags = bq.getFaultStatusFlags();
  uint16_t chargerFlags = bq.getChargerFlags();
  uint8_t faultFlagsCleared = bq.getFaultFlags();
  uint8_t status0 = statusFlags & 0xFF;
  uint8_t status1 = (statusFlags >> 8) & 0xFF;
  uint8_t flag0 = chargerFlags & 0xFF;
  uint8_t flag1 = (chargerFlags >> 8) & 0xFF;
  
  Serial.println(F("=== Charger Status ==="));
  Serial.print(F("Status: 0x"));
  Serial.print(statusFlags, HEX);
  Serial.print(F(", Fault: 0x"));
  Serial.print(faultFlags, HEX);
  Serial.print(F(", Flags: 0x"));
  Serial.print(chargerFlags, HEX);
  Serial.print(F(", FaultFlags: 0x"));
  Serial.println(faultFlagsCleared, HEX);
  
  // REG0x1D Status 0 flags
  Serial.print(F("Status 0 (0x"));
  Serial.print(status0, HEX);
  Serial.println(F("):"));
  
  if (status0 & BQ25628E_STATUS0_WD_STAT) {
    Serial.println(F("  ⚠️  WD Timer Expired"));
  }
  if (status0 & BQ25628E_STATUS0_SAFETY_TMR_STAT) {
    Serial.println(F("  ⚠️  Safety Timer Expired"));
  }
  if (status0 & BQ25628E_STATUS0_VINDPM_STAT) {
    Serial.println(F("  📉 VINDPM Regulation Active"));
  }
  if (status0 & BQ25628E_STATUS0_IINDPM_STAT) {
    Serial.println(F("  📉 IINDPM/ILIM Regulation Active"));
  }
  if (status0 & BQ25628E_STATUS0_VSYS_STAT) {
    Serial.println(F("  📉 VSYSMIN Regulation Active"));
  }
  if (status0 & BQ25628E_STATUS0_TREG_STAT) {
    Serial.println(F("  🌡️  Thermal Regulation Active"));
  }
  if (status0 & BQ25628E_STATUS0_ADC_DONE_STAT) {
    Serial.println(F("  ✅ ADC Conversion Complete"));
  }
  
  // REG0x1E Status 1 flags
  Serial.print(F("Status 1 (0x"));
  Serial.print(status1, HEX);
  Serial.println(F("):"));
  
  // VBUS Status (bits 2:0)
  uint8_t vbus_stat = status1 & BQ25628E_STATUS1_VBUS_STAT_MASK;
  Serial.print(F("  🔌 VBUS: "));
  switch (vbus_stat) {
    case BQ25628E_VBUS_STAT_NOT_POWERED:
      Serial.println(F("Not Powered"));
      break;
    case BQ25628E_VBUS_STAT_UNKNOWN_ADAPTER:
      Serial.println(F("Unknown Adapter"));
      break;
    default:
      Serial.print(F("Status Code "));
      Serial.println(vbus_stat);
      break;
  }
  
  // Charge Status (bits 4:3)
  uint8_t chg_stat = (status1 & BQ25628E_STATUS1_CHG_STAT_MASK) >> BQ25628E_STATUS1_CHG_STAT_SHIFT;
  Serial.print(F("  🔋 Charge: "));
  switch (chg_stat) {
    case BQ25628E_CHG_STAT_NOT_CHARGING:
      Serial.println(F("Not Charging/Terminated"));
      break;
    case BQ25628E_CHG_STAT_CHARGING:
      Serial.println(F("Charging (CC mode)"));
      break;
    case BQ25628E_CHG_STAT_TAPER:
      Serial.println(F("Taper Charge (CV mode)"));
      break;
    case BQ25628E_CHG_STAT_TOPOFF:
      Serial.println(F("Top-off Timer Active"));
      break;
  }
  
  // REG0x1F Fault Status flags
  Serial.print(F("Fault Status (0x"));
  Serial.print(faultFlags, HEX);
  Serial.println(F("):"));
  
  if (faultFlags & BQ25628E_FAULT_VBUS_FAULT_STAT) {
    Serial.println(F("  🚨 VBUS Fault (OVP/Sleep)"));
  }
  if (faultFlags & BQ25628E_FAULT_BAT_FAULT_STAT) {
    Serial.println(F("  🚨 Battery Fault (OCP/OVP)"));
  }
  if (faultFlags & BQ25628E_FAULT_SYS_FAULT_STAT) {
    Serial.println(F("  🚨 System Fault (Short/OVP)"));
  }
  if (faultFlags & BQ25628E_FAULT_TSHUT_STAT) {
    Serial.println(F("  🔥 Thermal Shutdown"));
  }
  
  // TS Status (bits 2:0)
  uint8_t ts_stat = faultFlags & BQ25628E_FAULT_TS_STAT_MASK;
  Serial.print(F("  🌡️  TS Status: "));
  switch (ts_stat) {
    case BQ25628E_TS_STAT_NORMAL:
      Serial.println(F("Normal"));
      break;
    case BQ25628E_TS_STAT_COLD:
      Serial.println(F("Cold"));
      break;
    case BQ25628E_TS_STAT_HOT:
      Serial.println(F("Hot"));
      break;
    case BQ25628E_TS_STAT_COOL:
      Serial.println(F("Cool"));
      break;
    case BQ25628E_TS_STAT_WARM:
      Serial.println(F("Warm"));
      break;
    case BQ25628E_TS_STAT_PRECOOL:
      Serial.println(F("Pre-cool"));
      break;
    case BQ25628E_TS_STAT_PREWARM:
      Serial.println(F("Pre-warm"));
      break;
    case BQ25628E_TS_STAT_BIAS_FAULT:
      Serial.println(F("Bias Reference Fault"));
      break;
  }
  
  // REG0x20/0x21 Charger Flag status (cleared on read!)
  if (flag0 != 0 || flag1 != 0) {
    Serial.print(F("Charger Flags (0x"));
    Serial.print(flag1, HEX);
    Serial.print(F("/0x"));
    Serial.print(flag0, HEX);
    Serial.println(F(") - CLEARED:"));
    
    // Flag0 bits
    if (flag0 & BQ25628E_FLAG0_WD_FLAG) {
      Serial.println(F("  🚩 WD Timer Expired"));
    }
    if (flag0 & BQ25628E_FLAG0_SAFETY_TMR_FLAG) {
      Serial.println(F("  🚩 Safety Timer Expired"));
    }
    if (flag0 & BQ25628E_FLAG0_VINDPM_FLAG) {
      Serial.println(F("  🚩 VINDPM Regulation Event"));
    }
    if (flag0 & BQ25628E_FLAG0_IINDPM_FLAG) {
      Serial.println(F("  🚩 IINDPM/ILIM Regulation Event"));
    }
    if (flag0 & BQ25628E_FLAG0_VSYS_FLAG) {
      Serial.println(F("  🚩 VSYSMIN Regulation Event"));
    }
    if (flag0 & BQ25628E_FLAG0_TREG_FLAG) {
      Serial.println(F("  🚩 Thermal Regulation Event"));
    }
    if (flag0 & BQ25628E_FLAG0_ADC_DONE_FLAG) {
      Serial.println(F("  🚩 ADC Conversion Complete"));
    }
    
    // Flag1 bits
    if (flag1 & BQ25628E_FLAG1_VBUS_FLAG) {
      Serial.println(F("  🚩 VBUS Status Changed"));
    }
    if (flag1 & BQ25628E_FLAG1_CHG_FLAG) {
      Serial.println(F("  🚩 Charge Status Changed"));
    }
  } else {
    Serial.println(F("No charger flags set"));
  }
  
  // REG0x22 Fault Flag status (cleared on read!)
  if (faultFlagsCleared != 0) {
    Serial.print(F("Fault Flags (0x"));
    Serial.print(faultFlagsCleared, HEX);
    Serial.println(F(") - CLEARED:"));
    
    if (faultFlagsCleared & BQ25628E_FAULT_FLAG_VBUS_FAULT) {
      Serial.println(F("  💥 VBUS Fault Event (OVP/Sleep)"));
    }
    if (faultFlagsCleared & BQ25628E_FAULT_FLAG_BAT_FAULT) {
      Serial.println(F("  💥 Battery Fault Event (OCP/OVP)"));
    }
    if (faultFlagsCleared & BQ25628E_FAULT_FLAG_SYS_FAULT) {
      Serial.println(F("  💥 System Fault Event (OVP/Short)"));
    }
    if (faultFlagsCleared & BQ25628E_FAULT_FLAG_TSHUT) {
      Serial.println(F("  💥 Thermal Shutdown Event"));
    }
    if (faultFlagsCleared & BQ25628E_FAULT_FLAG_TS_CHANGED) {
      Serial.println(F("  💥 TS Status Changed Event"));
    }
  } else {
    Serial.println(F("No fault flags set"));
  }
  
  // Display current interrupt mask status
  uint32_t currentMask = bq.getInterruptMask();
  Serial.print(F("INT Mask: CHG="));
  Serial.print((currentMask & BQ25628E_INT_MASK_CHG) ? F("OFF") : F("ON"));
  Serial.print(F(", VBUS="));
  Serial.print((currentMask & BQ25628E_INT_MASK_VBUS) ? F("OFF") : F("ON"));
  Serial.print(F(", Others="));
  Serial.println((currentMask & ~(BQ25628E_INT_MASK_CHG | BQ25628E_INT_MASK_VBUS)) ? F("OFF") : F("ON"));
  
  Serial.println(F("====================="));
  Serial.println();
}

void loop() {
  static unsigned long lastStatusTime = 0;
  static unsigned long lastADCTime = 0;
  unsigned long currentTime = millis();
  
  // Print all ADC values every 1 second
  if (currentTime - lastADCTime >= 1000) {
    float ibus_current = bq.getIBUScurrent();
    float ibat_current = bq.getIBATcurrent();
    float vbus_voltage = bq.getVBUSvoltage();
    float vpmid_voltage = bq.getVPMIDvoltage();
    float vbat_voltage = bq.getVBATvoltage();
    float vsys_voltage = bq.getVSYSvoltage();
    float thermistor_percent = bq.getThermistorPercent();
    float die_temp = bq.getDieTempC();
    
    Serial.print(F("ADC: IBUS="));
    Serial.print(ibus_current, 3);
    Serial.print(F("A, IBAT="));
    Serial.print(ibat_current, 3);
    Serial.print(F("A, VBUS="));
    Serial.print(vbus_voltage, 3);
    Serial.print(F("V, VPMID="));
    Serial.print(vpmid_voltage, 3);
    Serial.print(F("V, VBAT="));
    Serial.print(vbat_voltage, 3);
    Serial.print(F("V, VSYS="));
    Serial.print(vsys_voltage, 3);
    Serial.print(F("V, TS="));
    Serial.print(thermistor_percent, 1);
    Serial.print(F("%, TDIE="));
    Serial.print(die_temp, 1);
    Serial.println(F("°C"));
    
    lastADCTime = currentTime;
  }
  
  // Print status every 5 seconds
  if (currentTime - lastStatusTime >= 5000) {
    printChargerStatus();
    lastStatusTime = currentTime;
  }
  
  delay(100);
}