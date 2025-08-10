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
  
  Serial.println(F("All tests completed!"));
}

void loop() {
  delay(1000);
}