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
}

void loop() {
  delay(1000);
}