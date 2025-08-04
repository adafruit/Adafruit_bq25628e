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
  
  Serial.println("Adafruit BQ25628E Test!");
  
  if (!bq.begin()) {
    Serial.println("Failed to find BQ25628E chip");
    while (1) delay(10);
  }
  
  Serial.println("BQ25628E Found!");
}

void loop() {
  delay(1000);
}