# Adafruit BQ25628E Library [![Build Status](https://github.com/adafruit/Adafruit_BQ25628E/workflows/Arduino%20Library%20CI/badge.svg)](https://github.com/adafruit/Adafruit_BQ25628E/actions)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_BQ25628E/html/index.html)

Arduino library for the BQ25628E I2C Battery Charger with Power Path Management

## About the BQ25628E

The BQ25628E is a 1-cell Li-Ion battery charger with power path management:
- 2A charging current capability
- I2C interface for control and monitoring
- Power path management for seamless operation
- Integrated ADC for voltage and current monitoring
- Configurable charging parameters
- Safety features including thermal regulation

## Installation

Download and install the library using the Arduino Library Manager or by downloading the latest release from GitHub.

### Dependencies

This library requires:
- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)

## Hardware

Connect the BQ25628E to your microcontroller via I2C. The default I2C address is 0x6A.

## License

MIT License. See LICENSE file for details.

## Contributing

Contributions are welcome! Please read the contributing guidelines and submit pull requests to the main repository.