# SR04Advanced Library

Advanced Arduino library for the HC-SR04 ultrasonic sensor with smart filtering and diagnostics.

Created by: Md Khairul Islam
Institution: Hobart and William Smith Colleges
Major: Double major in Robotics and Computer Science

## Features

- Smart filtering with multiple algorithms:
  - Low-pass filter
  - Median filter
  - Kalman filter
  - Adaptive smart filtering
- Temperature compensation
- Multiple measurement modes:
  - Normal mode
  - Precise mode (multiple readings)
  - Fast mode
  - Adaptive mode
- Advanced diagnostics:
  - Signal quality assessment
  - Noise level measurement
  - Confidence calculation
- Real-time visualization
- JSON output support
- Comprehensive debugging options

## Installation

### Arduino IDE
1. Download the ZIP file of this repository
2. Open Arduino IDE
3. Go to Sketch > Include Library > Add .ZIP Library
4. Select the downloaded ZIP file

### PlatformIO
Add the following to your `platformio.ini`:
```ini
lib_deps =
    sr04advanced
```

## Quick Start

```cpp
#include <SR04Advanced.h>

#define TRIGGER_PIN 9
#define ECHO_PIN 10

SR04Advanced sonar(TRIGGER_PIN, ECHO_PIN);

void setup() {
    Serial.begin(9600);
    sonar.begin();
}

void loop() {
    float distance = sonar.getSmartDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(100);
}
```

## Examples

1. BasicDistance - Simple distance measurement
2. AdvancedMeasurement - Different measurement modes
3. TemperatureCompensation - Temperature-compensated measurements
4. JsonOutput - Data output in JSON format
5. NoiseAnalysis - Analyze sensor noise and filtering
6. DataVisualizer - Real-time data visualization

## Documentation

### Core Methods
- `begin(bool autoCalibrate = true)` - Initialize sensor
- `getDistance()` - Get filtered distance
- `getRawDistance()` - Get unfiltered distance
- `getSmartDistance()` - Get distance with advanced filtering

### Configuration
- `setDebugLevel(uint8_t level)` - Set debug output level
- `setFilterMode(uint8_t mode)` - Set filtering mode
- `setMeasurementMode(uint8_t mode)` - Set measurement mode
- `setTemperature(float temp_c)` - Set ambient temperature

### Diagnostics
- `calibrate()` - Perform sensor calibration
- `getSignalQuality()` - Get signal quality (0-100%)
- `getConfidence()` - Get measurement confidence (0-100%)
- `getNoise()` - Get noise level

## Contributing

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues, questions, or contributions:
- Open an issue on GitHub
- Contact: khairul.robotics@gmail.com

## Acknowledgments

- Thanks to the Arduino community
- Special thanks to faculty and peers at Hobart and William Smith Colleges
