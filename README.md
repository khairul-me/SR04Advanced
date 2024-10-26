# SR04Advanced Library

## Overview
Advanced Arduino library for the HC-SR04 ultrasonic sensor that provides sophisticated filtering, noise reduction, and diagnostic capabilities. This library significantly improves measurement accuracy and reliability compared to basic HC-SR04 implementations.

Created by: Md Khairul Islam
Institution: Hobart and William Smith Colleges
Major: Double major in Robotics and Computer Science

## Key Features

### Advanced Filtering
- **Multiple Filter Types:**
  - Low-pass filter for smooth measurements
  - Median filter for spike removal
  - Kalman filter for precise tracking
  - Smart adaptive filtering that combines multiple methods
  - Option to use raw unfiltered data

### Measurement Modes
- **Normal Mode:** Balanced performance for general use
- **Precise Mode:** Higher accuracy through multiple readings
- **Fast Mode:** Rapid measurements for quick updates
- **Adaptive Mode:** Automatically adjusts based on noise levels

### Smart Features
- Temperature compensation for accurate measurements
- Automatic calibration capability
- Signal quality assessment
- Noise level detection and measurement
- Confidence level calculation
- Statistical analysis of measurements

### Diagnostics & Debugging
- Real-time data visualization
- JSON output support
- Multiple debug levels
- Comprehensive error checking
- Statistical reporting

## Hardware Requirements

### Components Needed
- Arduino board (Uno, Nano, Mega, etc.)
- HC-SR04 Ultrasonic Sensor
- Jumper wires
- Optional: Temperature sensor for enhanced accuracy

### Connections
```
HC-SR04     Arduino
VCC    -->  5V
GND    -->  GND
TRIG   -->  Digital Pin 9 (configurable)
ECHO   -->  Digital Pin 10 (configurable)
```

## Installation

### Method 1: Arduino Library Manager
1. Open Arduino IDE
2. Go to Sketch > Include Library > Manage Libraries
3. Search for "SR04Advanced"
4. Click Install

### Method 2: Manual Installation
1. Download the ZIP file from this repository
2. In Arduino IDE: Sketch > Include Library > Add .ZIP Library
3. Select the downloaded ZIP file

### Method 3: PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps =
    sr04advanced
```

## Quick Start Guide

### Basic Usage
```cpp
#include <SR04Advanced.h>

const int TRIGGER_PIN = 9;
const int ECHO_PIN = 10;

SR04Advanced sonar(TRIGGER_PIN, ECHO_PIN);

void setup() {
    Serial.begin(9600);
    sonar.begin();  // Initializes with auto-calibration
}

void loop() {
    float distance = sonar.getSmartDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(100);
}
```

### Advanced Usage
```cpp
#include <SR04Advanced.h>

SR04Advanced sonar(9, 10);

void setup() {
    Serial.begin(9600);
    
    // Advanced configuration
    sonar.setDebugLevel(DEBUG_ADVANCED);
    sonar.setMeasurementMode(SR04Advanced::MODE_PRECISE);
    sonar.setFilterParams(0.1, 5);  // Alpha, window size
    sonar.setTemperature(25.0);     // Ambient temperature in Celsius
    
    // Initialize and calibrate
    sonar.begin(true);  // true = perform auto-calibration
    
    // Enable data visualization
    sonar.enablePlotter(true);
}

void loop() {
    float distance = sonar.getSmartDistance();
    uint8_t quality = sonar.getSignalQuality();
    uint8_t confidence = sonar.getConfidence();
    
    // Print detailed information
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm | Quality: ");
    Serial.print(quality);
    Serial.print("% | Confidence: ");
    Serial.print(confidence);
    Serial.println("%");
    
    delay(100);
}
```

## Example Sketches

### 1. BasicDistance
Simple distance measurement with basic filtering.
```cpp
Examples/BasicDistance/BasicDistance.ino
```

### 2. AdvancedMeasurement
Demonstrates different measurement modes and their uses.
```cpp
Examples/AdvancedMeasurement/AdvancedMeasurement.ino
```

### 3. TemperatureCompensation
Shows how temperature affects measurements and compensation.
```cpp
Examples/TemperatureCompensation/TemperatureCompensation.ino
```

### 4. JsonOutput
Provides sensor data in JSON format for external processing.
```cpp
Examples/JsonOutput/JsonOutput.ino
```

### 5. NoiseAnalysis
Tools for analyzing sensor noise and filter performance.
```cpp
Examples/NoiseAnalysis/NoiseAnalysis.ino
```

### 6. DataVisualizer
Real-time data visualization using Arduino's Serial Plotter.
```cpp
Examples/DataVisualizer/DataVisualizer.ino
```

## API Reference

### Core Methods
```cpp
void begin(bool autoCalibrate = true)
float getDistance()
float getRawDistance()
float getSmartDistance()
```

### Configuration Methods
```cpp
void setDebugLevel(uint8_t level)
void setFilterMode(uint8_t mode)
void setMeasurementMode(uint8_t mode)
void setFilterParams(float alpha, int window_size)
void setThresholds(float min_dist, float max_dist)
void setTemperature(float temp_c)
```

### Diagnostic Methods
```cpp
void calibrate(uint16_t samples = 20)
uint8_t getSignalQuality()
uint8_t getConfidence()
float getNoise()
bool isMeasurementValid()
```

### Statistical Methods
```cpp
float getAverageDistance()
float getMinDistance()
float getMaxDistance()
float getStandardDeviation()
```

## Troubleshooting Guide

### Common Issues

1. **Inconsistent Readings**
   - Check power supply stability
   - Ensure proper wiring
   - Try different measurement modes
   - Enable debugging for more information

2. **High Noise Levels**
   - Verify sensor mounting stability
   - Check for interference sources
   - Use MODE_PRECISE for better accuracy
   - Adjust filter parameters

3. **Slow Response**
   - Switch to MODE_FAST
   - Reduce filter window size
   - Check for code delays
   - Optimize loop timing

4. **Temperature Effects**
   - Update temperature regularly
   - Use temperature compensation
   - Consider ambient conditions

### Debug Levels
```cpp
DEBUG_NONE      // No debug output
DEBUG_BASIC     // Basic status messages
DEBUG_ADVANCED  // Detailed information
DEBUG_VERBOSE   // All available information
```

## Performance Optimization

### Best Practices
1. Choose appropriate measurement mode for your application
2. Use temperature compensation when accuracy is critical
3. Adjust filter parameters based on your needs
4. Monitor signal quality and confidence levels
5. Use appropriate delay between measurements

### Filter Selection Guide
- **Low-pass Filter:** Smooth, continuous measurements
- **Median Filter:** Remove occasional spikes
- **Kalman Filter:** Precise tracking with noise
- **Smart Filter:** Best for general use

## Contributing

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## Version History

- 1.0.0 (2024-10-26)
  - Initial release
  - Basic filtering capabilities
  - Temperature compensation
  - Diagnostic features

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support and Contact

For support, feature requests, or contributions:
1. Open an issue on GitHub
2. Contact: khairul.robotics@gmail.com
3. Visit: https://www.linkedin.com/in/khairul7/

## Acknowledgments

- Faculty and peers at Hobart and William Smith Colleges
- Arduino community members
- Contributors and testers

---
Copyright © 2024 Md Khairul Islam. All rights reserved.
