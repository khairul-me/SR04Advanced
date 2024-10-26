# SR04Advanced Library

<div align="center">
<h3>Advanced Ultrasonic Sensor Library with Smart Filtering</h3>
<p>Created by: Md Khairul Islam</p>
<p>Hobart and William Smith Colleges</p>
<p>Double major in Robotics and Computer Science</p>
</div>

## Quick Navigation
- [Overview](#overview)
- [Hardware Setup](#hardware-setup)
- [Features](#features)
- [Installation](#installation)
- [Usage Examples](#usage-examples)
- [API Reference](#api-reference)
- [Advanced Topics](#advanced-topics)
- [Troubleshooting](#troubleshooting)

## Overview

SR04Advanced is a sophisticated Arduino library that enhances HC-SR04 ultrasonic sensor measurements through advanced filtering, noise reduction, and intelligent processing techniques.

### Hardware Connection Diagram

```mermaid
graph LR
    A[Arduino] -- 5V --- B[HC-SR04]
    A -- GND --- B
    A -- Pin 9/TRIG --- B
    A -- Pin 10/ECHO --- B
    
    style A fill:#4CAF50,stroke:#fff,stroke-width:2px
    style B fill:#2196F3,stroke:#fff,stroke-width:2px
```

### Library Architecture

```mermaid
graph TD
    A[Core Library] --> B[Filtering Layer]
    A --> C[Measurement Modes]
    A --> D[Analysis & Output]
    
    B --> B1[Low-Pass Filter]
    B --> B2[Median Filter]
    B --> B3[Kalman Filter]
    B --> B4[Smart Filter]
    
    C --> C1[Normal Mode]
    C --> C2[Precise Mode]
    C --> C3[Fast Mode]
    C --> C4[Adaptive Mode]
    
    D --> D1[JSON Output]
    D --> D2[Visualization]
    D --> D3[Diagnostics]
    D --> D4[Statistics]
    
    style A fill:#FF5722,stroke:#fff,stroke-width:2px
    style B fill:#2196F3,stroke:#fff,stroke-width:2px
    style C fill:#4CAF50,stroke:#fff,stroke-width:2px
    style D fill:#9C27B0,stroke:#fff,stroke-width:2px
```

### Signal Processing Flow

```mermaid
graph LR
    A[Raw Signal] --> B[Filtering]
    B --> C[Processing]
    C --> D[Validation]
    D --> E[Final Output]
    
    style A fill:#FF5722,stroke:#fff,stroke-width:2px
    style B fill:#FF9800,stroke:#fff,stroke-width:2px
    style C fill:#FFC107,stroke:#fff,stroke-width:2px
    style D fill:#8BC34A,stroke:#fff,stroke-width:2px
    style E fill:#4CAF50,stroke:#fff,stroke-width:2px
```

## Features

### Advanced Filtering
| Filter Type | Best For | Performance Impact |
|-------------|----------|-------------------|
| Low-Pass | Smooth, continuous measurements | Low |
| Median | Spike removal | Medium |
| Kalman | Precise tracking | High |
| Smart | Adaptive filtering | Medium-High |

### Measurement Modes
```mermaid
graph TD
    A[Measurement Modes] --> B[Normal]
    A --> C[Precise]
    A --> D[Fast]
    A --> E[Adaptive]
    
    B --> B1[Balanced Performance]
    C --> C1[Multiple Readings]
    D --> D1[Quick Updates]
    E --> E1[Noise-Based Adjustment]
```

## Installation

### Method 1: Arduino Library Manager
```mermaid
graph TD
    A[Arduino IDE] --> B[Manage Libraries]
    B --> C[Search 'SR04Advanced']
    C --> D[Install]
```

### Method 2: Manual Installation
1. Download ZIP
2. Arduino IDE: Sketch > Include Library > Add .ZIP Library

## Basic Usage

```cpp
#include <SR04Advanced.h>

const int TRIGGER_PIN = 9;
const int ECHO_PIN = 10;

SR04Advanced sonar(TRIGGER_PIN, ECHO_PIN);

void setup() {
    Serial.begin(9600);
    sonar.begin();  // Auto-calibration enabled
}

void loop() {
    float distance = sonar.getSmartDistance();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(100);
}
```

## Filter Performance Visualization

Here's how different filters affect the sensor readings:

```mermaid
graph TD
    subgraph "Raw Signal"
        A1[Noisy Data]
    end
    
    subgraph "Low-Pass Filter"
        B1[Smoothed Data]
    end
    
    subgraph "Median Filter"
        C1[Spike Removed]
    end
    
    subgraph "Kalman Filter"
        D1[Tracked Data]
    end
    
    A1 --> B1
    A1 --> C1
    A1 --> D1
```

## Advanced Features

### Temperature Compensation
```cpp
// Update temperature for accurate measurements
sonar.setTemperature(25.0);  // 25°C
```

### Signal Quality Monitoring
```mermaid
graph LR
    A[Signal] --> B{Quality Check}
    B --> C[High Quality]
    B --> D[Low Quality]
    C --> E[Use Reading]
    D --> F[Apply Extra Filtering]
```

## Performance Metrics

### Filter Comparison
| Filter Type | CPU Usage | Memory | Accuracy | Latency |
|-------------|-----------|---------|-----------|----------|
| None | ⭐ | ⭐ | ⭐ | ⭐⭐⭐ |
| Low-Pass | ⭐⭐ | ⭐⭐ | ⭐⭐ | ⭐⭐⭐ |
| Median | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |
| Kalman | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |
| Smart | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |

## Troubleshooting

### Common Issues Flowchart
```mermaid
graph TD
    A[Issue Detected] --> B{Check Type}
    B -->|Inconsistent Readings| C[Check Power Supply]
    B -->|High Noise| D[Check Environment]
    B -->|Slow Response| E[Check Mode Settings]
    
    C --> F[Verify Wiring]
    D --> G[Reduce Interference]
    E --> H[Adjust Parameters]
```

## Contributing

1. Fork the repository
2. Create feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes
4. Push to branch
5. Open Pull Request

## Support and Contact

- GitHub Issues: [Open Issue](https://github.com/yourusername/SR04Advanced/issues)
- Email: [your.email@example.com]

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---
Made with 💡 by Md Khairul Islam
