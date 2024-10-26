/*
 * DataVisualizer.ino - Real-time data visualization example
 * 
 * Created by: Md Khairul Islam
 * Institution: Hobart and William Smith Colleges
 * Major: Double major in Robotics and Computer Science
 * 
 * This example provides real-time visualization of sensor data
 * using Arduino's Serial Plotter and custom formatting.
 */

#include <SR04Advanced.h>

const uint8_t TRIGGER_PIN = 9;
const uint8_t ECHO_PIN = 10;

SR04Advanced sonar(TRIGGER_PIN, ECHO_PIN);

// Visualization settings
bool plotterMode = true;          // Serial Plotter mode
bool dataLogging = false;         // CSV data logging mode
unsigned long startTime;          // For timestamp in logging
bool showStatistics = false;      // Statistical analysis display
const int STATS_WINDOW = 50;      // Window size for moving statistics

// Circular buffer for statistics
float distances[STATS_WINDOW];
int bufferIndex = 0;
bool bufferFull = false;

void setup() {
    Serial.begin(9600);
    
    if (!plotterMode) {
        Serial.println("SR04Advanced - Data Visualizer Example");
        Serial.println("------------------------------------");
    }
    
    sonar.begin();
    sonar.setDebugLevel(DEBUG_NONE); // Prevent debug messages from interfering with plotting
    
    startTime = millis();
    
    if (!plotterMode) {
        printInstructions();
    }
}

void loop() {
    // Get measurements
    float raw = sonar.getRawDistance();
    float filtered = sonar.getSmartDistance();
    float noise = sonar.getNoise() * 100; // Convert to percentage
    uint8_t quality = sonar.getSignalQuality();
    
    // Update statistics buffer
    updateStatistics(filtered);
    
    if (plotterMode) {
        // Format for Arduino Serial Plotter
        Serial.print("Raw:");
        Serial.print(raw);
        Serial.print(",Filtered:");
        Serial.print(filtered);
        Serial.print(",Noise%:");
        Serial.print(noise);
        Serial.print(",Quality:");
        Serial.print(quality);
        
        if (showStatistics) {
            Serial.print(",MovingAvg:");
            Serial.print(calculateMovingAverage());
            Serial.print(",UpperBound:");
            Serial.print(calculateMovingAverage() + calculateStdDev() * 2);
            Serial.print(",LowerBound:");
            Serial.print(calculateMovingAverage() - calculateStdDev() * 2);
        }
        
        Serial.println();
    } 
    else if (dataLogging) {
        // CSV format for data logging
        unsigned long timeStamp = millis() - startTime;
        Serial.print(timeStamp);
        Serial.print(",");
        Serial.print(raw);
        Serial.print(",");
        Serial.print(filtered);
        Serial.print(",");
        Serial.print(noise);
        Serial.print(",");
        Serial.print(quality);
        
        if (showStatistics) {
            Serial.print(",");
            Serial.print(calculateMovingAverage());
            Serial.print(",");
            Serial.print(calculateStdDev());
        }
        
        Serial.println();
    } 
    else {
        // Human-readable format
        Serial.print("Time: ");
        Serial.print((millis() - startTime) / 1000.0, 1);
        Serial.print("s | Raw: ");
        Serial.print(raw, 1);
        Serial.print(" cm | Filtered: ");
        Serial.print(filtered, 1);
        Serial.print(" cm | Noise: ");
        Serial.print(noise, 1);
        Serial.print("% | Quality: ");
        Serial.print(quality);
        Serial.println("%");
        
        if (showStatistics) {
            printStatistics();
        }
    }
    
    checkCommands();
    delay(50);
}

void updateStatistics(float value) {
    distances[bufferIndex] = value;
    bufferIndex = (bufferIndex + 1) % STATS_WINDOW;
    if (bufferIndex == 0) {
        bufferFull = true;
    }
}

float calculateMovingAverage() {
    float sum = 0;
    int count = bufferFull ? STATS_WINDOW : bufferIndex;
    
    for (int i = 0; i < count; i++) {
        sum += distances[i];
    }
    
    return count > 0 ? sum / count : 0;
}

float calculateStdDev() {
    float mean = calculateMovingAverage();
    float sumSquares = 0;
    int count = bufferFull ? STATS_WINDOW : bufferIndex;
    
    for (int i = 0; i < count; i++) {
        float diff = distances[i] - mean;
        sumSquares += diff * diff;
    }
    
    return count > 1 ? sqrt(sumSquares / (count - 1)) : 0;
}

void printStatistics() {
    Serial.println("\nMoving Statistics:");
    Serial.print("Average: ");
    Serial.print(calculateMovingAverage(), 2);
    Serial.print(" cm | StdDev: ");
    Serial.print(calculateStdDev(), 3);
    Serial.println(" cm");
    Serial.print("Bounds: ");
    Serial.print(calculateMovingAverage() - calculateStdDev() * 2, 2);
    Serial.print(" to ");
    Serial.print(calculateMovingAverage() + calculateStdDev() * 2, 2);
    Serial.println(" cm (±2σ)");
}

void checkCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        switch(cmd) {
            case 'p':
                plotterMode = !plotterMode;
                if (!plotterMode) {
                    Serial.println("\nPlotter mode disabled");
                }
                break;
                
            case 'l':
                dataLogging = !dataLogging;
                if (dataLogging) {
                    startTime = millis();
                    Serial.println("\nTime,Raw,Filtered,Noise,Quality"
                                 + String(showStatistics ? ",Average,StdDev" : ""));
                } else {
                    Serial.println("\nData logging stopped");
                }
                break;
                
            case 's':
                showStatistics = !showStatistics;
                Serial.print("\nStatistics display ");
                Serial.println(showStatistics ? "enabled" : "disabled");
                break;
                
            case 'r':
                // Reset statistics
                bufferIndex = 0;
                bufferFull = false;
                startTime = millis();
                Serial.println("\nStatistics reset");
                break;
                
            case 'h':
                if (!plotterMode) {
                    printInstructions();
                }
                break;
        }
    }
}

void printInstructions() {
    Serial.println("\nData Visualizer Commands:");
    Serial.println("------------------------");
    Serial.println("'p' - Toggle Serial Plotter mode");
    Serial.println("'l' - Toggle CSV data logging");
    Serial.println("'s' - Toggle statistics display");
    Serial.println("'r' - Reset statistics");
    Serial.println("'h' - Show this help");
    
    Serial.println("\nModes:");
    Serial.println("1. Plotter Mode - Use with Arduino Serial Plotter");
    Serial.println("   - Shows real-time graphs of all measurements");
    Serial.println("2. Data Logging - CSV format for external analysis");
    Serial.println("   - Timestamps in milliseconds");
    Serial.println("3. Human Readable - Formatted text output");
    
    Serial.println("\nStatistics Features:");
    Serial.println("- Moving average over last 50 samples");
    Serial.println("- Standard deviation calculation");
    Serial.println("- Confidence bounds (±2 standard deviations)");
    
    Serial.println("\nNote: Use Arduino IDE's Serial Plotter");
    Serial.println("(Tools -> Serial Plotter) for visualization");
    Serial.println();
}

/*
 * Usage Guide:
 * 
 * Serial Plotter Mode (default):
 * - Opens automatically in Arduino IDE's Serial Plotter
 * - Shows multiple data series in different colors
 * - Enable statistics to see confidence bounds
 * 
 * Data Logging Mode:
 * - CSV format for spreadsheet analysis
 * - Can be captured with Serial Monitor
 * - Includes timestamp for time-series analysis
 * 
 * Visualized Data:
 * - Raw distance (cm)
 * - Filtered distance (cm)
 * - Noise level (%)
 * - Signal quality (%)
 * - Moving average and bounds (when statistics enabled)
 */
