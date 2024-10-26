/*
 * BasicDistance.ino - Basic example for SR04Advanced library
 * 
 * Created by: Md Khairul Islam
 * Institution: Hobart and William Smith Colleges
 * Major: Double major in Robotics and Computer Science
 * 
 * This example demonstrates the basic usage of the SR04Advanced library
 * with an HC-SR04 ultrasonic sensor. It shows how to get filtered
 * distance measurements with improved accuracy.
 */

#include <SR04Advanced.h>

// Pin definitions for HC-SR04
const uint8_t TRIGGER_PIN = 9;  // Connect to Trigger Pin on the sensor
const uint8_t ECHO_PIN = 10;    // Connect to Echo Pin on the sensor

// Create sensor instance
SR04Advanced sonar(TRIGGER_PIN, ECHO_PIN);

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    Serial.println("SR04Advanced Basic Distance Example");
    Serial.println("-----------------------------------");
    
    // Initialize the sensor (with automatic calibration)
    sonar.begin();
    
    // Optional: Set debug level for additional information
    sonar.setDebugLevel(DEBUG_BASIC);
    
    // Optional: Enable real-time plotting
    sonar.enablePlotter(true);
    
    Serial.println("Sensor initialized and calibrated");
    Serial.println("Starting measurements...\n");
}

void loop() {
    // Get basic filtered distance
    float distance = sonar.getDistance();
    
    // Get signal quality and confidence levels
    uint8_t quality = sonar.getSignalQuality();
    uint8_t confidence = sonar.getConfidence();
    
    // Print readings in a formatted way
    Serial.print("Distance: ");
    Serial.print(distance, 1);  // One decimal place
    Serial.print(" cm | Quality: ");
    Serial.print(quality);
    Serial.print("% | Confidence: ");
    Serial.print(confidence);
    Serial.println("%");
    
    // Optional: Print detailed debug information every second
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime >= 1000) {
        sonar.printDebugInfo();
        lastDebugTime = millis();
    }
    
    // Delay between readings (adjust as needed)
    delay(100);  // 100ms = 10 readings per second
}

/*
 * Serial Plotter View:
 * Open Arduino IDE's Serial Plotter (Tools -> Serial Plotter) to see
 * real-time visualization of:
 * - Raw distance measurements
 * - Filtered distance
 * - Noise level
 * - Confidence level
 *
 * Connection Guide:
 * - Connect HC-SR04 VCC to Arduino 5V
 * - Connect HC-SR04 GND to Arduino GND
 * - Connect HC-SR04 TRIG to Arduino pin 9 (or as defined)
 * - Connect HC-SR04 ECHO to Arduino pin 10 (or as defined)
 *
 * Note: The sensor automatically uses smart filtering to provide
 * stable and accurate readings. For more advanced features, check
 * other examples included with the library.
 */
