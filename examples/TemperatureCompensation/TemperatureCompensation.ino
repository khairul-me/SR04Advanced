/*
 * TemperatureCompensation.ino - Temperature compensation example
 * 
 * Created by: Md Khairul Islam
 * Institution: Hobart and William Smith Colleges
 * Major: Double major in Robotics and Computer Science
 * 
 * This example demonstrates how temperature affects ultrasonic readings
 * and how to compensate for it.
 */

#include <SR04Advanced.h>

const uint8_t TRIGGER_PIN = 9;
const uint8_t ECHO_PIN = 10;

SR04Advanced sonar(TRIGGER_PIN, ECHO_PIN);

// Simulated temperature changes for demonstration
float currentTemp = 20.0;
float tempDirection = 1.0;
unsigned long lastTempUpdate = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("SR04Advanced - Temperature Compensation Example");
    Serial.println("--------------------------------------------");
    
    sonar.begin();
    sonar.setDebugLevel(DEBUG_BASIC);
    
    // Print initial conditions
    Serial.println("\nStarting conditions:");
    Serial.print("Initial temperature: ");
    Serial.print(currentTemp);
    Serial.println("°C");
    Serial.println("\nThis example will simulate temperature changes");
    Serial.println("and show how they affect measurements.\n");
}

void loop() {
    // Simulate temperature changes
    simulateTemperature();
    
    // Take measurements with and without temperature compensation
    sonar.setTemperature(20.0); // Standard temperature
    float standardReading = sonar.getSmartDistance();
    
    sonar.setTemperature(currentTemp); // Compensated for current temperature
    float compensatedReading = sonar.getSmartDistance();
    
    // Calculate theoretical difference
    float speedOfSound = 331.3 + (0.606 * currentTemp);
    float standardSpeed = 331.3 + (0.606 * 20.0);
    float theoreticalDiff = (speedOfSound - standardSpeed) / standardSpeed * 100;
    
    // Print results
    Serial.print("Temperature: ");
    Serial.print(currentTemp, 1);
    Serial.print("°C | Uncompensated: ");
    Serial.print(standardReading, 1);
    Serial.print(" cm | Compensated: ");
    Serial.print(compensatedReading, 1);
    Serial.print(" cm | Difference: ");
    Serial.print(compensatedReading - standardReading, 2);
    Serial.print(" cm (");
    Serial.print(theoreticalDiff, 1);
    Serial.println("% theoretical)");
    
    delay(100);
}

void simulateTemperature() {
    // Update temperature every 5 seconds
    if (millis() - lastTempUpdate > 5000) {
        currentTemp += tempDirection;
        
        // Reverse direction at temperature limits
        if (currentTemp >= 40.0 || currentTemp <= 0.0) {
            tempDirection *= -1;
        }
        
        lastTempUpdate = millis();
        
        // Print temperature change notification
        Serial.print("\nTemperature changed to: ");
        Serial.print(currentTemp, 1);
        Serial.println("°C");
    }
}

/*
 * Note on Temperature Compensation:
 * 
 * The speed of sound varies with temperature according to the formula:
 * v = 331.3 + (0.606 × T)  m/s
 * where T is the temperature in °C
 * 
 * Without compensation, measurements can be off by:
 * - About 0.17% per °C difference from standard temperature
 * - Up to 7% error over a 40°C temperature range
 * 
 * For accurate measurements in varying conditions, always:
 * 1. Use a temperature sensor if possible
 * 2. Update the temperature regularly
 * 3. Consider humidity effects for extreme precision
 */
