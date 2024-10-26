/*
 * AdvancedMeasurement.ino - Advanced measurement modes example
 * 
 * Created by: Md Khairul Islam
 * Institution: Hobart and William Smith Colleges
 * Major: Double major in Robotics and Computer Science
 * 
 * This example demonstrates different measurement modes of the SR04Advanced library
 * and how to use them for different scenarios.
 */

#include <SR04Advanced.h>

const uint8_t TRIGGER_PIN = 9;
const uint8_t ECHO_PIN = 10;

SR04Advanced sonar(TRIGGER_PIN, ECHO_PIN);

// Button pins for mode switching (optional)
const int MODE_BUTTON_PIN = 2;
int currentMode = 0;

void setup() {
    Serial.begin(9600);
    Serial.println("SR04Advanced - Advanced Measurement Modes Example");
    Serial.println("----------------------------------------------");
    
    // Initialize sensor
    sonar.begin();
    sonar.setDebugLevel(DEBUG_BASIC);
    
    // Optional: Setup mode button
    pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
    
    printModeInstructions();
}

void loop() {
    // Check for mode change (via button or serial)
    checkModeChange();
    
    // Get distance based on current mode
    float distance = sonar.getSmartDistance();
    
    // Get measurement quality metrics
    uint8_t quality = sonar.getSignalQuality();
    uint8_t confidence = sonar.getConfidence();
    
    // Print results
    Serial.print("Mode: ");
    switch(currentMode) {
        case SR04Advanced::MODE_NORMAL:
            Serial.print("NORMAL");
            break;
        case SR04Advanced::MODE_PRECISE:
            Serial.print("PRECISE");
            break;
        case SR04Advanced::MODE_FAST:
            Serial.print("FAST");
            break;
        case SR04Advanced::MODE_ADAPTIVE:
            Serial.print("ADAPTIVE");
            break;
    }
    
    Serial.print(" | Distance: ");
    Serial.print(distance, 1);
    Serial.print(" cm | Quality: ");
    Serial.print(quality);
    Serial.print("% | Confidence: ");
    Serial.print(confidence);
    Serial.println("%");
    
    // Adjust delay based on mode
    switch(currentMode) {
        case SR04Advanced::MODE_FAST:
            delay(50);  // Faster updates
            break;
        case SR04Advanced::MODE_PRECISE:
            delay(200); // Slower but more precise
            break;
        default:
            delay(100); // Normal speed
    }
}

void checkModeChange() {
    // Check button
    static bool lastButtonState = HIGH;
    bool buttonState = digitalRead(MODE_BUTTON_PIN);
    
    if (buttonState == LOW && lastButtonState == HIGH) {
        // Button pressed, change mode
        currentMode = (currentMode + 1) % 4;
        sonar.setMeasurementMode(currentMode);
        printModeInstructions();
        delay(250); // Debounce
    }
    lastButtonState = buttonState;
    
    // Check serial for mode change commands
    if (Serial.available()) {
        char cmd = Serial.read();
        switch(cmd) {
            case 'n':
                currentMode = SR04Advanced::MODE_NORMAL;
                break;
            case 'p':
                currentMode = SR04Advanced::MODE_PRECISE;
                break;
            case 'f':
                currentMode = SR04Advanced::MODE_FAST;
                break;
            case 'a':
                currentMode = SR04Advanced::MODE_ADAPTIVE;
                break;
        }
        sonar.setMeasurementMode(currentMode);
        printModeInstructions();
    }
}

void printModeInstructions() {
    Serial.println("\nCurrent mode changed to: ");
    switch(currentMode) {
        case SR04Advanced::MODE_NORMAL:
            Serial.println("NORMAL MODE");
            Serial.println("- Standard balanced performance");
            Serial.println("- Good for most applications");
            break;
        case SR04Advanced::MODE_PRECISE:
            Serial.println("PRECISE MODE");
            Serial.println("- Higher accuracy but slower");
            Serial.println("- Best for static measurements");
            break;
        case SR04Advanced::MODE_FAST:
            Serial.println("FAST MODE");
            Serial.println("- Quick updates but less filtering");
            Serial.println("- Good for rapid movement detection");
            break;
        case SR04Advanced::MODE_ADAPTIVE:
            Serial.println("ADAPTIVE MODE");
            Serial.println("- Automatically adjusts based on noise");
            Serial.println("- Best for varying conditions");
            break;
    }
    Serial.println("\nCommands:");
    Serial.println("'n' - Normal mode");
    Serial.println("'p' - Precise mode");
    Serial.println("'f' - Fast mode");
    Serial.println("'a' - Adaptive mode");
    Serial.println("Or use the mode button to cycle through modes\n");
}
