/*
 * JsonOutput.ino - JSON data output example
 * 
 * Created by: Md Khairul Islam
 * Institution: Hobart and William Smith Colleges
 * Major: Double major in Robotics and Computer Science
 * 
 * This example demonstrates how to get sensor data in JSON format
 * for easy integration with other systems or data logging.
 */

#include <SR04Advanced.h>

const uint8_t TRIGGER_PIN = 9;
const uint8_t ECHO_PIN = 10;

SR04Advanced sonar(TRIGGER_PIN, ECHO_PIN);

// Output mode flags
bool prettyPrint = true;
bool continuousMode = true;

void setup() {
    Serial.begin(9600);
    Serial.println("SR04Advanced - JSON Output Example");
    Serial.println("--------------------------------");
    
    sonar.begin();
    
    printInstructions();
}

void loop() {
    // Take measurement
    sonar.getSmartDistance();
    
    // Check for commands
    checkCommands();
    
    // Output JSON data
    if (continuousMode) {
        outputJson();
    }
    
    delay(100);
}

void outputJson() {
    if (prettyPrint) {
        // Get JSON string and format it
        String json = sonar.getStatusJson();
        prettyPrintJson(json);
    } else {
        // Direct output
        Serial.println(sonar.getStatusJson());
    }
}

void prettyPrintJson(String json) {
    int indent = 0;
    bool inQuotes = false;
    
    Serial.println();
    for (uint16_t i = 0; i < json.length(); i++) {
        char c = json.charAt(i);
        
        if (c == '"' && json.charAt(i-1) != '\\') {
            inQuotes = !inQuotes;
        }
        
        if (!inQuotes) {
            if (c == '{' || c == '[') {
                Serial.println(c);
                indent += 2;
                printIndent(indent);
                continue;
            }
            if (c == '}' || c == ']') {
                Serial.println();
                indent -= 2;
                printIndent(indent);
            }
            if (c == ',') {
                Serial.println(c);
                printIndent(indent);
                continue;
            }
        }
        
        Serial.print(c);
    }
    Serial.println();
}

void printIndent(int indent) {
    for (int i = 0; i < indent; i++) {
        Serial.print(" ");
    }
}

void checkCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        switch(cmd) {
            case 'p':
                prettyPrint = !prettyPrint;
                Serial.print("\nPretty print ");
                Serial.println(prettyPrint ? "enabled" : "disabled");
                break;
            case 'c':
                continuousMode = !continuousMode;
                Serial.print("\nContinuous mode ");
                Serial.println(continuousMode ? "enabled" : "disabled");
                break;
            case 's':
                if (!continuousMode) {
                    outputJson();
                }
                break;
            case 'h':
                printInstructions();
                break;
        }
    }
}

void printInstructions() {
    Serial.println("\nCommands:");
    Serial.println("'p' - Toggle pretty print");
    Serial.println("'c' - Toggle continuous mode");
    Serial.println("'s' - Single reading (in non-continuous mode)");
    Serial.println("'h' - Show this help");
    Serial.println("\nJSON Output includes:");
    Serial.println("- Raw and filtered distances");
    Serial.println("- Signal quality and confidence");
    Serial.println("- Noise level and statistics");
    Serial.println("- Temperature and calibration status\n");
}

/*
 * Example JSON output:
 * {
 *   "raw_distance": 123.45,
 *   "filtered_distance": 123.42,
 *   "signal_quality": 95,
 *   "confidence": 92,
 *   "noise_level": 0.02,
 *   "readings_count": 150,
 *   "average_distance": 123.40,
 *   "min_distance": 122.98,
 *   "max_distance": 123.86,
 *   "std_deviation": 0.15,
 *   "measurement_mode": "NORMAL",
 *   "temperature": 20.0,
 *   "is_calibrated": true
 * }
 */
