/*
 * NoiseAnalysis.ino - Noise analysis and filtering demonstration
 * 
 * Created by: Md Khairul Islam
 * Institution: Hobart and William Smith Colleges
 * Major: Double major in Robotics and Computer Science
 * 
 * This example demonstrates the noise analysis capabilities
 * and different filtering methods available in the library.
 */

#include <SR04Advanced.h>

const uint8_t TRIGGER_PIN = 9;
const uint8_t ECHO_PIN = 10;

SR04Advanced sonar(TRIGGER_PIN, ECHO_PIN);

// Analysis settings
const int SAMPLE_SIZE = 100;
float samples[SAMPLE_SIZE];
int sampleIndex = 0;
bool analysisMode = false;

// Current filter mode
int currentFilter = FILTER_SMART;

void setup() {
    Serial.begin(9600);
    Serial.println("SR04Advanced - Noise Analysis Example");
    Serial.println("-----------------------------------");
    
    sonar.begin();
    sonar.setDebugLevel(DEBUG_BASIC);
    
    printInstructions();
}

void loop() {
    if (analysisMode) {
        performNoiseAnalysis();
    } else {
        compareFilters();
    }
    
    checkCommands();
    delay(50);
}

void compareFilters() {
    // Get measurements with different filters
    float raw = sonar.getRawDistance();
    
    sonar.setFilterMode(FILTER_LOWPASS);
    float lowPass = sonar.getDistance();
    
    sonar.setFilterMode(FILTER_MEDIAN);
    float median = sonar.getDistance();
    
    sonar.setFilterMode(FILTER_KALMAN);
    float kalman = sonar.getDistance();
    
    sonar.setFilterMode(FILTER_SMART);
    float smart = sonar.getDistance();
    
    // Calculate noise level
    float noise = sonar.getNoise();
    uint8_t quality = sonar.getSignalQuality();
    
    // Print comparison
    Serial.println("\nFilter Comparison:");
    Serial.println("------------------");
    Serial.print("Raw: ");
    Serial.print(raw, 2);
    Serial.print(" cm | Noise: ");
    Serial.print(noise * 100, 1);
    Serial.println("%");
    
    Serial.print("Low-Pass: ");
    Serial.print(lowPass, 2);
    Serial.print(" cm | Diff: ");
    Serial.print(abs(raw - lowPass), 3);
    Serial.println(" cm");
    
    Serial.print("Median: ");
    Serial.print(median, 2);
    Serial.print(" cm | Diff: ");
    Serial.print(abs(raw - median), 3);
    Serial.println(" cm");
    
    Serial.print("Kalman: ");
    Serial.print(kalman, 2);
    Serial.print(" cm | Diff: ");
    Serial.print(abs(raw - kalman), 3);
    Serial.println(" cm");
    
    Serial.print("Smart: ");
    Serial.print(smart, 2);
    Serial.print(" cm | Diff: ");
    Serial.print(abs(raw - smart), 3);
    Serial.println(" cm");
    
    Serial.print("Signal Quality: ");
    Serial.print(quality);
    Serial.println("%");
}

void performNoiseAnalysis() {
    // Collect sample
    samples[sampleIndex] = sonar.getRawDistance();
    sampleIndex++;
    
    if (sampleIndex >= SAMPLE_SIZE) {
        // Calculate statistics
        float sum = 0;
        float sumSquared = 0;
        float min = 999999;
        float max = -999999;
        
        for (int i = 0; i < SAMPLE_SIZE; i++) {
            sum += samples[i];
            sumSquared += samples[i] * samples[i];
            min = min(min, samples[i]);
            max = max(max, samples[i]);
        }
        
        float mean = sum / SAMPLE_SIZE;
        float variance = (sumSquared / SAMPLE_SIZE) - (mean * mean);
        float stdDev = sqrt(variance);
        float range = max - min;
        
        // Print analysis
        Serial.println("\nNoise Analysis Results:");
        Serial.println("----------------------");
        Serial.print("Mean Distance: ");
        Serial.print(mean, 2);
        Serial.println(" cm");
        Serial.print("Standard Deviation: ");
        Serial.print(stdDev, 3);
        Serial.println(" cm");
        Serial.print("Range: ");
        Serial.print(range, 2);
        Serial.println(" cm");
        Serial.print("Min: ");
        Serial.print(min, 2);
        Serial.print(" cm, Max: ");
        Serial.print(max, 2);
        Serial.println(" cm");
        Serial.print("Coefficient of Variation: ");
        Serial.print((stdDev/mean) * 100, 2);
        Serial.println("%");
        
        // Additional noise metrics
        float snr = mean / stdDev;  // Signal-to-Noise Ratio
        float peakToPeak = max - min;
        float noisePower = stdDev * stdDev;
        
        Serial.print("Signal-to-Noise Ratio: ");
        Serial.println(snr, 2);
        Serial.print("Peak-to-Peak: ");
        Serial.print(peakToPeak, 2);
        Serial.println(" cm");
        Serial.print("Noise Power: ");
        Serial.print(noisePower, 4);
        Serial.println(" cm²");
        
        // Reset for next analysis
        sampleIndex = 0;
        Serial.println("\nStarting new analysis...");
    } else {
        // Show progress
        if (sampleIndex % 10 == 0) {
            Serial.print("Collecting samples: ");
            Serial.print((sampleIndex * 100) / SAMPLE_SIZE);
            Serial.println("%");
        }
    }
}

void checkCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        switch(cmd) {
            case 'a':
                analysisMode = !analysisMode;
                sampleIndex = 0;
                Serial.print("\nAnalysis mode ");
                Serial.println(analysisMode ? "enabled" : "disabled");
                break;
                
            case 'f':
                // Cycle through filter modes
                currentFilter = (currentFilter + 1) % 5;  // 5 filter modes
                sonar.setFilterMode(currentFilter);
                Serial.print("\nFilter mode changed to: ");
                switch(currentFilter) {
                    case FILTER_NONE:
                        Serial.println("None");
                        break;
                    case FILTER_LOWPASS:
                        Serial.println("Low-Pass");
                        break;
                    case FILTER_MEDIAN:
                        Serial.println("Median");
                        break;
                    case FILTER_KALMAN:
                        Serial.println("Kalman");
                        break;
                    case FILTER_SMART:
                        Serial.println("Smart");
                        break;
                }
                break;
                
            case 'c':
                sonar.calibrate();
                Serial.println("\nSensor calibrated");
                break;
                
            case 'r':
                sampleIndex = 0;
                Serial.println("\nAnalysis reset");
                break;
                
            case 'h':
                printInstructions();
                break;
        }
    }
}

void printInstructions() {
    Serial.println("\nNoise Analysis Commands:");
    Serial.println("------------------------");
    Serial.println("'a' - Toggle between filter comparison and noise analysis");
    Serial.println("'f' - Cycle through filter modes");
    Serial.println("'c' - Calibrate sensor");
    Serial.println("'r' - Reset analysis");
    Serial.println("'h' - Show this help");
    
    Serial.println("\nModes:");
    Serial.println("1. Filter Comparison - Shows real-time comparison of different filters");
    Serial.println("2. Noise Analysis - Collects samples and provides statistical analysis");
    
    Serial.println("\nAnalysis Features:");
    Serial.println("- Standard deviation and variance calculation");
    Serial.println("- Signal-to-noise ratio measurement");
    Serial.println("- Peak-to-peak noise measurement");
    Serial.println("- Real-time filter performance comparison");
    
    Serial.println("\nPlace sensor at a fixed distance from a stable target");
    Serial.println("for best analysis results.\n");
}

/*
 * Understanding the Results:
 * 
 * Standard Deviation: Lower is better
 * Signal-to-Noise Ratio (SNR): Higher is better
 * Coefficient of Variation: Lower is better
 * 
 * Typical values for HC-SR04:
 * - Standard Deviation: 0.1-0.5 cm (good), >1 cm (poor)
 * - SNR: >20 (excellent), 10-20 (good), <10 (poor)
 * - CV%: <1% (excellent), 1-3% (good), >3% (poor)
 */
