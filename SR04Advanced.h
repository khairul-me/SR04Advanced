/*
 * SR04Advanced.h - Advanced HC-SR04 Ultrasonic Sensor Library
 * 
 * Created by: Md Khairul Islam
 * Institution: Hobart and William Smith Colleges
 * Major: Double major in Robotics and Computer Science
 * 
 * This library provides advanced filtering and processing capabilities
 * for the HC-SR04 ultrasonic sensor, including smart filtering,
 * temperature compensation, and diagnostic features.
 * 
 * Version: 1.0.0
 * Release Date: 2024
 */

#ifndef SR04Advanced_h
#define SR04Advanced_h

#include "Arduino.h"

// Debug levels for granular control over serial output
#define DEBUG_NONE 0
#define DEBUG_BASIC 1
#define DEBUG_ADVANCED 2
#define DEBUG_VERBOSE 3

// Filter modes for different filtering strategies
#define FILTER_NONE 0
#define FILTER_LOWPASS 1
#define FILTER_MEDIAN 2
#define FILTER_KALMAN 3
#define FILTER_SMART 4

class SR04Advanced {
public:
    // Constructor & basic setup
    SR04Advanced(uint8_t trigger_pin, uint8_t echo_pin);
    void begin(bool autoCalibrate = true);
    
    // Core measurement functions
    float getDistance();              // Basic filtered measurement
    float getRawDistance();           // Raw unfiltered measurement
    float getSmartDistance();         // Advanced filtered measurement
    
    // Configuration methods
    void setDebugLevel(uint8_t level);
    void setFilterMode(uint8_t mode);
    void setMeasurementMode(uint8_t mode);
    void setFilterParams(float alpha = 0.1, int window_size = 5);
    void setThresholds(float min_dist = 2.0, float max_dist = 400.0);
    void setTemperature(float temp_c = 20.0);
    
    // Calibration and diagnostics
    void calibrate(uint16_t samples = 20);
    bool isMeasurementValid();
    uint8_t getSignalQuality();
    uint8_t getConfidence();
    float getNoise();
    
    // Statistical methods
    float getAverageDistance();
    float getMinDistance();
    float getMaxDistance();
    float getStandardDeviation();
    void resetStatistics();
    
    // Debug and visualization
    void enablePlotter(bool enable = true);
    void printDebugInfo();
    void sendPlotterData();
    String getStatusJson();
    
    // Measurement modes
    static const uint8_t MODE_NORMAL = 0;
    static const uint8_t MODE_PRECISE = 1;
    static const uint8_t MODE_FAST = 2;
    static const uint8_t MODE_ADAPTIVE = 3;

private:
    // Pin configuration
    uint8_t _trigger_pin;
    uint8_t _echo_pin;
    
    // Measurement parameters
    float _temperature;               // Temperature in Celsius
    float _speed_of_sound;           // Calculated speed of sound
    float _min_distance;             // Minimum valid distance
    float _max_distance;             // Maximum valid distance
    uint8_t _measurement_mode;       // Current measurement mode
    uint8_t _filter_mode;           // Current filter mode
    
    // Filter parameters
    float _alpha;                    // Low-pass filter coefficient
    int _window_size;                // Moving average window size
    float _kalman_q;                 // Process noise covariance
    float _kalman_r;                 // Measurement noise covariance
    float _kalman_p;                 // Estimation error covariance
    float _kalman_k;                 // Kalman gain
    float _kalman_x;                 // Estimated value
    
    // State variables
    float _last_raw_distance;
    float _last_filtered_distance;
    float _last_valid_distance;
    unsigned long _last_reading_time;
    
    // Statistical variables
    float _sum_distances;
    float _sum_squared_distances;
    float _min_measured;
    float _max_measured;
    uint32_t _reading_count;
    
    // Moving window for median filter
    static const int MAX_WINDOW_SIZE = 10;
    float _distance_window[MAX_WINDOW_SIZE];
    int _window_index;
    
    // Calibration variables
    float _noise_level;
    float _std_dev;
    float _baseline_distance;
    bool _is_calibrated;
    
    // Debug settings
    uint8_t _debug_level;
    bool _plotter_enabled;
    
    // Internal methods
    float performMeasurement();
    float calculateDistance(long duration);
    float applyFilters(float raw_distance);
    float lowPassFilter(float input);
    float medianFilter(float input);
    float kalmanFilter(float input);
    float smartFilter(float input);
    bool validateReading(float distance);
    void updateStatistics(float distance);
    void updateSpeedOfSound();
    String getMeasurementModeName();
    void clearStatistics();
    
    // Utility methods
    float getMedian(float* values, int size);
    void bubbleSort(float* values, int size);
    bool isTimeout(unsigned long duration);
    void debugPrint(String message, uint8_t required_level = DEBUG_BASIC);
};

#endif