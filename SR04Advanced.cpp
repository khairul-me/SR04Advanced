/*
 * SR04Advanced.cpp - Advanced HC-SR04 Ultrasonic Sensor Library
 * 
 * Created by: Md Khairul Islam
 * Institution: Hobart and William Smith Colleges
 * Major: Double major in Robotics and Computer Science
 * 
 * This implementation file contains the core functionality of the SR04Advanced library,
 * providing sophisticated filtering and processing capabilities for improved
 * ultrasonic sensor measurements.
 * 
 * Version: 1.0.0
 * Release Date: 2024
 */

#include "SR04Advanced.h"

// Constructor
SR04Advanced::SR04Advanced(uint8_t trigger_pin, uint8_t echo_pin) {
    _trigger_pin = trigger_pin;
    _echo_pin = echo_pin;
    
    // Initialize default parameters
    _temperature = 20.0;
    _min_distance = 2.0;
    _max_distance = 400.0;
    _measurement_mode = MODE_NORMAL;
    _filter_mode = FILTER_SMART;
    
    // Filter parameters
    _alpha = 0.1;
    _window_size = 5;
    _kalman_q = 0.004;  // Process noise
    _kalman_r = 0.16;   // Measurement noise
    _kalman_p = 1.0;    // Initial estimate uncertainty
    _kalman_k = 0.0;    // Kalman gain
    _kalman_x = 0.0;    // Estimated value
    
    // Initialize state
    _last_raw_distance = 0.0;
    _last_filtered_distance = 0.0;
    _last_valid_distance = 0.0;
    _last_reading_time = 0;
    
    clearStatistics();
    
    // Debug settings
    _debug_level = DEBUG_NONE;
    _plotter_enabled = false;
    _is_calibrated = false;
    
    updateSpeedOfSound();
}

void SR04Advanced::begin(bool autoCalibrate) {
    // Configure pins
    pinMode(_trigger_pin, OUTPUT);
    pinMode(_echo_pin, INPUT);
    digitalWrite(_trigger_pin, LOW);
    
    // Initialize moving window
    for (int i = 0; i < MAX_WINDOW_SIZE; i++) {
        _distance_window[i] = 0.0;
    }
    
    if (autoCalibrate) {
        calibrate();
    }
    
    debugPrint("SR04Advanced initialized");
}

float SR04Advanced::getDistance() {
    float raw = getRawDistance();
    return applyFilters(raw);
}

float SR04Advanced::getRawDistance() {
    return performMeasurement();
}

float SR04Advanced::performMeasurement() {
    // Generate trigger pulse
    digitalWrite(_trigger_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigger_pin, LOW);
    
    // Measure echo pulse duration with timeout
    long duration = pulseIn(_echo_pin, HIGH, 30000); // 30ms timeout
    
    if (duration == 0) {
        debugPrint("Measurement timeout", DEBUG_ADVANCED);
        return _last_valid_distance;
    }
    
    float distance = calculateDistance(duration);
    _last_raw_distance = distance;
    _last_reading_time = millis();
    
    return distance;
}

float SR04Advanced::calculateDistance(long duration) {
    // d = (t × c) / 2
    // where:
    // d = distance in cm
    // t = time in seconds
    // c = speed of sound in cm/s
    return (duration * _speed_of_sound * 0.0001) / 2.0; // Result in cm
}

float SR04Advanced::getSmartDistance() {
    float result = 0.0;
    int valid_readings = 0;
    float sum = 0.0;
    
    switch (_measurement_mode) {
        case MODE_PRECISE:
            // Take multiple readings for better accuracy
            for (int i = 0; i < 5; i++) {
                float reading = performMeasurement();
                if (validateReading(reading)) {
                    sum += reading;
                    valid_readings++;
                }
                delay(10);
            }
            result = valid_readings > 0 ? sum / valid_readings : _last_valid_distance;
            break;
            
        case MODE_FAST:
            // Single quick reading
            result = performMeasurement();
            if (!validateReading(result)) {
                result = _last_valid_distance;
            }
            break;
            
        case MODE_ADAPTIVE:
            // Adaptive number of readings based on noise
            int readings = map(getNoise() * 100, 0, 100, 1, 5);
            for (int i = 0; i < readings; i++) {
                float reading = performMeasurement();
                if (validateReading(reading)) {
                    sum += reading;
                    valid_readings++;
                }
                delay(5);
            }
            result = valid_readings > 0 ? sum / valid_readings : _last_valid_distance;
            break;
            
        default: // MODE_NORMAL
            result = performMeasurement();
            if (!validateReading(result)) {
                result = _last_valid_distance;
            }
    }
    
    // Apply filtering
    result = applyFilters(result);
    
    // Update statistics
    if (validateReading(result)) {
        updateStatistics(result);
        _last_valid_distance = result;
    }
    
    // Debug output
    if (_debug_level >= DEBUG_VERBOSE) {
        Serial.print("Raw: ");
        Serial.print(_last_raw_distance);
        Serial.print(" Filtered: ");
        Serial.println(result);
    }
    
    if (_plotter_enabled) {
        sendPlotterData();
    }
    
    return result;
}

float SR04Advanced::applyFilters(float raw_distance) {
    if (!validateReading(raw_distance)) {
        return _last_valid_distance;
    }
    
    float filtered = raw_distance;
    
    switch (_filter_mode) {
        case FILTER_LOWPASS:
            filtered = lowPassFilter(filtered);
            break;
            
        case FILTER_MEDIAN:
            filtered = medianFilter(filtered);
            break;
            
        case FILTER_KALMAN:
            filtered = kalmanFilter(filtered);
            break;
            
        case FILTER_SMART:
            // Apply median filter first to remove spikes
            filtered = medianFilter(filtered);
            // Then apply Kalman filter for smooth tracking
            filtered = kalmanFilter(filtered);
            // Finally, apply adaptive low-pass filter
            float noise = getNoise();
            _alpha = map(noise * 100, 0, 100, 5, 50) / 100.0;
            filtered = lowPassFilter(filtered);
            break;
    }
    
    _last_filtered_distance = filtered;
    return filtered;
}

float SR04Advanced::lowPassFilter(float input) {
    return (_alpha * input) + ((1 - _alpha) * _last_filtered_distance);
}

float SR04Advanced::medianFilter(float input) {
    _distance_window[_window_index] = input;
    _window_index = (_window_index + 1) % _window_size;
    
    // Copy values for sorting
    float temp[MAX_WINDOW_SIZE];
    for (int i = 0; i < _window_size; i++) {
        temp[i] = _distance_window[i];
    }
    
    return getMedian(temp, _window_size);
}

float SR04Advanced::kalmanFilter(float input) {
    // Prediction
    float pred_x = _kalman_x;
    _kalman_p = _kalman_p + _kalman_q;
    
    // Update
    _kalman_k = _kalman_p / (_kalman_p + _kalman_r);
    _kalman_x = pred_x + _kalman_k * (input - pred_x);
    _kalman_p = (1 - _kalman_k) * _kalman_p;
    
    return _kalman_x;
}

void SR04Advanced::calibrate(uint16_t samples) {
    debugPrint("Starting calibration...");
    
    float sum = 0;
    float sum_squared = 0;
    float min_val = 999999;
    float max_val = -999999;
    int valid_samples = 0;
    
    // Collect samples
    for (int i = 0; i < samples && valid_samples < samples; i++) {
        float reading = performMeasurement();
        
        if (reading >= _min_distance && reading <= _max_distance) {
            sum += reading;
            sum_squared += reading * reading;
            min_val = min(min_val, reading);
            max_val = max(max_val, reading);
            valid_samples++;
        }
        
        delay(50);
    }
    
    if (valid_samples > 0) {
        _baseline_distance = sum / valid_samples;
        float variance = (sum_squared / valid_samples) - (_baseline_distance * _baseline_distance);
        _std_dev = sqrt(variance);
        _noise_level = (max_val - min_val) / _baseline_distance;
        _is_calibrated = true;
        
        debugPrint("Calibration complete:");
        debugPrint("Baseline distance: " + String(_baseline_distance));
        debugPrint("Noise level: " + String(_noise_level));
    } else {
        debugPrint("Calibration failed - no valid readings");
    }
}

bool SR04Advanced::validateReading(float distance) {
    // Check for out of range readings
    if (distance < _min_distance || distance > _max_distance) {
        debugPrint("Reading out of range: " + String(distance), DEBUG_ADVANCED);
        return false;
    }
    
    // Check for sudden large changes
    if (_last_valid_distance > 0) {
        float change = abs(distance - _last_valid_distance);
        float max_change = _last_valid_distance * 0.5; // 50% change threshold
        
        if (change > max_change) {
            debugPrint("Sudden change detected: " + String(change), DEBUG_ADVANCED);
            return false;
        }
    }
    
    return true;
}

void SR04Advanced::updateSpeedOfSound() {
    // Speed of sound (cm/s) = 331.3 + (0.606 * Temp)
    _speed_of_sound = 33130.0 + (60.6 * _temperature);
}

void SR04Advanced::setTemperature(float temp_c) {
    _temperature = temp_c;
    updateSpeedOfSound();
    debugPrint("Temperature updated to: " + String(temp_c));
}

void SR04Advanced::updateStatistics(float distance) {
    if (_reading_count == 0) {
        _min_measured = distance;
        _max_measured = distance;
    } else {
        _min_measured = min(_min_measured, distance);
        _max_measured = max(_max_measured, distance);
    }
    
    _reading_count++;
    _sum_distances += distance;
    _sum_squared_distances += (distance * distance);
}

float SR04Advanced::getAverageDistance() {
    return _reading_count > 0 ? _sum_distances / _reading_count : 0;
}

float SR04Advanced::getMinDistance() {
    return _min_measured;
}

float SR04Advanced::getMaxDistance() {
    return _max_measured;
}

float SR04Advanced::getStandardDeviation() {
    if (_reading_count < 2) return 0;
    
    float mean = getAverageDistance();
    float variance = (_sum_squared_distances / _reading_count) - (mean * mean);
    return sqrt(variance);
}

uint8_t SR04Advanced::getSignalQuality() {
    if (!_is_calibrated) return 0;
    
    float noise = getNoise();
    float quality = 100 * (1 - noise);
    return constrain(quality, 0, 100);
}

uint8_t SR04Advanced::getConfidence() {
    if (!_is_calibrated) return 0;
    
    // Calculate confidence based on multiple factors
    float confidence = 100.0;
    
    // Factor 1: Distance from baseline
    float baseline_diff = abs(_last_filtered_distance - _baseline_distance);
    float baseline_factor = 1.0 - constrain(baseline_diff / (_baseline_distance * 0.5), 0, 1);
    
    // Factor 2: Noise level
    float noise_factor = 1.0 - getNoise();
    
    // Factor 3: Reading stability
    float stability_factor = 1.0;
    if (_reading_count > 1) {
        float std_dev = getStandardDeviation();
        stability_factor = 1.0 - constrain(std_dev / (_baseline_distance * 0.1), 0, 1);
    }
    
    confidence *= baseline_factor * noise_factor * stability_factor;
    return constrain(confidence, 0, 100);
}

float SR04Advanced::getNoise() {
    if (!_is_calibrated) return 1.0;
    
    float current_noise = abs(_last_raw_distance - _last_filtered_distance) / _last_filtered_distance;
    return constrain(current_noise, 0, 1);
}

void SR04Advanced::clearStatistics() {
    _sum_distances = 0;
    _sum_squared_distances = 0;
    _min_measured = 0;
    _max_measured = 0;
    _reading_count = 0;
}

void SR04Advanced::setDebugLevel(uint8_t level) {
    _debug_level = level;
    debugPrint("Debug level set to: " + String(level));
}

void SR04Advanced::enablePlotter(bool enable) {
    _plotter_enabled = enable;
    if (enable) {
        Serial.println("raw,filtered,noise*10,confidence");
    }
}

void SR04Advanced::sendPlotterData() {
    if (!_plotter_enabled) return;
    
    Serial.print(_last_raw_distance);
    Serial.print(",");
    Serial.print(_last_filtered_distance);
    Serial.print(",");
    Serial.print(getNoise() * 10); // Scale noise for visibility
    Serial.print(",");
    Serial.println(getConfidence());
}

void SR04Advanced::printDebugInfo() {
    Serial.println("\n=== SR04Advanced Debug Info ===");
    Serial.println("Last Raw Distance: " + String(_last_raw_distance) + " cm");
    Serial.println("Last Filtered Distance: " + String(_last_filtered_distance) + " cm");
    Serial.println("Signal Quality: " + String(getSignalQuality()) + "%");
    Serial.println("Confidence: " + String(getConfidence()) + "%");
    Serial.println("Noise Level: " + String(getNoise()));
    Serial.println("Average Distance: " + String(getAverageDistance()) + " cm");
    Serial.println("Standard Deviation: " + String(getStandardDeviation()));
    Serial.println("Temperature: " + String(_temperature) + "°C");
    Serial.println("Speed of Sound: " + String(_speed_of_sound) + " cm/s");
    Serial.println("Readings Count: " + String(_reading_count));
    Serial.println("============================");
}

String SR04Advanced::getStatusJson() {
    String json = "{";
    json += "\"raw_distance\":" + String(_last_raw_distance) + ",";
    json += "\"filtered_distance\":" + String(_last_filtered_distance) + ",";
    json += "\"signal_quality\":" + String(getSignalQuality()) + ",";
    json += "\"confidence\":" + String(getConfidence()) + ",";
    json += "\"noise_level\":" + String(getNoise()) + ",";
    json += "\"readings_count\":" + String(_reading_count) + ",";
    json += "\"average_distance\":" + String(getAverageDistance()) + ",";
    json += "\"min_distance\":" + String(_min_measured) + ",";
    json += "\"max_distance\":" + String(_max_measured) + ",";
    json += "\"std_deviation\":" + String(getStandardDeviation()) + ",";
    json += "\"measurement_mode\":\"" + getMeasurementModeName() + "\",";
    json += "\"temperature\":" + String(_temperature) + ",";
    json += "\"is_calibrated\":" + String(_is_calibrated ? "true" : "false");
    json += "}";
    return json;
}

String SR04Advanced::getMeasurementModeName() {
    switch (_measurement_mode) {
        case MODE_PRECISE: return "PRECISE";
        case MODE_FAST: return "FAST";
        case MODE_ADAPTIVE: return "ADAPTIVE";
        default: return "NORMAL";
    }
}

void SR04Advanced::debugPrint(String message, uint8_t required_level) {
    if (_debug_level >= required_level) {
        Serial.println("SR04Advanced: " + message);
    }
}

float SR04Advanced::getMedian(float* values, int size) {
    bubbleSort(values, size);
    return values[size / 2];
}

void SR04Advanced::bubbleSort(float* values, int size) {
    for (int i = 0; i < size - 1; i++) {
        for (int j = 0; j < size - i - 1; j++) {
            if (values[j] > values[j + 1]) {
                float temp = values[j];
                values[j] = values[j + 1];
                values[j + 1] = temp;
            }
        }
    }
}

void SR04Advanced::setFilterMode(uint8_t mode) {
    _filter_mode = mode;
    debugPrint("Filter mode set to: " + String(mode));
}

void SR04Advanced::setMeasurementMode(uint8_t mode) {
    _measurement_mode = mode;
    debugPrint("Measurement mode set to: " + getMeasurementModeName());
}

void SR04Advanced::setFilterParams(float alpha, int window_size) {
    _alpha = constrain(alpha, 0.0, 1.0);
    _window_size = constrain(window_size, 1, MAX_WINDOW_SIZE);
    debugPrint("Filter parameters updated - Alpha: " + String(_alpha) + " Window: " + String(_window_size));
}

void SR04Advanced::setThresholds(float min_dist, float max_dist) {
    _min_distance = max(0.0, min_dist);
    _max_distance = min(500.0, max_dist);
    debugPrint("Thresholds updated - Min: " + String(_min_distance) + " Max: " + String(_max_distance));
}

bool SR04Advanced::isMeasurementValid() {
    return validateReading(_last_raw_distance);
}

void SR04Advanced::resetStatistics() {
    clearStatistics();
    debugPrint("Statistics reset");
}

bool SR04Advanced::isTimeout(unsigned long duration) {
    return duration > 30000; // 30ms timeout
}