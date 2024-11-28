/*
Project: SitSense
Author: Mukesh Sankhla
Website: https://www.makerbrains.com
GitHub: https://github.com/MukeshSankhla
Social Media: Instagram @makerbrains_official
*/

#include <BleKeyboard.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SitSense_inferencing.h> // Edge Impulse Inferencing SDK

#define Buzzer 6
const int frequency = 1000;
const int beepDuration = 50;
const int pauseDuration = 1000;

BleKeyboard bleKeyboard;
Adafruit_MPU6050 mpu;

// Edge Impulse constants
#define EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME 3 // 3-axis accelerometer data
#define EI_CLASSIFIER_RAW_SAMPLE_COUNT 50    // Number of samples required

// Buffer for collecting data
float raw_features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 20; // Adjusted to match model's sampling rate (50 Hz)

// Variables for locking and unlocking PC
bool isLocked = false; // Start in locked state
unsigned long incorrectPostureStartTime = 0;
const unsigned long lockDelay = 5000; // 5 Sec in milliseconds

// Helper for Edge Impulse result
float classify_posture() {
    signal_t signal;
    int err = numpy::signal_from_buffer(raw_features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        Serial.printf("Failed to create signal (%d)\n", err);
        return -1;
    }

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR ei_err = run_classifier(&signal, &result, false);
    if (ei_err != EI_IMPULSE_OK) {
        Serial.printf("Classifier error: %d\n", ei_err);
        return -1;
    }

    // Print results
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        Serial.print(result.classification[ix].label);
        Serial.print(": ");
        Serial.println(result.classification[ix].value, 5);
    }
    Serial.println();

    // Return "Bad" posture classification value
    return result.classification[0].value; // Assuming "Bad" is the first label
}

void lockScreen() {
    if (bleKeyboard.isConnected() && !isLocked) {
        Serial.println("Locking screen (Win+L)...");
        bleKeyboard.press(KEY_LEFT_GUI);  // Windows (GUI) key
        bleKeyboard.press('l');           // 'L' key
        delay(100);                       // Small delay for action
        bleKeyboard.releaseAll();         // Release keys after locking
        isLocked = true;  // Set lock flag
    }
}

void unlockScreen() {
    if (bleKeyboard.isConnected() && isLocked) {
        Serial.println("Unlocking screen with PIN...");
        bleKeyboard.write(KEY_HOME);
        delay(100);
        bleKeyboard.releaseAll();
        delay(1000);
        bleKeyboard.press(KEY_NUM_2);
        delay(100);
        bleKeyboard.releaseAll();
        bleKeyboard.press(KEY_NUM_8);
        delay(100);
        bleKeyboard.releaseAll();
        bleKeyboard.press(KEY_NUM_3);
        delay(100);
        bleKeyboard.releaseAll();
        bleKeyboard.press(KEY_NUM_0);
        delay(100);
        bleKeyboard.releaseAll();         // Release all keys after unlocking
        isLocked = false;  // Clear lock flag after unlocking
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    // Initialize BLE Keyboard
    bleKeyboard.begin();

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip. Check connections.");
        while (1) {
            delay(10);
        }
    }

    // Configure MPU6050
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Configure buzzer
    pinMode(Buzzer, OUTPUT);

    // Welcome message
    Serial.println("MPU6050 initialized. Starting posture detection with Edge Impulse.");
}

void loop() {
    // Collect data samples
    static size_t sample_idx = 0;
    unsigned long currentTime = millis();

    if (currentTime - lastSampleTime >= sampleInterval) {
        lastSampleTime = currentTime;

        // Get new sensor event
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Add accelerometer data to raw_features buffer
        raw_features[sample_idx++] = a.acceleration.x;
        raw_features[sample_idx++] = a.acceleration.y;
        raw_features[sample_idx++] = a.acceleration.z;

        // If enough samples are collected, run inference
        if (sample_idx >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            Serial.println("Running inference...");

            float badScore = classify_posture();

            if (badScore >= 0.5) { // Threshold for bad posture
                Serial.println("Bad posture detected.");
                tone(Buzzer, frequency, beepDuration);
                delay(beepDuration);
                noTone(Buzzer);
                delay(pauseDuration);

                // Start or reset the timer for incorrect posture
                if (incorrectPostureStartTime == 0) {
                    incorrectPostureStartTime = millis();
                } else if (millis() - incorrectPostureStartTime >= lockDelay) {
                    lockScreen();  // Lock if incorrect posture persists
                }
            } else {
                Serial.println("Good posture detected.");
                digitalWrite(Buzzer, LOW);
                incorrectPostureStartTime = 0; // Reset timer if posture is corrected
                if (isLocked) {
                    unlockScreen();  // Unlock if posture is corrected
                }
            }
            // Reset buffer
            sample_idx = 0;
        }
    }
}
