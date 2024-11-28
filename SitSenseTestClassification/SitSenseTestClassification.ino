#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SitSense_inferencing.h> // Edge Impulse Inferencing SDK

// Initialize the MPU6050 sensor
Adafruit_MPU6050 mpu;

// Edge Impulse constants
#define EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME 3 // 3-axis accelerometer data
#define EI_CLASSIFIER_RAW_SAMPLE_COUNT 50    // Number of samples required

// Buffer for collecting data
float raw_features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 20; // Adjusted to match model's sampling rate (50 Hz)

// Helper for Edge Impulse result
void print_inference_results(ei_impulse_result_t result) {
    // Print classification results
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        Serial.print(result.classification[ix].label);
        Serial.print(": ");
        Serial.println(result.classification[ix].value, 5);
    }
    Serial.println();
}

void setup() {
    // Initialize Serial
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

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

    // Welcome message
    Serial.println("MPU6050 initialized. Starting Edge Impulse inference.");
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

            // Perform inference
            signal_t signal;
            int err = numpy::signal_from_buffer(raw_features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
            if (err != 0) {
                Serial.printf("Failed to create signal (%d)\n", err);
                return;
            }

            ei_impulse_result_t result = { 0 };
            EI_IMPULSE_ERROR ei_err = run_classifier(&signal, &result, false);
            if (ei_err != EI_IMPULSE_OK) {
                Serial.printf("Classifier error: %d\n", ei_err);
                return;
            }

            // Print results
            print_inference_results(result);

            // Reset buffer
            sample_idx = 0;
        }
    }
}
