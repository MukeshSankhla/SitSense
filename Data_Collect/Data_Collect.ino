/*
Project: SitSense
Author: Mukesh Sankhla
Website: https://www.makerbrains.com
GitHub: https://github.com/MukeshSankhla
Social Media: Instagram @makerbrains_official
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Initialize the MPU6050 sensor
Adafruit_MPU6050 mpu;

// Sampling interval (in milliseconds)
unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 50; // 20 Hz

void setup() {
    // Initialize Serial for data output to Edge Impulse CLI
    Serial.begin(115200);
    while (!Serial) {
        delay(10); // Wait for Serial to initialize
    }

    // Initialize I2C communication
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip. Check connections.");
        while (1) {
            delay(10);
        }
    }

    // Configure the MPU6050 sensor
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.println("MPU6050 initialized.");
    Serial.println("Ready to forward data. Connect Edge Impulse CLI.");
}

void loop() {
    // Check if it's time to sample
    unsigned long currentTime = millis();
    if (currentTime - lastSampleTime >= sampleInterval) {
        lastSampleTime = currentTime;

        // Get new sensor events
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Format and send data to Serial
        Serial.print(a.acceleration.x);
        Serial.print(",");
        Serial.print(a.acceleration.y);
        Serial.print(",");
        Serial.println(a.acceleration.z);
    }
}
