/*
Project: SitSense
Author: Mukesh Sankhla
Website: https://www.makerbrains.com
GitHub: https://github.com/MukeshSankhla
Social Media: Instagram @makerbrains_official.diy
*/

#include <BleKeyboard.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define Buzzer 6
const int frequency = 1000;
const int beepDuration = 50;
const int pauseDuration = 1000;

BleKeyboard bleKeyboard;
Adafruit_MPU6050 mpu;

// Thresholds for posture detection
float baselineX, baselineY, baselineZ;
bool postureInitialized = false;
bool isLocked = false;  // Start in locked state

unsigned long incorrectPostureStartTime = 0;
const unsigned long lockDelay = 10000; // 10 Sec minutes in milliseconds

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work and MPU setup...");

  // Initialize BLE Keyboard
  bleKeyboard.begin();

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(Buzzer, OUTPUT);
}

void initializePosture() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Set the baseline for posture
  baselineX = a.acceleration.x;
  baselineY = a.acceleration.y;
  baselineZ = a.acceleration.z;
  postureInitialized = true;

  Serial.println("Baseline posture initialized:");
  Serial.print("X: "); Serial.print(baselineX);
  Serial.print(" Y: "); Serial.print(baselineY);
  Serial.print(" Z: "); Serial.println(baselineZ);
}

bool isPostureCorrect() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Check if current posture is within a threshold of the baseline
  float threshold = 1.0;  // Adjust sensitivity
  return (abs(a.acceleration.x - baselineX) < threshold &&
          abs(a.acceleration.y - baselineY) < threshold &&
          abs(a.acceleration.z - baselineZ) < threshold);
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
    bleKeyboard.press(KEY_NUM_1);
    delay(100);
    bleKeyboard.releaseAll();
    bleKeyboard.press(KEY_NUM_2);
    delay(100);
    bleKeyboard.releaseAll();
    bleKeyboard.press(KEY_NUM_3);
    delay(100);
    bleKeyboard.releaseAll();
    bleKeyboard.press(KEY_NUM_4);
    delay(100);
    bleKeyboard.releaseAll();         // Release all keys after unlocking
    isLocked = false;  // Clear lock flag after unlocking
  }
}

void loop() {
  if (!postureInitialized) {
    initializePosture();  // Initialize baseline posture once
  }

  if (isPostureCorrect()) {
    Serial.println("Posture is correct.");
    digitalWrite(Buzzer, LOW);
    incorrectPostureStartTime = 0; // Reset timer if posture is corrected
    if (isLocked) {
      unlockScreen();  // Unlock if posture is corrected
    }
  } else {
    Serial.println("Incorrect posture detected.");
    
    tone(Buzzer, frequency, beepDuration);
    delay(beepDuration);
    noTone(Buzzer);
    delay(pauseDuration);

    // Start or reset the timer for incorrect posture
    if (incorrectPostureStartTime == 0) {
      incorrectPostureStartTime = millis();
    } else if (millis() - incorrectPostureStartTime >= lockDelay) {
      lockScreen();  // Lock if incorrect posture persists for 2 minutes
    }
  }
}
