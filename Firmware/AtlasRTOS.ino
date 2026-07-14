#include <AccelStepper.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <Arduino.h>
#include <SPI.h>
#include <MD_MAX72xx.h>
#include <esp_task_wdt.h>

//--- UI Related Pins and Variables Beginning ---
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 2

// HSPI Pins
#define HSPI_MOSI 13 // DIN
#define HSPI_MISO 15
#define HSPI_SCLK 14 // CLK
#define HSPI_CS   12 // CS

MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, HSPI_MOSI, HSPI_SCLK, HSPI_CS, MAX_DEVICES);

// All the eye shapes
const byte openEye[8] = {
  0b00000000,
  0b00111100,
  0b01111110,
  0b11111111,
  0b11111111,
  0b01111110,
  0b00111100,
  0b00000000
};

const byte leftEye[8] = {
  0b00000000,
  0b00000000,
  0b10000001,
  0b11000011,
  0b01100110,
  0b00111100,
  0b00011000,
  0b00000000
};

const byte rightEye[8] = {
  0b00000000,
  0b00011000,
  0b00111100,
  0b01100110,
  0b11000011,
  0b10000001,
  0b00000000,
  0b00000000
};

// --- UI Related Pins and Variables End ---

// Volatile type is used so that this can be changed by one core and read by another
volatile float currentPitch = 0.0;

// P.I.D values
const float kP = 200.0;
const float kI = 0.0;
const float kD = 2.0;

float targetPitch = 0.0; // PLEASE CHANGE WHEN THE PCB ARRIVES
float errorSum = 0.0;
float lastError = 0.0;

// IMU setup
BNO080 IMU;
#define BNO_INT 23
#define BNO_RST 19

// Motor Pins
const int left_stepPin = 33;
const int left_dirPin = 32;
const int right_stepPin = 26;
const int right_dirPin = 25;

// Master Enable Pin for both TMC2209s
const int enablePin = 27;

/* Motors will be running in 1/16 microstepping for smoothness
  MS1 and MS2 on the TMC2209 will be powered by ESP32 3V3
  Also remember that the coils on the NEMA 17 motors are NOT next to each other
  Why is it designed that way), the coils are one pin apart.
*/

// Motor Initialization
AccelStepper leftMotor(1, left_stepPin, left_dirPin);
AccelStepper rightMotor(1, right_stepPin, right_dirPin);

// Two different Tasks for FreeRTOS
TaskHandle_t CoreTaskHandle;
TaskHandle_t UITaskHandle;

void drawEyes(int deviceIndex, const byte sprite[]) {
  for (int row = 0; row < 8; row++) {
    mx.setRow(deviceIndex, row, sprite[row]); 
  }
}

void CoreTask(void *pvParameters) {

  // Enable TMC2209s
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  // BNO085 Reset Sequence
  pinMode(BNO_RST, OUTPUT);
  digitalWrite(BNO_RST, LOW);
  vTaskDelay(10 / portTICK_PERIOD_MS); // FreeRTOS friendly delay
  digitalWrite(BNO_RST, HIGH);
  vTaskDelay(50 / portTICK_PERIOD_MS);

  // I2C Setup
  Wire.begin();
  Wire.setClock(100000);
  Serial.println("Initializing BNO085...");

  // 0x4A is the address because the IMU's ADO and GND are tied and its decimal is 74
  if (!IMU.begin(74, Wire, BNO_INT)) {
    Serial.println("BNO085 not detected. Check wiring.");
    while(1) { vTaskDelay(10); }  // Stops when the sensor fails
  }
  Serial.println("BNO085 initialized successfully!");

  // We are going to be using the Game Rotation Vector as using the magnetometer would mess up the
  // calculations as the inductance from the motors would interfere with the IMU
  // 5ms = 200Hz
  IMU.enableGameRotationVector(5);

  leftMotor.setMaxSpeed(20000);
  rightMotor.setMaxSpeed(200000);

  // Timing variables for the PID calculation
  unsigned long lastPIDTime = micros();
  const unsigned long PIDInterval = 5000; // 5000 us = 5ms = 200Hz

  esp_task_wdt_add(NULL);

  // Task L O O P Phase
  for (;;) {
    leftMotor.runSpeed();
    rightMotor.runSpeed();

    // Watchdog timer
    esp_task_wdt_reset();

    if (digitalRead(BNO_INT) == LOW) {
      if (IMU.dataAvailable() == true) {
        float pitchRad = IMU.getPitch();
        currentPitch = pitchRad * (180.0 / PI);
      }
    }

    // Check to see if 5ms has passed on the PID loop
    if (micros() - lastPIDTime >= PIDInterval) {
      lastPIDTime = micros();

        // --------- PID ---------
        float error = targetPitch - currentPitch;

        // Integral Calculation
        errorSum += error;
        errorSum = constrain(errorSum, -2000, 2000);

        // Derivative Calculation
        float dError = error - lastError;
        lastError = error;

        // PID equation
        float PID = (kP * error) + (kI * errorSum) + (kD * dError); 

        leftMotor.setSpeed(PID);
        rightMotor.setSpeed(-PID);
    }
  }
}

void UITask(void *pvParameters) {

  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, 5);
  mx.clear();

  for (;;) {
    float localPitch = currentPitch;

    if (abs(localPitch) > 15.0) {
      drawEyes(1, leftEye);
      drawEyes(0, rightEye);
    } else {
      drawEyes(1, openEye);
      drawEyes(0, openEye);
    }

    // Serial for Testing
    Serial.print("Pitch: ");
    Serial.println(localPitch);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  // Core 1 Balancing Task (Priority 1 - High)
  xTaskCreatePinnedToCore(
    CoreTask, "CoreTask", 10000, NULL, 1, &CoreTaskHandle, 1
  );

  // Core 0 UI Task (Priority 0 - Low)
  xTaskCreatePinnedToCore(
    UITask, "UITask", 10000, NULL, 0, &UITaskHandle, 0
  );
}

void loop() {
  vTaskDelete(NULL);
}

// Note to self, motors A+ -> A2, B+ -> B2