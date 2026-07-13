#include <AccelStepper.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <Arduino.h>
#include <SPI.h>
#include <MD_MAX72xx.h>

//--- UI Related Pins and Variables Beginning ---
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW
#define MAX_DEVICES 2

// HSPI Pins
#define HSPI_MOSI 13
#define HSPI_MISO 15
#define HSPI_SCLK 14
#define HSPI_CS   12

MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, HSPI_CS, MAX_DEVICES);

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
const float kP = 0.0;
const float kI = 0.0;
const float kD = 0.0;

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

  // TickType_t defines the exact timing interval
  // 5ms = 200Hz this ensures that the loop runs exactly every 5ms
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;
  xLastWakeTime = xTaskGetTickCount();

  // Task L O O P Phase
  for (;;) {
    if (digitalRead(BNO_INT) == LOW) {
      if (IMU.dataAvailable() == true) {
        float pitchRad = IMU.getPitch();

        currentPitch = pitchRad * (180.0 / PI);
      }
    }
    // 2. Calculate PID
    // 3. Step motors
    
    // Update the global variable so Core 0 can see it
    // currentPitch = myIMU.getPitch() * (180.0 / PI); 

    // This command tells FreeRTOS: "I am done. Pause this task and 
    // wake me up exactly 5ms from the last time I woke up."
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
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
