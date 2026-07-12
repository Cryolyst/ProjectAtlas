#include <AccelStepper.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <Arduino.h>

// Volatile type is used so that this can be changed by one core and read by another
volatile float currentPitch = 0.0;

// P.I.D values
const int kP = 0.0;
const int kI = 0.0;
const int kD = 0.0;

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

  // 0x4A is the address because the IMU's ADO and GND are tied
  if (!IMU.begin(0x4A, Wire, BNO_INT)) {
    Serial.println("BNO085 not detected. Check wiring.");
    while(1); // Stops when the sensor fails
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

        float currentPitch = pitchRad * (180.0 / PI);

        Serial.print("Pitch: ");
        Serial.println(currentPitch, 2);
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
  // Write MAX7219 setup here

  for (;;) {
    float localPitch = currentPitch;

    if (abs(localPitch) > 15.0) {
      // Draw > < on matrices
    } else {
      // Draw O O on matrices
    }

    // Serial for Testing
    Serial.print("Pitch: ");
    Serial.println(localPitch);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  // Core 1 Balancing Task
  xTaskCreatePinnedToCore(
    CoreTask, "CoreTask", 10000, NULL, 1, &CoreTaskHandle, 1
  );

  // Core 0 UI Task
  xTaskCreatePinnedToCore(
    UITask, "UITask", 10000, NULL, 0, &CoreTaskHandle, 0
  );
}

void loop() {
  vTaskDelete(NULL);
}
