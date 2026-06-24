#include <Wire.h>
#include <U8x8lib.h>
#include <math.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
unsigned long last_time = 0;
float pitch_angle = 0.0;

// Calibration Offset just in case
float gyro_X_Offset = 0.0;

void moduleScanner(){
  byte error;   
  int devices = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      devices++;
    }
  }
  
  if (devices == 0) {
    Serial.println("No devices found");
  } else {
    Serial.println("Scan completed.. " + String(devices) + " devices found");
  }

  delay(2000);  
}

void Gyro_setup(){
  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // 
  Wire.write(0); 
  Wire.endTransmission(true);
}
void setup() {
  Serial.begin(115200);
  Gyro_setup();
  delay(100);
  last_time = micros();
}

void loop() {

  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0;
  last_time = current_time;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 7*2, true); // Request a total of 7*2 = 14 registers

  int16_t raw_accele_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  int16_t raw_accele_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  int16_t raw_accele_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  int16_t raw_temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  int16_t raw_gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  int16_t raw_gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  int16_t raw_gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  float accelerometer_Y_g = raw_accele_y / 16384.0; // For 2g scale range
  float accelerometer_Z_g = raw_accele_z / 16384.0; // For 2g scale range
  
  float gyro_rate_X = (raw_gyro_x / 131.0) - gyro_X_Offset; // 250 degree scale range

  float accel_angle = atan2(accelerometer_Y_g, accelerometer_Z_g) * 180.0 / PI; // Gets the angle from accelerometer

  // I m p o r t a n t
  // 98% Gyro and 2% Accelerometer is used
  // Change if needed in the future tests
  pitch_angle = 0.98 * (pitch_angle + gyro_rate_X * dt) + 0.02 * accel_angle;
  
  // Serial Plotter
  Serial.print("Gyro:");
  Serial.print(gyro_rate_X);
  Serial.print(",");
  Serial.print("Accel:");
  Serial.print(accel_angle);
  Serial.print(",");
  Serial.print("PitchAngle:");
  Serial.println(pitch_angle);

  // Loop stabilizing
  while (micros() - current_time < 5000) {
  }
}

