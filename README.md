# ProjectAtlas
An ESP32-controlled self-balancing robot utilizing MPU6050 sensor, real-time PID control, custom PCBS and 3d printed parts and NEMA 17 stepper motors.

# Overview
Project Atlast is an exploration into embedded real-time control systems, sensor fusion and robotics. The robot uses a complementary filter to fuse accelerometer and gyroscope data from an MPU6050. This clean angle data feeds into a high-speed PID control loop to drive two NEMA 17 stepper motors, which would keep the chassis perfectly balanced. The chassis will also be 3d printed.

# Hardware Components
1x ESP32 DOIT DevKit V1
1x GY521 MPU6050
2x NEMA 17 Stepper Motors (1.8° step angle)
2x BIGTREETECH TMC2209 V1.3
1x 3S (11.1V) 1500mAh LiPo Battery
1x MP1584EN Step-Down Buck Converter (11.1V to 5V)
2x SSD1306 OLED Display (For aesthetics)
3D Printed Chassis
Custom PCB

# Software Architecture
**Sensor Fusion:** Implements a high-speed Complementary Filter (98% Gyroscope / 2% Accelerometer) to eliminate mechanical vibration noise and gyroscope drift.
**Non-Blocking Timing:** Built using microsecond-precision timing (micros()) to maintain a strict, stutter-free control loop (e.g., 200Hz).
**Dual-Core Processing (Planned):** Utilizes FreeRTOS to pin the mission-critical PID balancing loop to Core 1, while Core 0 handles asynchronous tasks like serial telemetry and OLED UI updates.
