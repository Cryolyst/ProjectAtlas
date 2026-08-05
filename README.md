# Project Atlas: Self-Balancing Inverted-Pendulum Robot

## Overview
Project Atlas is a custom-designed, two-wheeled self-balancing robot. This project serves as a comprehensive exploration of embedded systems design, combining a custom 4-layer PCB, real-time control loops, and 3D-printed mechanical enclosures. The system leverages an ESP32 microcontroller to process IMU data and dynamically drive NEMA 17 stepper motors to maintain an upright position.

## System Architecture

### Hardware & Electronics
*   **Microcontroller:** ESP32 (running FreeRTOS for task scheduling)
*   **Sensor:** BNO085 IMU (I2C communication)
*   **Actuation:** 2x NEMA 17 Stepper Motors driven by TMC2209 stepper drivers
*   **Display:** 16x16 Matrix Display for real-time status and diagnostics
*   **Power:** 3S (11.1V) LiPo Battery with LM2596 step-down conversion for logic level
*   **PCB:** Custom 4-layer board (designed in KiCad) featuring isolated ground planes and optimized power delivery for the motor drivers.

### Mechanical Design
*   Custom chassis and component enclosures modeled in Autodesk Fusion 360.
*   Designed with tight tolerances for a low center of gravity to optimize the PID control loop response.

## Repository Structure
*   `/Firmware` - ESP32 C/C++ source code, FreeRTOS tasks, and PID control logic.
*   `/Hardware` - KiCad project files, schematics (PDF), and Gerber files for the 4-layer PCB.
*   `/Mechanical` - STL files and Fusion 360 step files for the chassis.
*   `/Docs` - System block diagrams, BOM (Bill of Materials), and assembly photos.

## Current Status
*   **Phase 1:** Breadboard prototyping and base firmware structure - *Complete*
*   **Phase 2:** Custom 4-layer PCB design and manufacturing - *In Progress (Awaiting Delivery)*
*   **Phase 3:** Mechanical 3D printing and physical assembly - *In Progress (Awaiting Delivery)*
*   **Phase 4:** PID tuning and control loop optimization - *Pending*
