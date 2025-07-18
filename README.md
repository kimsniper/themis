# Project THEMIS
**Tilt Handling via Estimation and Motion-Integrated Stabilization**

THEMIS is a self-stabilizing embedded control system that uses an **IMU (MPU6050)** and a **PID-controlled servo motor** to correct and maintain pitch alignment. The system continuously monitors tilt using fused data from accelerometer and gyroscope via a **complementary filter**, then adjusts the servo angle to return to a **zero-degree reference**.

Inspired by **Themis**, the Greek goddess of divine order and balance, this project reflects a disciplined and reactive feedback system — ensuring precise stabilization even in the presence of noise, drift, or sudden disturbance.

---

## ⚙Features

- Real-time **pitch estimation** via **sensor fusion**
- Uses **MPU6050 IMU** (accelerometer + gyroscope)
- Implements **complementary filter** for reliable angle measurement
- **PID control loop** to minimize tilt error
- Actuated with a standard **servo motor** (0–180°)
- Threaded **C++ implementation** on ESP32 (ESP-IDF or POSIX pthreads)
- Modular design (sensor logic, filtering, control layers)

---

## Technical Stack

| Component         | Description                                |
|------------------|--------------------------------------------|
| **MPU6050**       | Inertial sensor for pitch sensing (I2C)    |
| **Complementary Filter** | Fusion of accelerometer and gyroscope data |
| **PID Controller** | Controls servo to return pitch to neutral |
| **Servo Motor**   | Output actuator for stabilization          |
| **ESP32 + ESP-IDF** | MCU platform using multithreaded C++    |
| **C++ Modules**   | `imu.hpp`, `servo.hpp`, `pitch_control.cpp` |

---

## Control Architecture

```text
      +------------------+           +--------------------+
      |   MPU6050 IMU    |           |  Servo Motor (PWM) |
      |  Accel + Gyro    |           |     (0–180°)       |
      +--------+---------+           +----------+---------+
               |                                ^
               v                                |
      +------------------+           +----------+---------+
      | Complementary     | -------> |      PID Controller |
      |     Filter        |   θ_est  |        (pitch)      |
      +------------------+           +----------------------+
