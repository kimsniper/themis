# Project THEMIS
**Tilt Handling via Estimation and Motion-Integrated Stabilization**

THEMIS is a self-stabilizing embedded control system that uses an **IMU (MPU6050)** and a **PID-controlled servo motor** to correct and maintain alignment. The system continuously monitors tilt using fused data from accelerometer and gyroscope via a **complementary filter**, then adjusts the servo angle to return to a **zero-degree reference**.

---

## Project Structure

```text
THEMIS/
├── main/
│ ├── M CMakeLists.txt
│ ├── Kconfig.projbuild
│ ├── main.cpp
│ ├── mpu6050_hal.cpp
│ ├── mpu6050_hal.hpp
│ ├── mpu6050_bypes.hpp
│ ├── mpu6050.cpp
│ ├── mpu6050.hpp
│ ├── pwm_hal.cpp
│ ├── pwm_hal.hpp
│ ├── pwm_bypes.hpp
│ ├── pwm.cpp
│ ├── pwm.hpp
│ ├── servo.cpp
│ └── servo.hpp
├── .gitignore
├── CMakeLists.txt
├── LICENSE.txt
├── plot_data.py
└── README.md
```

---

## Features

- Real-time **roll estimation** via **sensor fusion**
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
| **MPU6050**       | Inertial sensor for roll sensing (I2C)    |
| **Complementary Filter** | Fusion of accelerometer and gyroscope data |
| **PID Controller** | Controls servo to return roll to neutral |
| **Servo Motor**   | Output actuator for stabilization          |
| **ESP32 + ESP-IDF** | MCU platform using multithreaded C++    |

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
      |     Filter        |   θ_est  |        (roll)      |
      +------------------+           +----------------------+

```

---

## ESP-IDF Setup

### Navigate to Themis directory
```bash
cd /path/to/your/themis
```

### Build & flash
```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Verify
```bash
idf.py --version  # Project is based on v5.4.2
```

---

# Data Visualization UI (`plot_data.py`)

A real-time Python-based UI is provided to:

- Visualize roll, accelerometer, gyroscope, and servo angle data  
- Tune PID gains and complementary filter alpha via sliders  
- Toggle PID control and test manual servo angles

### Requirements

Install dependencies:

```bash
pip install matplotlib pyserial
```

### Run the UI

```bash
python3 plot_data.py
```