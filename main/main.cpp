/*
 * Copyright (c) 2025, Mezael Docoy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <esp_pthread.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <cmath>

#include "mpu6050.hpp"
#include "servo.hpp"

using namespace std::chrono_literals;

static const char* TAG = "main";
static Servo servo;
static std::atomic<float> roll{0.0f};
static std::atomic<float> Kp{3.0f};
static std::atomic<float> Ki{0.05f};
static std::atomic<float> Kd{0.5f};
static std::atomic<float> alpha{0.98f};
static std::atomic<bool> pid_enabled{true};
static std::atomic<float> manual_angle{90.0f};
static std::atomic<float> manual_smoothed_angle{90.0f};

static MPU6050::Device Dev = {
    .i2cPort = 0,
    .i2cAddress = MPU6050::I2C_ADDRESS_MPU6050_AD0_L
};

float computeAccelRoll(float ax, float ay, float az) {
    return atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / M_PI;
}

float integrateGyroRoll(float prevRoll, float gyroY_deg_per_sec, float dt) {
    return prevRoll + gyroY_deg_per_sec * dt;
}

bool calibrateSensors(MPU6050::MPU6050_Driver& mpu, float& gyroBiasY, int samples = 500) {
    float sumGyroY = 0;
    int successCount = 0;

    std::cout << "Calibrating IMU" << std::endl;
    
    for (int i = 0; i < samples; i++) {
        MPU6050::Mpu6050_GyroData_t gyroData;
        if (mpu.Mpu6050_GetGyroData(gyroData) == Mpu6050_Error_t::MPU6050_OK) {
            sumGyroY += gyroData.Gyro_Y;
            successCount++;
        }
        std::this_thread::sleep_for(10ms);
    }

    if (successCount < samples * 0.9) {
        std::cerr << "Calibration failed - too many read errors" << std::endl;
        return false;
    }

    gyroBiasY = sumGyroY / successCount;
    std::cout << "Calibration complete. Gyro bias Y: " << gyroBiasY << " °/s" << std::endl;
    return true;
}

void sendGains() {
    std::cout << "Gains," 
              << Kp.load() << "," 
              << Ki.load() << "," 
              << Kd.load() << "," 
              << alpha.load() << std::endl;
}

void gain_tuning_thread() {
    char buffer[64];

    std::this_thread::sleep_for(1000ms);
    sendGains();

    while (true) {
        if (fgets(buffer, sizeof(buffer), stdin) != nullptr) {
            
            if (strncmp(buffer, "pid ", 4) == 0) {
                int enabled;
                if (sscanf(buffer + 4, "%d", &enabled) == 1) {
                    pid_enabled = (enabled != 0);
                    std::cerr << "[PID Toggle] PID enabled: " << std::boolalpha << pid_enabled << std::endl;
                } else {
                    std::cerr << "[PID Toggle] Invalid PID toggle format: " << buffer;
                }
                continue;
            }

            if (buffer[0] == 'm') {
                float angle;
                if (sscanf(buffer + 1, "%f", &angle) == 1) {
                    manual_angle = std::clamp(angle, 0.0f, 180.0f);
                    std::cerr << "[Manual Servo] Angle set to: " << manual_angle << "°" << std::endl;
                } else {
                    std::cerr << "[Manual Servo] Invalid angle format: " << buffer;
                }
                continue;
            }
            
            char type;
            float value;
            if (sscanf(buffer, " %c %f", &type, &value) == 2) {
                switch (type) {
                    case 'p':
                    case 'P':
                        Kp = value;
                        std::cerr << "[Gain Update] Kp = " << Kp << std::endl;
                        break;
                    case 'i':
                    case 'I':
                        Ki = value;
                        std::cerr << "[Gain Update] Ki = " << Ki << std::endl;
                        break;
                    case 'd':
                    case 'D':
                        Kd = value;
                        std::cerr << "[Gain Update] Kd = " << Kd << std::endl;
                        break;
                    case 'a':
                    case 'A':
                        alpha = value;
                        std::cerr << "[Complimentary Filter Update] Alpha = " << alpha << std::endl;
                        break;
                    default:
                        // std::cerr << "[Gain Update] Unknown type: " << type << std::endl;
                        break;
                }
            } else {
                std::cerr << "[Gain Update] Invalid input format: " << buffer;
            }
        }

        std::this_thread::sleep_for(10ms);
    }
}


void imu_sensor_thread() {

    if (mpu6050_hal_init(Dev.i2cPort) == Mpu6050_Error_t::MPU6050_ERR) {
        std::cerr << "Failed to initialize I2C HAL" << std::endl;
        return;
    }

    MPU6050::MPU6050_Driver mpu(Dev);
    if (mpu.Mpu6050_Init(&MPU6050::DefaultConfig) != Mpu6050_Error_t::MPU6050_OK) {
        std::cerr << "MPU6050 initialization failed!" << std::endl;
        return;
    }

    uint8_t dev_id = 0;
    if (mpu.Mpu6050_GetDevideId(dev_id) != Mpu6050_Error_t::MPU6050_OK || dev_id != MPU6050::WHO_AM_I_VAL) {
        std::cerr << "Invalid MPU6050 device ID: 0x" 
                  << std::hex << static_cast<int>(dev_id) << std::dec << std::endl;
        return;
    }

    std::cout << "MPU6050 initialized successfully. Device ID: 0x"
              << std::hex << static_cast<int>(dev_id) << std::dec << std::endl;

    float gyroBiasY = 0.0f;
    if (!calibrateSensors(mpu, gyroBiasY)) {
        std::cerr << "Failed calibration, using zero bias" << std::endl;
    }

    if (servo.attach(0) != Pwm_Error_t::PWM_OK) {
        std::cerr << "Failed to attach servo" << std::endl;
        return;
    }

    std::this_thread::sleep_for(500ms);

    // float alpha = 0.98f;
    // float Kp = 3.0f;
    // float Ki = 0.05f;
    // float Kd = 0.5f;
    float integral = 0.0f;
    float prevError = 0.0f;
    const float integralLimit = 25.0f;

    auto prevTime = std::chrono::steady_clock::now();
    int errorCount = 0;
    const int maxErrorCount = 10;
    float servoAngle = 90.0;
    int servoAnglePrev = 0;

    while (true) {
        MPU6050::Mpu6050_AccelData_t accelData;
        MPU6050::Mpu6050_GyroData_t gyroData;

        if (mpu.Mpu6050_GetAccelData(accelData) != Mpu6050_Error_t::MPU6050_OK ||
            mpu.Mpu6050_GetGyroData(gyroData) != Mpu6050_Error_t::MPU6050_OK) {
            if (++errorCount > maxErrorCount) {
                std::cerr << "Too many sensor errors, exiting!" << std::endl;
                return;
            }
            std::this_thread::sleep_for(20ms);
            continue;
        }
        errorCount = 0;

        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - prevTime).count();
        prevTime = now;

        float gyroY_corrected = gyroData.Gyro_Y - gyroBiasY;
        float accRoll = computeAccelRoll(accelData.Accel_X, accelData.Accel_Y, accelData.Accel_Z);
        float gyroRoll = integrateGyroRoll(roll, gyroY_corrected, dt);
        roll = alpha * gyroRoll + (1.0f - alpha) * accRoll;

        float error = 0.0f - roll;
        integral += error * dt;
        integral = std::clamp(integral, -integralLimit, integralLimit);
        float derivative = (error - prevError) / dt;
        float control = Kp * error + Ki * integral + Kd * derivative;
        prevError = error;

        if (pid_enabled) {
            servoAngle = std::clamp(90.0f + control, 0.0f, 180.0f);
        } else {
            float target = manual_angle.load();
            float current = manual_smoothed_angle.load();

            float smoothing_factor = 0.1f;
            float delta = target - current;

            if (std::fabs(delta) > 0.5f) {
                current += delta * smoothing_factor;
            } else {
                current = target;
            }

            manual_smoothed_angle = current;
            servoAngle = current;
        }

        if(static_cast<int>(servoAngle) != servoAnglePrev)
        {
            servoAnglePrev = static_cast<int>(servoAngle);
            servo.setAngle(static_cast<int>(servoAngle));
        }

        static int printCounter = 0;
        if (++printCounter >= 10) {
            printCounter = 0;
            // std::cout << "Accelerometer Roll: " << accRoll << "°, "
            //           << "Gyro Roll: " << gyroRoll << "°, "
            //           << "Roll: " << roll << "°, "
            //           << "Servo: " << static_cast<int>(servoAngle) << "°" << std::endl;
            // std::cout << roll << "," << servoAngle << std::endl;
            std::cout << roll << "," << accRoll << "," << gyroRoll << "," << servoAngle << std::endl;

            // std::cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << ", Control: " << control << std::endl;
            std::cout << "Kp: " << Kp << ", Ki: " << Ki << ", Kd: " << Kd << ", Alpha: " << alpha << ", Control: " << control << std::endl;
        }

        auto processingTime = std::chrono::steady_clock::now() - now;
        auto sleepDuration = 20ms - processingTime;
        if (sleepDuration > 0ms) {
            std::this_thread::sleep_for(sleepDuration);
        }
    }
}

extern "C" void app_main(void) {
    esp_pthread_cfg_t cfg = esp_pthread_get_default_config();
    cfg.stack_size = 4096;
    cfg.prio = 5;
    cfg.pin_to_core = 0;
    cfg.thread_name = "imu_thread";
    ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg));

    std::thread imu_thread(imu_sensor_thread);
    imu_thread.detach();

    esp_pthread_cfg_t cfg_gain = esp_pthread_get_default_config();
    cfg_gain.stack_size = 4096;
    cfg_gain.prio = 5;
    cfg_gain.pin_to_core = 1;
    cfg_gain.thread_name = "gain_thread";
    ESP_ERROR_CHECK(esp_pthread_set_cfg(&cfg_gain));
    std::thread gain_thread(gain_tuning_thread);
    gain_thread.detach();
}
