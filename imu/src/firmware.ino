// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <stdio.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>


#include <sensor_msgs/msg/imu.h>
#include "config.h"
#include "imu.h"
IMU imu;

sensor_msgs__msg__Imu imu_msg;

bool imu_ok = false;

void setup()
{
    Serial.begin(9600);
    while (!Serial) {
    }
    Wire.begin();

    delay(5000);
    Serial.println("Dit is een IMU tester");
    Serial.println("");

    Serial.println("Initializing IMU...");
    auto& mpu2 = (MPU6050IMU&)imu;
    imu_ok = imu.init();
    while(!imu_ok)
    {
//        flashLED(3);
        Serial.println("Initializing again...");
        Serial.print("DeviceID: ");
        Serial.println((int)mpu2.deviceId());
        delay(100);
        imu_ok = imu.init();
    }
    Serial.println("IMU initialized");
    Serial.print("Version: ");
    Serial.println((int)mpu2.deviceId());
    delay(10000);
}

void loop() {
    imu_msg = imu.getData();

    Serial.write(27);
    Serial.print("[2J");
    Serial.write(27);
    Serial.print("[H");
    
    Serial.print("Angular Velocity X:    ");
    Serial.println(imu_msg.angular_velocity.x);
    Serial.print("Angular Velocity Y:    ");
    Serial.println(imu_msg.angular_velocity.y);
    Serial.print("Angular Velocity Z:    ");
    Serial.println(imu_msg.angular_velocity.z);

    Serial.print("Linear accel X:        ");
    Serial.println(imu_msg.linear_acceleration.x);
    Serial.print("Linear accel Y:        ");
    Serial.println(imu_msg.linear_acceleration.y);
    Serial.print("Linear accel Z:        ");
    Serial.println(imu_msg.linear_acceleration.z);
    Serial.print("Linear accel cov 0:    ");
    Serial.println(imu_msg.linear_acceleration_covariance[0]);
    Serial.print("Linear accel cov 4:    ");
    Serial.println(imu_msg.linear_acceleration_covariance[4]);
    Serial.print("Linear accel cov 8:    ");
    Serial.println(imu_msg.linear_acceleration_covariance[8]);

    Serial.print("Orientation cov 0:     ");
    Serial.println(imu_msg.orientation_covariance[0]);
    Serial.print("Orientation cov 4:     ");
    Serial.println(imu_msg.orientation_covariance[4]);
    Serial.print("Orientation cov 8:     ");
    Serial.println(imu_msg.orientation_covariance[8]);


    delay(50);
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}
