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

#define IMU_NO_CALIBRATE
#include <Arduino.h>
#include <stdio.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>


#include "motor.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "kinematics.h"
#include <sensor_msgs/msg/imu.h>
#include "config.h"
#include "imu.h"



Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

long long int counts_per_rev[4];
int total_motors = 4;
Motor *motors[4] = {&motor1_controller, &motor2_controller, &motor3_controller, &motor4_controller};
Encoder *encoders[4];//= {&motor1_encoder, &motor2_encoder, &motor3_encoder, &motor4_encoder};
String labels[4] = {"FRONT LEFT - M1: ", "FRONT RIGHT - M2: ", "REAR LEFT - M3: ", "REAR RIGHT - M4: "};

IMU imu;
sensor_msgs__msg__Imu imu_msg;
bool imu_ok = false;

void setup()
{
    if(Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 2;
    }

    Serial.begin(9600);
    while (!Serial) {
    }
    Wire.begin();
    for(int i = 0; i < 3; i++)
    {
        Serial.print("Starting in ");
        Serial.print(3-i);
        Serial.print(" seconds");
        for(int ii = 0; ii < 10; ii++)
        {
            Serial.print(".");
            delay(100);
        }
        Serial.println();
    }
    if(total_motors > 0)
        encoders[0] = new Encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
    if(total_motors > 1)
        encoders[1] = new Encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
    if(total_motors > 2)
        encoders[2] = new Encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
    if(total_motors > 3)
        encoders[3] = new Encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

}

void loop() {
    test();
    Serial.println("Done testing");
    delay(5000);
    return;
}

void test()
{
    testLed();
    testMotor();
    testEncoders();
    testImu();
}

void testLed()
{
    Serial.println("[testLed] Testing built in LED...");
    for(int i=0; i<20; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    Serial.println("[testLed] Done");
}

void testMotor()
{
    Serial.println("[testMotor] Testing motors");
    Serial.print("[testMotor] PWM_MAX: ");
    Serial.println(PWM_MAX);

    for(int i = 0; i < total_motors; i++)
    {
        Serial.print("[testMotor] Motor ");
        Serial.println(i+1);
        accellerate(i, 0, PWM_MAX, 20, 50);
        delay(50);
        accellerate(i, PWM_MAX, -PWM_MAX, 20, 50);
        delay(50);
        accellerate(i, -PWM_MAX, 0, 20, 50);
        delay(250);
    }

    Serial.println("[testMotor] Done");
}

void testImu()
{
    Serial.println("[testImu] Testing IMU");
    Serial.println("[testImu] Initializing IMU...");
    auto& mpu2 = (MPU6050IMU&)imu;
    imu_ok = imu.init();
    if(!imu_ok)
    {
        Serial.println("[testImu] IMU initialization failed!");
        Serial.print("[testImu] DeviceID: ");
        Serial.println((int)mpu2.deviceId());
        return;
    }
    Serial.println("[testImu] IMU initialized");
    Serial.print("[testImu] DeviceID: ");
    Serial.println((int)mpu2.deviceId());

    imu_msg = imu.getData();

    Serial.println("[testImu] IMU data:");
    Serial.println("[testImu] Angular\t\tLinear Accel\t\t\tLinear Accel Cov\t\tOrientation Cov");
    for(int i = 0; i < 10; i++)
    {

        Serial.print(imu_msg.angular_velocity.x);
        Serial.write('\t');
        Serial.print(imu_msg.angular_velocity.y);
        Serial.write('\t');
        Serial.print(imu_msg.angular_velocity.z);
        Serial.write('\t');
        Serial.write('\t');

        Serial.print(imu_msg.linear_acceleration.x);
        Serial.write('\t');
        Serial.print(imu_msg.linear_acceleration.y);
        Serial.write('\t');
        Serial.print(imu_msg.linear_acceleration.z);
        Serial.write('\t');
        Serial.write('\t');
        Serial.print(imu_msg.linear_acceleration_covariance[0]);
        Serial.write('\t');
        Serial.print(imu_msg.linear_acceleration_covariance[4]);
        Serial.write('\t');
        Serial.print(imu_msg.linear_acceleration_covariance[8]);
        Serial.write('\t');
        Serial.write('\t');

        Serial.print(imu_msg.orientation_covariance[0]);
        Serial.write('\t');
        Serial.print(imu_msg.orientation_covariance[4]);
        Serial.write('\t');
        Serial.print(imu_msg.orientation_covariance[8]);
        Serial.println();
        delay(100);
    }
    Serial.println("[testIMU] Calibrating PID on IMU (can take a while)");
    Serial.println("[testIMU] Make sure the IMU is stationary during calibration");
    mpu2.calibrate();
    Serial.println("[testIMU] Done");
}

void testEncoders()
{
    Serial.println("[testEncoders] Testing encoders");

    for(int current_motor = 0; current_motor < total_motors; current_motor++)
        motors[current_motor]->spin(0);
    delay(750);

    for(int current_motor = 0; current_motor < total_motors; current_motor++)
    {
        Serial.print("[testEncoders] Testing RPM for motor ");
        Serial.println(current_motor+1);
        Serial.println("[testEncoders] Spinning up motor ");
        accellerate(current_motor, 0, PWM_MAX, 20, 50);
        delay(100);
        Serial.println("[testEncoders] Motor is up to speed, reading RPM for 2 seconds ");

        for(int encoder = 0; encoder < total_motors; encoder++)
            encoders[encoder]->getRPM(); //reset the rpm calculation
        delay(200);

        for(int i = 0; i < 10; i++)
        {
            for(int encoder = 0; encoder < total_motors; encoder++)
            {
                int rpm = encoders[encoder]->getRPM();
                Serial.print(encoder+1);
                Serial.print(": ");
                Serial.print(rpm);
                Serial.print("\t");
            }
            Serial.println();
            delay(200);
        }
        accellerate(current_motor, PWM_MAX, 0, 20, 50);
        delay(50);
    }
    Serial.println("[testEncoders] Done");
}


void accellerate(int motor, int start_pwm, int end_pwm, int stepcount, int delay_ms)
{
    int step = (end_pwm - start_pwm) / stepcount;
    if(start_pwm < end_pwm)
    {
        for(int pwm = start_pwm; pwm < end_pwm; pwm+=step)
        {
            motors[motor]->spin(min(pwm, PWM_MAX));
            delay(delay_ms);
        }
        motors[motor]->spin(end_pwm);
    }
    else
    {
        for(int pwm = start_pwm; pwm > end_pwm; pwm+=step)
        {
            motors[motor]->spin(max(pwm, -PWM_MAX));
            delay(delay_ms);
        }
        motors[motor]->spin(end_pwm);
    }
}