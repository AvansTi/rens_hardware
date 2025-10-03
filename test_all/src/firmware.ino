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
    for(int i = 0; i < 5; i++)
    {
        Serial.print("Starting in ");
        Serial.print(5-i);
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
    int pwm_max = PWM_MAX; //macro
    Serial.println("[testMotor] Testing motors");
    Serial.print("[testMotor] PWM_MAX: ");
    Serial.println(PWM_MAX);

    int inc = (PWM_MAX)/100;
    Serial.print("[testMotor] inc: ");
    Serial.println(inc);
    for(int i = 0; i < total_motors; i++)
    {
        Serial.print("[testMotor] Motor ");
        Serial.println(i+1);
        for(int pwm = 0; pwm < pwm_max; pwm+=inc)
        {
            motors[i]->spin(min(pwm, pwm_max));
            delay(10);
        }
        delay(500);
        for(int pwm = pwm_max; pwm > -pwm_max; pwm-=inc)
        {
            motors[i]->spin(max(pwm, -pwm_max));
            delay(10);
        }
        delay(500);
        for(int pwm = -pwm_max; pwm < 0; pwm+=inc)
        {
            motors[i]->spin(pwm);
            delay(10);
        }
        motors[i]->spin(0);
        delay(500);
    }

    Serial.println("[testMotor] Done");
}

void testImu()
{
  /*  Serial.print("[testMotor] Testing IMU");
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

*/
    delay(50);
    Serial.println("[testIMU] Done");
}

void testEncoders()
{
    Serial.println("[testEncoders] Testing encoders");
    for(int current_motor = 0; current_motor < total_motors; current_motor++)
    {
        Serial.print("[testEncoders] Testing encoder ");
        Serial.println(current_motor+1);
        delay(50);


        motors[current_motor]->spin(PWM_MAX);
        delay(500);

        for(int i = 0; i < 10; i++)
        {
            int rpm = encoders[current_motor]->getRPM();
            Serial.print("[testEncoders] RPM for motor ");
            Serial.print(current_motor+1);
            Serial.print(" : ");
            Serial.println(rpm);
            delay(50);
        }
        motors[current_motor]->spin(0);
        delay(50);
    }
    Serial.println("[testEncoders] Done");
}