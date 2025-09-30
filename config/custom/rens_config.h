// Copyright (c) 2025 Avans Hogeschool

#ifndef RENS_CONFIG_H
#define RENS_CONFIG_H

#define LED_PIN 13

#define LINO_BASE DIFFERENTIAL_DRIVE
#define USE_GENERIC_1_IN_MOTOR_DRIVER
#define USE_MPU6050_IMU

#define ACCEL_COV { 0.01, 0.01, 0.01 }
#define GYRO_COV { 0.001, 0.001, 0.001 }
#define ORI_COV { 0.01, 0.01, 0.01 }
#define MAG_COV { 1e-12, 1e-12, 1e-12 }
#define POSE_COV { 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 }
#define TWIST_COV { 0.001, 0.001, 0.001, 0.003, 0.003, 0.003 }

#define K_P 0.6
#define K_I 0.8
#define K_D 0.5

#define MOTOR_MAX_RPM 100                   // motor's max RPM..........
#define MAX_RPM_RATIO 0.9                  // max RPM allowed for each MAX_RPM_ALLOWED = MOTO
#define MOTOR_OPERATING_VOLTAGE 24          // motor's operating voltage (used to calculate m
#define MOTOR_POWER_MAX_VOLTAGE 20        // max voltage of the motor's power source (used to
#define MOTOR_POWER_MEASURED_VOLTAGE 20   // current voltage reading of the power connected t
#define COUNTS_PER_REV1 6400              // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 6400              // wheel2 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.144                // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.271            // distance between left and right wheels
#define PWM_BITS 10                          // PWM Resolution of the microcontroller
#define PWM_FREQUENCY 20000                 // PWM Frequency


// INVERT ENCODER COUNTS
#define MOTOR1_ENCODER_INV true
#define MOTOR2_ENCODER_INV false

// INVERT MOTOR DIRECTIONS
#define MOTOR1_INV false
#define MOTOR2_INV false

// ENCODER PINS
#define MOTOR1_ENCODER_A 14
#define MOTOR1_ENCODER_B 15

#define MOTOR2_ENCODER_A 16
#define MOTOR2_ENCODER_B 17

#define MOTOR1_PWM 1 //Pin no 21 is not a PWM pin on Teensy 4.x, you can use pin no 1 inste
#define MOTOR1_IN_A 3

#define MOTOR2_PWM 5
#define MOTOR2_IN_A 6

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -PWM_MAX








//unused but still needed
#define COUNTS_PER_REV3 6400              // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV4 6400              // wheel2 encoder's no of ticks per rev


#define MOTOR3_ENCODER_INV false
#define MOTOR4_ENCODER_INV false

#define MOTOR3_INV false
#define MOTOR4_INV false

#define MOTOR3_ENCODER_A -1
#define MOTOR3_ENCODER_B -1

#define MOTOR4_ENCODER_A -1
#define MOTOR4_ENCODER_B -1

#define MOTOR1_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder
#define MOTOR2_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

#define MOTOR3_PWM -1
#define MOTOR3_IN_A -1
#define MOTOR3_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder

#define MOTOR4_PWM -1
#define MOTOR4_IN_A -1
#define MOTOR4_IN_B -1 //DON'T TOUCH THIS! This is just a placeholder



#endif