#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include <SPI.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <stdio.h>
#include "PIDController.hpp"
#include "Motor.hpp"



#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder
#define MOT1PWM 11 // PIN 11 is motor1's PWM pin
#define MOT1DIR 12
#define MOT2PWM 9// PIN 9 is motor2's PWM pin
#define MOT2DIR 10

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(15.5, 82); 

mtrn3100::PIDController motor1_encoder_position_controller(100, 0.01, 0);
mtrn3100::PIDController motor2_encoder_position_controller(100, 0.01, 0);

mtrn3100::PIDController yaw_controller(0.05, 0.1, 0);

// TODO: create a struct for this, avoid using 2 separate motor objects
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

/*************** SCREEN *******************/

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MPU6050 mpu(Wire);

int loop_counter = 0;

static float target_motion_rotation_radians = 0;

// motion
enum MotionState {
    IDLE,
    MOVING_FORWARD,
    TURNING_LEFT,
    TURNING_RIGHT
};

MotionState currentMotion = MOVING_FORWARD;

float motion_target_distance_mm = 0;
float motion_target_rotation_deg = 0;
float motion_start_left = 0;
float motion_start_right = 0;
float motion_start_yaw = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    //Set up the IMU
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");

    float target_motion_length = 0; // 1000 mm, specified by task4
    float motion_length_to_rotation_scale = 1; // to be adjusted based on the motor and encoder specifications
    float r = 15.5; // radius?
    target_motion_rotation_radians = (target_motion_length * motion_length_to_rotation_scale) / r  ;

    // target_motion_rotation_radians = 2.0f * M_PIF;

    motor1_encoder_position_controller.zeroAndSetTarget(encoder.getLeftRotation(), target_motion_rotation_radians); 
    motor2_encoder_position_controller.zeroAndSetTarget(encoder.getRightRotation(), -target_motion_rotation_radians); // reverse it for vehicle's motion

    yaw_controller.zeroAndSetTarget(0, 90);

}

void loop() {
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
    mpu.update();

    float current_yaw = mpu.getAngleZ();
    float left_dist = encoder.getLeftDistanceMM() - motion_start_left;
    float right_dist = encoder.getRightDistanceMM() - motion_start_right;

    switch(currentMotion) {
        case IDLE:
            stopMotors();
        break;

        case MOVING_FORWARD: {
            float current_left = encoder.getLeftDistanceMM();
            float current_right = encoder.getRightDistanceMM();
            float avg_distance = (current_left - motion_start_left + current_right - motion_start_right) / 2.0;

            if (avg_distance < motion_target_distance_mm) {
                float yaw_error = mpu.getAngleZ() - motion_start_yaw;
                float yaw_correction = yaw_controller.compute(yaw_error);

                // You could adjust targets based on correction
                int speedL = motor1_encoder_position_controller.compute(encoder.getLeftRotation());
                int speedR = motor2_encoder_position_controller.compute(encoder.getRightRotation());

                motor1.setPWM(speedL - yaw_correction);
                motor2.setPWM(speedR + yaw_correction);
            } else {
                stopMotors();
                currentMotion = IDLE;
            }
            break;
        }

        break;

        case TURNING_LEFT:
        case TURNING_RIGHT: {
            float yaw_diff = current_yaw - motion_start_yaw;
            float target_yaw = motion_target_rotation_deg;

            if (abs(yaw_diff) < abs(target_yaw)) {
                float yaw_control = yaw_controller.compute(current_yaw);
                motor1.setPWM(-yaw_control);
                motor2.setPWM(yaw_control);
            } else {
                stopMotors();
                currentMotion = IDLE;
            }
            break;
        }

        default:
            break;
    }

    delay(5);
}


void moveForward(float distance_mm) {
    currentMotion = MOVING_FORWARD;

    // Record starting encoder readings in mm
    motion_start_left = encoder.getLeftDistanceMM();
    motion_start_right = encoder.getRightDistanceMM();

    // Store target
    motion_target_distance_mm = distance_mm;

    // Reset yaw for drift correction (optional)
    motion_start_yaw = mpu.getAngleZ();
}

void turnLeft90() {
    currentMotion = TURNING_LEFT;
    motion_target_rotation_deg = 90.0;
    motion_start_yaw = mpu.getAngleZ();
}

void turnRight90() {
    currentMotion = TURNING_RIGHT;
    motion_target_rotation_deg = -90.0;
    motion_start_yaw = mpu.getAngleZ();
}

void stopMotors() {
    motor1.setPWM(0);
    motor2.setPWM(0);
}
