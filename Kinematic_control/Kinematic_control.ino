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

#define SIZE 8

#define CELL_SIZE 180

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

int cmd_ctr = 0;
char commands[9] = "flrflrfl";

float old_x = 0; 
float old_y = 0; 

float target_distance = 0; 
float target_angle = 0;

float current_angle_z;

static float target_motion_rotation_radians = 0;

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

    // target_motion_rotation_radians = 2.0f * M_PIF;

    motor1_encoder_position_controller.zeroAndSetTarget(encoder.getLeftRotation(), target_motion_rotation_radians); 
    motor2_encoder_position_controller.zeroAndSetTarget(encoder.getRightRotation(), -target_motion_rotation_radians); // reverse it for vehicle's motion

    yaw_controller.zeroAndSetTarget(0, 0);

}

void loop() {
    // Read the sensors
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
    mpu.update();

    float current_angle_z = mpu.getAngleZ();

    // Take next instruction
    if (cmd_ctr == 0 || checkCompletedCommand()) {
        // Stop if there are no more instructions 
        // while (cmd_ctr >= SIZE) {}
        if (cmd_ctr >= SIZE) {
            target_distance = 0; 
            target_angle = 0;
        } else {
            // Record current state
            old_x = encoder_odometry.getX();
            old_y = encoder_odometry.getY();

            // Set targets
            char c = commands[cmd_ctr];
            switch (c) {
                case 'f':
                    target_distance = CELL_SIZE;
                    target_angle = current_angle_z;
                    yaw_controller.zeroAndSetTarget(0, 0);
                break;
                case 'l':
                    target_distance = 0;
                    target_angle = (current_angle_z + 90) % 360;
                    yaw_controller.zeroAndSetTarget(0, 90);
                break;
                case 'r':
                    target_distance = 0;
                    target_angle = (current_angle_z - 90 + 360) % 360;
                    yaw_controller.zeroAndSetTarget(0, -90);
                break;
                default:
                    Serial.print("Invalid command: ");
                    Serial.println(c);
                break;
            }

            cmd_ctr++;
        }
    }

    // Calculate Targets
    float motion_length_to_rotation_scale = 1; // to be adjusted based on the motor and encoder specifications
    float r = 15.5; // radius?
    target_motion_rotation_radians = (target_distance * motion_length_to_rotation_scale) / r;

    motor1_encoder_position_controller.zeroAndSetTarget(encoder.getLeftRotation(), target_motion_rotation_radians); 
    motor2_encoder_position_controller.zeroAndSetTarget(encoder.getRightRotation(), -target_motion_rotation_radians);

    float yaw_controller_output = yaw_controller.compute(current_angle_z);

    motor1_encoder_position_controller.setTarget(target_motion_rotation_radians - yaw_controller_output);
    motor2_encoder_position_controller.setTarget(-target_motion_rotation_radians - yaw_controller_output);

    // Move motors
    int motor1_encoder_position_controller_output = motor1_encoder_position_controller.compute(encoder.getLeftRotation());
    int motor2_encoder_position_controller_output = motor2_encoder_position_controller.compute(encoder.getRightRotation());

    motor1.setPWM(motor1_encoder_position_controller_output); 
    motor2.setPWM(motor2_encoder_position_controller_output); 

    Serial.print(F("*****************************************loop "));
    Serial.print(loop_counter);
    Serial.println(F("*****************************************"));
    Serial.print(F("[INFO] angle Z: "));
    Serial.println(current_angle_z);
    Serial.print(F("[INFO] yaw_controller_output: "));
    Serial.println(yaw_controller_output);
    Serial.print(F("[INFO] motor1_encoder_position_controller_output: "));
    Serial.println(motor1_encoder_position_controller_output);
    Serial.print(F("[INFO] motor2_encoder_position_controller_output: "));
    Serial.println(motor2_encoder_position_controller_output);

    Serial.print(F("[INFO] Encoder left radian: "));
    Serial.println(encoder.getLeftRotation());
    Serial.print(F("[INFO] Encoder right radian: "));
    Serial.println(encoder.getRightRotation());

    loop_counter++;

    if (loop_counter > 30000) {
        // Serial.println("[INFO]: Loop count exceeded 30000, resetting to 0.");
        loop_counter = 0;
    }

    first = false;

    delay(5);

}

// TODO: Checks if the current command has been completed
// TODO: NEED TO CHECK WHAT THE RANGE FOR ANGLES IS e.g. 0 - 360
bool checkCompletedCommand() {
    char curr_cmd = commands[cmd_ctr - 1];

    if (curr_cmd == 'f') {
        // TODO: MAY NEED TO ADJUST VARIANCE
        float delta_x = curr_x - old_x;
        float delta_y = curr_y - old_y; 

        return sqrt(pow(delta_x, 2) + pow(delta_y, 2)) >= CELL_SIZE;
    } else if (curr_cmd == 'l') {
        return abs(target_angle - current_angle) <= 3;
    } else if (curr_cmd == 'r') {
        return abs(target_angle - current_angle) <= 3;
    }

    return false; 
}
