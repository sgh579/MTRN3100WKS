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
#include <math.h> 

#define EN_1_A 2
#define EN_1_B 7
#define EN_2_A 3
#define EN_2_B 8
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

#define SIZE 8
#define CELL_SIZE 180
#define DIST_TOLERANCE 5.0f // Distance tolerance in mm
#define MAX_COMMAND_TIME 5000 // Max time per command in ms - used to end turns - not the best solution

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(15.5, 82); 

mtrn3100::PIDController motor1_encoder_position_controller(100, 0.01, 0);
mtrn3100::PIDController motor2_encoder_position_controller(100, 0.01, 0);
mtrn3100::PIDController yaw_controller(0.05, 0.1, 0);

// TODO: create a struct for this, avoid using 2 separate motor objects
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);

/*************** SCREEN *******************/

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 mpu(Wire);

int loop_counter = 0;
int cmd_ctr = 0;
char commands[9] = "lfl"; 

float old_x = 0, old_y = 0;
float curr_x = 0, curr_y = 0;
float current_angle = 0, current_angle_z = 0;
float target_distance = 0;
float target_angle = 0;
static float target_motion_rotation_radians = 0;
float r = 16;
float motion_length_to_rotation_scale = 1; // leave for now
bool first = true;

// Track time to detect command timeout
unsigned long command_start_time = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while (status != 0) {}

    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true, true);
    Serial.println("Done!\n");

    yaw_controller.zeroAndSetTarget(0, 0);
}

void loop() {
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
    mpu.update();
    current_angle_z = mpu.getAngleZ();

    curr_x = encoder_odometry.getX();
    curr_y = encoder_odometry.getY();
    current_angle = current_angle_z;

    float yaw_controller_output = yaw_controller.compute(current_angle_z);

    if (cmd_ctr == 0 || checkCompletedCommand()) {
        if (cmd_ctr >= SIZE) {
            motor1.setPWM(0);
            motor2.setPWM(0);
            return;
        } else {
            old_x = curr_x;
            old_y = curr_y;

            char c = commands[cmd_ctr];
            switch (c) {
                case 'f':
                    target_distance = CELL_SIZE;
                    target_angle = current_angle_z;
                    yaw_controller.zeroAndSetTarget(0, 0);

                    // Compute only once at command start
                    // float motion_length_to_rotation_scale = 1; // leave for now
                    // float r = 16;
                    target_motion_rotation_radians = (target_distance * motion_length_to_rotation_scale) / r;

                    motor1_encoder_position_controller.zeroAndSetTarget(encoder.getLeftRotation(), target_motion_rotation_radians); 
                    motor2_encoder_position_controller.zeroAndSetTarget(encoder.getRightRotation(), -target_motion_rotation_radians);
                    break;

                case 'l':
                    target_distance = 0;
                    target_angle = fmodf(current_angle_z + 90.0f, 360.0f);
                    if (target_angle < 0) target_angle += 360.0f;
                    yaw_controller.zeroAndSetTarget(0, 90);

                    target_motion_rotation_radians = (target_distance * motion_length_to_rotation_scale) / r;

                    motor1_encoder_position_controller.zeroAndSetTarget(encoder.getLeftRotation(), target_motion_rotation_radians); 
                    motor2_encoder_position_controller.zeroAndSetTarget(encoder.getRightRotation(), -target_motion_rotation_radians);

                    // float yaw_controller_output = yaw_controller.compute(current_angle_z); // moved up

                    motor1_encoder_position_controller.setTarget(target_motion_rotation_radians - yaw_controller_output);
                    motor2_encoder_position_controller.setTarget(-target_motion_rotation_radians - yaw_controller_output);

                    // Move motors
                    int motor1_encoder_position_controller_output_left = motor1_encoder_position_controller.compute(encoder.getLeftRotation());
                    int motor2_encoder_position_controller_output_left = motor2_encoder_position_controller.compute(encoder.getRightRotation());

                    motor1.setPWM(motor1_encoder_position_controller_output_left); 
                    motor2.setPWM(motor2_encoder_position_controller_output_left); 
                    break;
                case 'r':
                    target_distance = 0;
                    target_angle = fmodf(current_angle_z - 90.0f + 360.0f, 360.0f); // changed
                    if (target_angle < 0) target_angle += 360.0f;
                    yaw_controller.zeroAndSetTarget(0, 270); // maybe change

                    target_motion_rotation_radians = (target_distance * motion_length_to_rotation_scale) / r;

                    motor1_encoder_position_controller.zeroAndSetTarget(encoder.getLeftRotation(), -target_motion_rotation_radians); 
                    motor2_encoder_position_controller.zeroAndSetTarget(encoder.getRightRotation(), target_motion_rotation_radians);

                    // float yaw_controller_output = yaw_controller.compute(current_angle_z);

                    motor1_encoder_position_controller.setTarget(-target_motion_rotation_radians + yaw_controller_output);
                    motor2_encoder_position_controller.setTarget(target_motion_rotation_radians - yaw_controller_output);

                    // Move motors
                    int motor1_encoder_position_controller_output_right = motor1_encoder_position_controller.compute(encoder.getLeftRotation());
                    int motor2_encoder_position_controller_output_right = motor2_encoder_position_controller.compute(encoder.getRightRotation());

                    motor1.setPWM(motor1_encoder_position_controller_output_right); 
                    motor2.setPWM(motor2_encoder_position_controller_output_right);
                    break;

                default:
                    Serial.print("Invalid command: ");
                    Serial.println(c);
                    break;
            }

            cmd_ctr++;
            command_start_time = millis(); // [ADD] Start timer
        }
    }

    if (millis() - command_start_time > MAX_COMMAND_TIME) { // see if necesary
        Serial.println("[WARN] Command timeout, stopping.");
        motor1.setPWM(0);
        motor2.setPWM(0);
        while (1); // freeze
    }

    float yaw_output = yaw_controller.compute(current_angle_z);
    motor1_encoder_position_controller.setTarget(target_motion_rotation_radians - yaw_output);
    motor2_encoder_position_controller.setTarget(-target_motion_rotation_radians - yaw_output);

    int motor1_output = motor1_encoder_position_controller.compute(encoder.getLeftRotation());
    int motor2_output = motor2_encoder_position_controller.compute(encoder.getRightRotation());

    motor1.setPWM(motor1_output); 
    motor2.setPWM(motor2_output); 

    Serial.print("**************** loop ");
    Serial.print(loop_counter);
    Serial.println(" ****************");
    Serial.print("[INFO] angle Z: "); Serial.println(current_angle_z);
    Serial.print("[INFO] yaw output: "); Serial.println(yaw_output);
    Serial.print("[INFO] motor1 output: "); Serial.println(motor1_output);
    Serial.print("[INFO] motor2 output: "); Serial.println(motor2_output);
    Serial.print("[INFO] Left encoder: "); Serial.println(encoder.getLeftRotation());
    Serial.print("[INFO] Right encoder: "); Serial.println(encoder.getRightRotation());

    float dist = sqrt(pow(curr_x - old_x, 2) + pow(curr_y - old_y, 2));
    // Serial.print("[INFO] Distance travelled: "); Serial.println(dist); // [ADD]

    loop_counter++;
    if (loop_counter > 30000) loop_counter = 0;

    first = false;
    delay(5);
}

bool checkCompletedCommand() {
    if (cmd_ctr == 0) return false; // no command yet

    char curr_cmd = commands[cmd_ctr - 1];
    if (curr_cmd == 'f') {
        float delta_x = curr_x - old_x;
        float delta_y = curr_y - old_y;
        float dist = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
        return dist >= (CELL_SIZE - DIST_TOLERANCE); // [MOD]
    } else if (curr_cmd == 'l' || curr_cmd == 'r') {
        float angleError = fmodf(target_angle - current_angle + 540.0f, 360.0f) - 180.0f;
        return abs(angleError) <= 3.0f;
    }

    return false; 
}
