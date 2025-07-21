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


//These are the pins set up
#define EN_1_A 2 
#define EN_1_B 7 
#define EN_2_A 3 
#define EN_2_B 8 
#define MOT1PWM 11 
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10

// OLED display settings
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

// Maze settings
#define CELL_SIZE 180 // Size of the cell in mm, used for distance calculations

// Global variables

// Global objects
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(15.5, 82); 
mtrn3100::PIDController motor1_encoder_position_controller(100, 0.01, 0);
mtrn3100::PIDController motor2_encoder_position_controller(100, 0.01, 0);
mtrn3100::PIDController yaw_controller(0.05, 0.1, 0);
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 mpu(Wire);

// Global variables
int loop_counter = 0; // Counter for the loop iterations, used for debugging and control

int cmd_pointer = 0; // Pointer to the current command in the command sequence
char commands[] = "frflflfr"; // Command sequence for the robot to follow

float previous_X = 0; // Previous X position of the robot, used to calculate distance traveled between commands
float previous_Y = 0; 

float target_distance = 0; // Target for the robot to travel. Change this value and it applies in the feedbacj control loop
float target_angle = 0;

float current_angle_z; // Current angle of the robot in the Z direction, used for orientation control

float target_motion_rotation_radians = 0; // Target motion rotation in radians, calculated based on the target distance and robot specifications

float curr_X = 0; // Current X position of the robot based on odometry, updated in each loop iteration
float curr_Y = 0;
float current_angle = 0;

// FLAGs
bool cmd_sequence_completion_FLAG = false; 

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


    // setup zero reference for the pid controllers
    motor1_encoder_position_controller.zeroAndSetTarget(encoder.getLeftRotation(), 0); 
    motor2_encoder_position_controller.zeroAndSetTarget(encoder.getRightRotation(), -0); // reverse it for vehicle's motion
    yaw_controller.zeroAndSetTarget(0, 0);


}

void loop() {
    // Read the sensors
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
    mpu.update();
    // yaw_controller.zeroAndSetTarget(current_angle_z, current_angle_z);

    float current_angle_z = mpu.getAngleZ();

    // Take next instruction
    curr_X = encoder_odometry.getX();
    curr_Y = encoder_odometry.getY();
    current_angle = current_angle_z;
    if (cmd_pointer == 0 || checkCompletedCommand()) {
        // Stop if there are no more instructions 
        // while (cmd_pointer >= commands.length()) {}
        if (cmd_pointer >= sizeof(commands)-1) { // (cmd_pointer >= SIZE)
            target_distance = 0; 
            target_angle = current_angle_z;
            cmd_sequence_completion_FLAG = true;
        } else {
            // Record current state
            previous_X = encoder_odometry.getX();
            previous_Y = encoder_odometry.getY();

            // Set targets
            char c = commands[cmd_pointer];
            switch (c) {
                case 'f':
                    target_distance = CELL_SIZE;
                    target_angle = current_angle_z;
                    // yaw_controller.zeroAndSetTarget(0, 0); - removed // TODO: dont ser target here, we can change the target angle value
                break;
                case 'l':
                    target_distance = 0;
                    // target_angle = (current_angle_z + 90) % 360;
                    target_angle = fmodf(current_angle_z + 90.0f, 360.0f);
                    if (target_angle < 0) target_angle += 360.0f;
                    yaw_controller.zeroAndSetTarget(0, 90); // TODO: dont use zero and se
                    // yaw_controller.setTarget(target_angle);
                break;
                case 'r':
                    target_distance = 0;
                    target_angle = fmodf(current_angle_z - 90.0f + 360.0f, 360.0f);
                    if (target_angle < 0) target_angle += 360.0f;
                    yaw_controller.zeroAndSetTarget(0, -90);
                    // yaw_controller.setTarget(target_angle);
                break;
                default:
                    Serial.print("Invalid command: "); // TODO maybe stop the loop
                    Serial.println(c);
                break;
            }

            cmd_pointer++;
            if (cmd_pointer >= sizeof(commands) - 1) { // SIZE
    			motor1.setPWM(0);
    			motor2.setPWM(0);
                // while (1);
    			return; // or enter idle state
			}
        }
    }

    // Calculate Targets
    float motion_length_to_rotation_scale = 1; // to be adjusted based on the motor and encoder specifications
    float r = 15.5; // radius?
    target_motion_rotation_radians = (target_distance * motion_length_to_rotation_scale) / r;

    motor1_encoder_position_controller.zeroAndSetTarget(encoder.getLeftRotation(), target_motion_rotation_radians); 
    motor2_encoder_position_controller.zeroAndSetTarget(encoder.getRightRotation(), -target_motion_rotation_radians);

    float yaw_controller_output = yaw_controller.compute(current_angle_z);

    motor1_encoder_position_controller.setTarget(target_motion_rotation_radians - yaw_controller_output); // 
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

    // Stop the robot
    while (cmd_sequence_completion_FLAG) {}

    delay(5); // TODO: REMOVE THIS MAYBE?

}

// TODO: Checks if the current command has been completed
// TODO: NEED TO CHECK WHAT THE RANGE FOR ANGLES IS e.g. 0 - 360
bool checkCompletedCommand() {
    char curr_cmd = commands[cmd_pointer - 1];

    if (curr_cmd == 'f') {
        // TODO: MAY NEED TO ADJUST VARIANCE
        float delta_x = curr_X - previous_X;
        float delta_y = curr_Y - previous_Y; 

        return sqrt(pow(delta_x, 2) + pow(delta_y, 2)) >= CELL_SIZE;
    } else if (curr_cmd == 'l') {
        // return abs(target_angle - current_angle) <= 3;
        return angleDifference(target_angle, current_angle) <= 3.0f; 
    } else if (curr_cmd == 'r') {
        // return abs(target_angle - current_angle) <= 3;
        return angleDifference(target_angle, current_angle) <= 3.0f; 
    }

    return false;
}

float normalizeAngle(float angle) {
    float result = fmod(angle, 360.0f);
    if (result < 0) result += 360.0f;
    return result;
}

float angleDifference(float a, float b) {
    float diff = normalizeAngle(a - b);
    if (diff > 180.0f)
        diff = 360.0f - diff;
    return diff;
}