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
#include "CommandParser.hpp"
#include <math.h> 
#include <VL6180X.h>

// TODO: not dealing with bounds very well - when starting from a big number and going to a small number, need to tell it to turn left or right
// Note: i think its working?
// char *script = "f180|o90|f180|o180|f180|o270|f180|o0";
// char *script = "o40|f50|o350|f50|o180|f50|o0";

// Polygon
// char *script = "o270|f18s0|f180|f180|f180|f180|f180|o0|f180|f180|f180|f180|f180|f180|o0";

// Small angles script
// char *script = "f100|o45|f50|o90|f30|o245|f100";

// Straight script 
// char *script = "f50|f50|f50|f50|f50|f50|f50";
// char *script = "f300|f300|f300";

char *script = "f1|o90|f540|o0|f180|o270|f180|o180|f180|o270|f360";



// ROBOT geometry
#define R 15.5 // radius of the wheel
#define LENGTH_TO_ROTATION_SCALE 1 // to be adjusted based on test
#define LENGTH_OF_AXLE 82 // length of the axle in mm, used for distance calculations


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
#define SCREEN_HEIGHT 24 // OLED display height, in pixels
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

// Maze settings
#define CELL_SIZE 180 // Size of the cell in mm, used for distance calculations

// Cycle threshold required for instruction completion determination
#define CMD_COMPLETE_STABLE_CYCLES 20 // TODO: ADJUST BACK TO 20
#define POSITION_ERROR_THRESHOLD 5.0f
#define ANGLE_ERROR_THRESHOLD 10.0f
#define MOTOR_OUTPUT_THRESHOLD 40
#define YAW_OUTPUT_THRESHOLD 30.0f
#define BIGGEST_WALL_DISTANCE_THRESHOLD 100.0f // TODO: ADJUST
#define DESIRED_WALL_DISTANCE 50.0f

// Global variables

// Global objects
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(15.5, 82); 
mtrn3100::PIDController motor1_encoder_position_controller(35, 0.05, 0.5); // 0.05
mtrn3100::PIDController motor2_encoder_position_controller(35, 0.05, 0.5);
mtrn3100::PIDController yaw_controller(0.25, 0.3, 0);
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 mpu(Wire);
mtrn3100::CommandParser commands(script); // Command sequence for the robot to follow. Length of the command sequence can be changed, and we can adapt to it in the code
VL6180X sensor1; // left
VL6180X sensor2; // front
VL6180X sensor3; // right

int sensor1_pin = A0; // ENABLE PIN FOR SENSOR 1 40
int sensor2_pin = A1; // ENABLE PIN FOR SENSOR 2 41
int sensor3_pin = A2; // ENABLE PIN FOR SENSOR 3 42

// Global variables
int loop_counter = 0; // Counter for the loop iterations, used for debugging and control

char prev_cmd = '\0';

float previous_X = 0; // Previous X position of the robot, used to calculate distance traveled between commands
float previous_Y = 0; 

float target_distance = 0; // Target for the robot to travel. Change this value and it applies in the feedbacj control loop
float target_angle = 0;

float current_angle_z; // Current angle of the robot in the Z direction, used for orientation control

float target_motion_rotation_radians = 0; // Target motion rotation in radians, calculated based on the target distance and robot specifications

float curr_X = 0; // Current X position of the robot based on odometry, updated in each loop iteration
float curr_Y = 0;
float current_angle = 0;

int lidar_left = 0;
int lidar_front = 0;
int lidar_right = 0;

// FLAGs
bool cmd_sequence_completion_FLAG = false; 

// buffer
char monitor_buffer[64];

// Outputs for the PID controllers
int motor1_encoder_position_controller_output = 0;
int motor2_encoder_position_controller_output = 0;

bool finished = false;

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

    // screen
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    Serial.println(F("SSD1306 allocation completed"));

    display.display();
    delay(1000); // Pause for 1 seconds


    // setup zero reference for the pid controllers
    motor1_encoder_position_controller.zeroAndSetTarget(encoder.getLeftRotation(), 0); 
    motor2_encoder_position_controller.zeroAndSetTarget(encoder.getRightRotation(), -0); // reverse it for vehicle's motion
    yaw_controller.zeroAndSetTarget(0, 0);

    // lidar setup
    Serial.println(F("Initializing lidar sensors..."));
    lidarInitialize();
    Serial.println("Done!\n");

    show_one_line_monitor("ROBOT setup completed");
}

void loop() {
    // Read the sensors
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
    mpu.update();
    // yaw_controller.zeroAndSetTarget(current_angle_z, current_angle_z);
    // assuming we can always successfully read value from lidar, without timeout issue
    lidar_left = sensor1.readRangeSingleMillimeters();
    lidar_front = sensor2.readRangeSingleMillimeters();
    lidar_right = sensor3.readRangeSingleMillimeters();

    float current_angle_z = mpu.getAngleZ();

    // Take next instruction
    curr_X = encoder_odometry.getX();
    curr_Y = encoder_odometry.getY();
    current_angle = current_angle_z;

    //modify the kinematic control target only when
    // the command pointer is at the start or the previous command has been completed
    if (prev_cmd == '\0' || is_this_cmd_completed()) {

        // are all commands completed?
        if (commands.isEmpty()) { 
            cmd_sequence_completion_FLAG = true;
        } else {
            sprintf(monitor_buffer, "command: %c%d", commands.getMoveType(), (int) round(commands.getMoveValue()));
            show_one_line_monitor(monitor_buffer);

            // Record current state
            previous_X = encoder_odometry.getX();
            previous_Y = encoder_odometry.getY();

            // Set targets
            char c = commands.getMoveType();
            float value = commands.getMoveValue();
            switch (c) {
                case 'f':
                    target_distance = value; // mm
                    // target_distance = value * 10; // cm
                    target_angle = target_angle;
                    yaw_controller.zeroAndSetTarget(current_angle, 0);
                    motor1_encoder_position_controller.setZeroRef(encoder.getLeftRotation());
                    motor2_encoder_position_controller.setZeroRef(encoder.getRightRotation());
                    yaw_controller.disable(); 
                break;
                case 'o':
                    target_distance = 0;
                    target_angle = fmodf(value + 360.0f, 360.0f);
                    float turn_angle = target_angle - current_angle;
                    if (turn_angle < -180.0f) turn_angle += 360.0f;
                    if (turn_angle > 180.0f) turn_angle -= 360.0f;
                    yaw_controller.zeroAndSetTarget(current_angle, turn_angle); // TODO: ADJUST FOR NEGATIVES
                    motor1_encoder_position_controller.setZeroRef(encoder.getLeftRotation());
                    motor2_encoder_position_controller.setZeroRef(encoder.getRightRotation());
                    yaw_controller.enable();
                    
                break;
                default:
                    Serial.print("Invalid command: ");
                    Serial.println(c);
                    while(true) {}
                break;
            }

            prev_cmd = c; 

            //move to the next command
            commands.next();
        }        

        target_motion_rotation_radians = (target_distance * LENGTH_TO_ROTATION_SCALE) / R;
    }


    if (cmd_sequence_completion_FLAG) {
        Serial.println(F("[INFO] Command sequence completed. The robot stops."));
        show_one_line_monitor("Command sequence completed. ROBOT stopped");
        motor1.setPWM(0);
        motor2.setPWM(0);
        while(true){}
    }

    float lidar_offset = 0;
    if (prev_cmd == 'f') {
        // use simple if sentences to integrate lidar into self correction
        // is there a wall on lefthand?
        float lidar_err_left;
        if (lidar_left <= BIGGEST_WALL_DISTANCE_THRESHOLD){
            // calculate error = actual value - desired value
            lidar_err_left = DESIRED_WALL_DISTANCE - lidar_left;
        } else {
            lidar_err_left = 0;
        }

        float lidar_err_right;
        // is there a wall on righthand?
        if (lidar_right <= BIGGEST_WALL_DISTANCE_THRESHOLD){
            // calculate error = actual value - desired value
            lidar_err_right = DESIRED_WALL_DISTANCE - lidar_right;
        } else {
            lidar_err_right = 0;
        }

        float scale = 0.2;
        lidar_offset = scale * (lidar_err_left - lidar_err_right);

    }

    // feedback control, dont change this part
    float yaw_controller_output = yaw_controller.compute(current_angle_z);

    motor1_encoder_position_controller.setTarget(target_motion_rotation_radians - yaw_controller_output - lidar_offset); 
    motor2_encoder_position_controller.setTarget(-target_motion_rotation_radians - yaw_controller_output - lidar_offset);

    motor1_encoder_position_controller_output = motor1_encoder_position_controller.compute(encoder.getLeftRotation());
    motor2_encoder_position_controller_output = motor2_encoder_position_controller.compute(encoder.getRightRotation());

    motor1.setPWM(motor1_encoder_position_controller_output); 
    motor2.setPWM(motor2_encoder_position_controller_output); 


    // // Debugging information
    // Serial.print(F("*****************************************loop "));
    // Serial.print(loop_counter);
    // Serial.println(F("*****************************************"));
    // Serial.print(F("[INFO] angle Z: "));
    // Serial.println(current_angle_z);
    // Serial.print(F("[INFO] yaw_controller_output: "));
    // Serial.println(yaw_controller_output);
    // Serial.print(F("[INFO] motor1_encoder_position_controller_output: "));
    // Serial.println(motor1_encoder_position_controller_output);
    // Serial.print(F("[INFO] motor2_encoder_position_controller_output: "));
    // Serial.println(motor2_encoder_position_controller_output);

    // Serial.print(F("[INFO] Encoder left radian: "));
    // Serial.println(encoder.getLeftRotation());
    // Serial.print(F("[INFO] Encoder right radian: "));
    // Serial.println(encoder.getRightRotation());

    loop_counter++;

    if (loop_counter > 30000) {
        // Serial.println("[INFO]: Loop count exceeded 30000, resetting to 0.");
        loop_counter = 0;
    }

    // Stop the robot
    while (cmd_sequence_completion_FLAG) {}

    delay(5); // TODO: REMOVE THIS MAYBE?

}

void lidarInitialize() {
    // SET UP ENABLE PINS AND DISABLE SENSORS
    pinMode(sensor1_pin, OUTPUT);
    pinMode(sensor2_pin, OUTPUT);
    pinMode(sensor3_pin, OUTPUT);
    digitalWrite(sensor1_pin, LOW);
    digitalWrite(sensor2_pin, LOW);
    digitalWrite(sensor3_pin, LOW);

    // ENABLE FIRST SENSOR AND CHANGE THE ADDRESS 
    digitalWrite(sensor1_pin, HIGH);
    delay(50);
    sensor1.init();
    sensor1.configureDefault();
    sensor1.setTimeout(250);
    sensor1.setAddress(0x40);
    delay(50);

    // ENABLE SECOND SENSOR AND CHANGE THE ADDRESS 
    digitalWrite(sensor2_pin, HIGH);
    delay(50);
    sensor2.init();
    sensor2.configureDefault();
    sensor2.setTimeout(250);
    sensor2.setAddress(0x41);
    delay(50);

    // ENABLE THIRD SENSOR AND CHANGE THE ADDRESS 
    digitalWrite(sensor3_pin, HIGH);
    delay(50);
    sensor3.init();
    sensor3.configureDefault();
    sensor3.setTimeout(250);
    sensor3.setAddress(0x42);
    delay(50);
}

// TODO:based on threshold, determine if the command is completed, affecting the accuracy
bool is_this_cmd_completed() {
    char curr_cmd = prev_cmd;
    static int stable_counter = 0; // Count of consecutive cycles that meet the condition

    bool completed = false;

    if (curr_cmd == 'f') {
        float delta_x = curr_X - previous_X;
        float delta_y = curr_Y - previous_Y;
        float position_error = target_distance - sqrt(pow(delta_x, 2) + pow(delta_y, 2));

        if (position_error <= POSITION_ERROR_THRESHOLD &&
            abs(motor1_encoder_position_controller_output) < MOTOR_OUTPUT_THRESHOLD &&
            abs(motor2_encoder_position_controller_output) < MOTOR_OUTPUT_THRESHOLD) {
            stable_counter++;
        } else {
            stable_counter = 0;
        }
        completed = (stable_counter >= CMD_COMPLETE_STABLE_CYCLES);
    } else if (curr_cmd == 'o') {
        float angle_error = angleDifference(target_angle, current_angle);

        // sprintf(monitor_buffer, "c: %d\nt: %d", (int) current_angle, (int) target_angle);
        // show_one_line_monitor(monitor_buffer);

        if (angle_error <= ANGLE_ERROR_THRESHOLD &&
            abs(motor1_encoder_position_controller_output) < YAW_OUTPUT_THRESHOLD &&
            abs(motor2_encoder_position_controller_output) < YAW_OUTPUT_THRESHOLD) {
            stable_counter++;
        } else {
            stable_counter = 0;
        }
        completed = (stable_counter >= CMD_COMPLETE_STABLE_CYCLES);
    } else {
        stable_counter = 0;
    }

    if (completed) stable_counter = 0; // reset the stable counter if the command is completed
    return completed;
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

// thisfunction shows one character array on the OLED display
// it can't be called in high frequency
void show_one_line_monitor(const char* str) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    int i = 0;
    while (str[i] != '\0') {
        display.write(str[i]);
        i++;
    }
    display.display();
}


/*********************************

TODO

**************************************/


// a data structure of grid and localization
class my_map {
    public:
        my_map();
        get_x_on_grid_map();
        get_y_on_grid_map();
        get_x_on_ground();
        get_y_on_ground();
    private:
        int x_on_grid_map;
        int y_on_grid_map;
        float x_on_ground;
        float y_on_ground;

};


// better linear motion
// if lidar at side detect obstable less than 100
// we can add it into the feedabck contol to correct the motion
void straight_line_with_lidar(){
    return;
}

/**************************************/
// task 4.3
/**************************************/

// use BFS to find the shorted path from start to goal
void explore(){
    return;
}

// the last part of task 4.3
void execute_shortest_path(){
    return;
}