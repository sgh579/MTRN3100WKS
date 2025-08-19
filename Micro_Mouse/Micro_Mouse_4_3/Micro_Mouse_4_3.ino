// Standard library
#include <math.h>
#include <stdio.h>
#include <string.h>

// Arduino / third-party libraries
#include <SSD1306Ascii.h>        // Memory-efficient OLED library
#include <SSD1306AsciiWire.h>    // I2C version
#include <MPU6050_light.h>
#include <SPI.h>
#include <VL6180X.h>
#include <Wire.h>

// Project headers
#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "IntegratedMicromouseSolver.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

// ROBOT geometry
#define R 15.5                     // radius of the wheel
#define LENGTH_TO_ROTATION_SCALE 1 // to be adjusted based on test
#define LENGTH_OF_AXLE 82          // length of the axle in mm, used for distance calculations

// These are the pins set up
#define EN_1_A 2
#define EN_1_B 7
#define EN_2_A 3
#define EN_2_B 8
#define MOT1PWM 11
#define MOT1DIR 12
#define MOT2PWM 9
#define MOT2DIR 10

// OLED display settings - SSD1306Ascii
#define I2C_ADDRESS 0x3C

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
#define WALL_DISTANCE_THRESHOLD 100.0f

// Global objects
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(15.5, 82);
mtrn3100::PIDController motor1_encoder_position_controller(35, 0.05, 1); // 0.05
mtrn3100::PIDController motor2_encoder_position_controller(35, 0.05, 1);
mtrn3100::PIDController yaw_controller(0.25, 0.3, 0);
mtrn3100::Motor motor1(MOT1PWM, MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM, MOT2DIR);
SSD1306AsciiWire oled;  // Memory-efficient OLED object
MPU6050 mpu(Wire);
VL6180X sensor1; // left
VL6180X sensor2; // front
VL6180X sensor3; // right
// IntegratedMicromouseSolver *maze_solver = nullptr;
IntegratedMicromouseSolver maze_solver(Position(0, 0), Position(3, 3));


unsigned char sensor1_pin = A0; // ENABLE PIN FOR SENSOR 1 40
unsigned char sensor2_pin = A1; // ENABLE PIN FOR SENSOR 2 41
unsigned char sensor3_pin = A2; // ENABLE PIN FOR SENSOR 3 42

// Global variables
int loop_counter = 0; // Counter for the loop iterations, used for debugging and control

char curr_cmd = '\0';

float target_distance = 0; // Target for the robot to travel. Change this value and it applies in the feedback control loop
float target_angle = 0;

float target_motion_rotation_radians = 0; // Target motion rotation in radians, calculated based on the target distance and robot specifications

unsigned short int lidar_left = 0;
unsigned short int lidar_front = 0;
unsigned short int lidar_right = 0;

// buffer
char monitor_buffer[50];

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    // Set up the IMU
    byte status = mpu.begin();
    while (status != 0)
    {
    } // stop everything if could not connect to MPU6050

    delay(1000);
    mpu.calcOffsets(true, true);

    // Initialize SSD1306Ascii display - memory efficient
    oled.begin(&Adafruit128x64, I2C_ADDRESS);
    oled.setFont(Adafruit5x7);
    oled.clear();
    
    delay(1000); // Pause for 1 seconds

    // setup zero reference for the pid controllers
    motor1_encoder_position_controller.zeroAndSetTarget(encoder.getLeftRotation(), 0);
    motor2_encoder_position_controller.zeroAndSetTarget(encoder.getRightRotation(), -0); // reverse it for vehicle's motion
    yaw_controller.zeroAndSetTarget(0, 0);

    // lidar setup
    lidarInitialize();

    show_one_line_monitor("ROBOT setup completed");
}

void loop()
{
    /**
     *   Read Sensors
     */
    encoder_odometry.update(encoder.getLeftRotation(), encoder.getRightRotation());
    mpu.update();

    float curr_X = encoder_odometry.getX();
    float curr_Y = encoder_odometry.getY();
    float current_angle = mpu.getAngleZ();

    // Read LIDAR sensors
    lidar_left = sensor1.readRangeSingleMillimeters();
    lidar_front = sensor2.readRangeSingleMillimeters();
    lidar_right = sensor3.readRangeSingleMillimeters();

    // Get motor outputs for completion detection
    int motor1_output = motor1_encoder_position_controller.compute(encoder.getLeftRotation());
    int motor2_output = motor2_encoder_position_controller.compute(encoder.getRightRotation());

    /**
     *   Process maze solving
     */
    char next_command = '\0';
    float next_value = 0;

    bool continue_solving = maze_solver.processMazeStep(
        curr_X, curr_Y, current_angle,
        lidar_left, lidar_front, lidar_right,
        motor1_output, motor2_output,
        next_command, next_value);

    // Execute new command if one was generated
    if (next_command != '\0' && !maze_solver.isMovementInProgress()) {
        executeCommand(next_command, next_value, curr_X, curr_Y, current_angle);
    }

    // Display map and completion
    if (loop_counter % 1000 == 0) {
        char map[10][17];
        maze_solver.getDisplayMaze(map);

        int percentage = maze_solver.getPercentage();

        oled.clear();
        for (int i = 0; i < 10; i++) {
            oled.println(map[i]);
        }
    }

    // Handle completion
    if (!continue_solving && maze_solver.getState() == COMPLETED)
    {
        show_one_line_monitor("MAZE SOLVED!");
        motor1.setPWM(0);
        motor2.setPWM(0);
        while (true) {}
    }

    // Your existing control loop
    runControlLoop(curr_X, curr_Y, current_angle);

    loop_counter++;
    if (loop_counter > 30000)
        loop_counter = 0;
    delay(5);
}

void lidarInitialize()
{
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

void executeCommand(char command, float value, float curr_x, float curr_y, float curr_angle)
{
    switch (command) {
        case 'f':
            target_distance = value;
            target_angle = curr_angle;
            yaw_controller.zeroAndSetTarget(curr_angle, 0);
            yaw_controller.disable();
            break;

        case 'o':
            target_distance = 0;
            target_angle = value;
            float turn_angle = target_angle - curr_angle;
            if (turn_angle < -180.0f)
                turn_angle += 360.0f;
            if (turn_angle > 180.0f)
                turn_angle -= 360.0f;
            yaw_controller.zeroAndSetTarget(curr_angle, turn_angle);
            yaw_controller.enable();
            break;
    }

    motor1_encoder_position_controller.setZeroRef(encoder.getLeftRotation());
    motor2_encoder_position_controller.setZeroRef(encoder.getRightRotation());

    maze_solver.startMovement(command, value, curr_x, curr_y);
    curr_cmd = command;
}

void runControlLoop(float curr_X, float curr_Y, float current_angle)
{
    float lidar_offset = 0;

    if (curr_cmd == 'f')
    {
        // Wall following correction (your existing code)
        float lidar_err_left = 0;
        if (lidar_left <= WALL_DISTANCE_THRESHOLD)
        {
            lidar_err_left = DESIRED_WALL_DISTANCE - lidar_left;
        }

        float lidar_err_right = 0;
        if (lidar_right <= WALL_DISTANCE_THRESHOLD)
        {
            lidar_err_right = DESIRED_WALL_DISTANCE - lidar_right;
        }

        float scale = 0.2;
        lidar_offset = scale * (lidar_err_left - lidar_err_right);
    }

    // PID control
    float target_motion_rotation_radians = (target_distance * LENGTH_TO_ROTATION_SCALE) / R;
    float yaw_controller_output = yaw_controller.compute(current_angle);

    motor1_encoder_position_controller.setTarget(target_motion_rotation_radians - yaw_controller_output + lidar_offset);
    motor2_encoder_position_controller.setTarget(-target_motion_rotation_radians - yaw_controller_output + lidar_offset);

    int motor1_output = motor1_encoder_position_controller.compute(encoder.getLeftRotation());
    int motor2_output = motor2_encoder_position_controller.compute(encoder.getRightRotation());

    motor1.setPWM(motor1_output);
    motor2.setPWM(motor2_output);
}

float normalizeAngle(float angle)
{
    float result = fmod(angle, 360.0f);
    if (result < 0)
        result += 360.0f;
    return result;
}

float angleDifference(float a, float b)
{
    float diff = normalizeAngle(a - b);
    if (diff > 180.0f)
        diff = 360.0f - diff;
    return diff;
}

// Memory-efficient display function using SSD1306Ascii
void show_one_line_monitor(const char *str)
{
    oled.clear();
    oled.println(str);
}