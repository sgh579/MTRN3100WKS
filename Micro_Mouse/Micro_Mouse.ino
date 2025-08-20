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

char *script = "f180|o180|f180|o0|f180|o180|f180|o0|f180|o180|f180|o0";
// char *script = "o90|o180|o270|o0";

// ROBOT geometry
#define R 15.5 // radius of the wheel
#define LENGTH_TO_ROTATION_SCALE 0.95 // to be adjusted based on test
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
#define CMD_COMPLETE_STABLE_CYCLES 20
#define POSITION_ERROR_THRESHOLD 5.0f
#define ANGLE_THRESHOLD 2.0f
#define BIGGEST_WALL_DISTANCE_THRESHOLD 80.0f 
#define SMALLEST_WALL_DISTANCE_THRESHOLD 15
#define DESIRED_WALL_DISTANCE 50.0f
#define DELAY_BETWEEN_CMD_VALUE 10
#define KP_LIDAR 0.1
#define KI_LIDAR 0.01

// Global objects
mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B, EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(15.5, 82); 
mtrn3100::PIDController motor1_encoder_position_controller(100, 0.01, 0); 
mtrn3100::PIDController motor2_encoder_position_controller(100, 0.1, 0);
mtrn3100::PIDController yaw_controller(2, 0.5, 0.1);
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

float target_motion_rotation_radians = 0; // Target motion rotation in radians, calculated based on the target distance and robot specifications

float curr_X = 0; // Current X position of the robot based on odometry, updated in each loop iteration
float curr_Y = 0;
float current_angle = 0;

int lidar_left = 0;
int lidar_front = 0;
int lidar_right = 0;

int delay_between_cmd = 0;

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
    yaw_controller.zeroAndSetTarget(0,0);

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

    // Take next instruction
    curr_X = encoder_odometry.getX();
    curr_Y = encoder_odometry.getY();
    current_angle = mpu.getAngleZ();

    // modify the kinematic control target only when
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
                    target_distance = value * LENGTH_TO_ROTATION_SCALE; // mm
                    target_angle = current_angle;
                    motor1_encoder_position_controller.setZeroRef(encoder.getLeftRotation());
                    motor2_encoder_position_controller.setZeroRef(encoder.getRightRotation());
                    yaw_controller.zeroAndSetTarget(0,target_angle);
                break;
                case 'o':
                    target_distance = 0;
                    target_angle = value;
                    motor1_encoder_position_controller.setZeroRef(encoder.getLeftRotation());
                    motor2_encoder_position_controller.setZeroRef(encoder.getRightRotation());
                    yaw_controller.zeroAndSetTarget(0,target_angle);

                    sprintf(monitor_buffer, "t_a: %d, curr: %d", (int) target_angle, (int) current_angle);
                    show_one_line_monitor(monitor_buffer);              
                break;
                default:
                    Serial.print("Invalid command: ");
                    Serial.println(c);
                    while(true) {}
                break;
            }
            prev_cmd = c; 
            commands.next();
        }        
        target_motion_rotation_radians = (target_distance ) / R;
        delay_between_cmd = DELAY_BETWEEN_CMD_VALUE;
        // delay for a while after one cmd is executed
    }

    if (cmd_sequence_completion_FLAG) {
        Serial.println(F("[INFO] Command sequence completed. The robot stops."));
        show_one_line_monitor("Command sequence completed. ROBOT stopped");
        motor1.setPWM(0);
        motor2.setPWM(0);
        while(true){}
    }
    
    MovementControl();

    loop_counter++;

    if (loop_counter > 30000) {
        // Serial.println("[INFO]: Loop count exceeded 30000, resetting to 0.");
        loop_counter = 0;
    }

    // Stop the robot
    while (cmd_sequence_completion_FLAG) {}

    delay(10);
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

// TODO
bool is_this_cmd_completed() {
    static int stable_counter = 0;
    bool completed = false;
    
    if (prev_cmd == 'f') {
        float delta_x = curr_X - previous_X;
        float delta_y = curr_Y - previous_Y;
        float distance_traveled = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
        float position_error = target_distance - distance_traveled;
        
        // Standard completion criteria OR emergency wall detection
        if (position_error <= POSITION_ERROR_THRESHOLD) {
            stable_counter++;
        } else {
            // stable_counter = 0;
            // stable_counter -= 1;
        }
        
        completed = (stable_counter >= CMD_COMPLETE_STABLE_CYCLES);
        
        if (completed) {
            sprintf(monitor_buffer, "Stopped by wall at %.0fmm", distance_traveled);
            show_one_line_monitor(monitor_buffer);
        }
        
    } else if (prev_cmd == 'o') {
        if (abs(target_angle-current_angle) <= ANGLE_THRESHOLD ) {
            stable_counter++;
        } else {
            // stable_counter = 0;
            // stable_counter -= 1;
        }
        completed = (stable_counter >= CMD_COMPLETE_STABLE_CYCLES);
    }
    
    if (completed) {
        stable_counter = 0;
        delay(10);
    }
    return completed;
}

// this function shows one character array on the OLED display
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

// Enhanced LIDAR reading with filtering and error handling
void updateLidarReadings() {
    lidar_left = sensor1.readRangeSingleMillimeters();
    lidar_front = sensor2.readRangeSingleMillimeters();
    lidar_right = sensor3.readRangeSingleMillimeters();
}

// Improved wall-following control with PD controller
float calculateWallFollowingCorrection() {
    float correction = 0.0f;
    bool wall_detect_flag = false;
    if (prev_cmd != 'f') return 0.0f;  // Only apply during forward motion

    float theta_rad = (target_angle - current_angle) * PI / 180.0 ;

    if ((lidar_left < BIGGEST_WALL_DISTANCE_THRESHOLD) && (lidar_left > SMALLEST_WALL_DISTANCE_THRESHOLD)){
        correction += DESIRED_WALL_DISTANCE - (lidar_left * cos(theta_rad));
        wall_detect_flag = true;
    } 

    if ((lidar_right < BIGGEST_WALL_DISTANCE_THRESHOLD) && (lidar_right > SMALLEST_WALL_DISTANCE_THRESHOLD)){
        correction -= DESIRED_WALL_DISTANCE - (lidar_right * cos(theta_rad));
        wall_detect_flag = true;
    } 

    if (!wall_detect_flag){
        correction = 0;
    }
    
    return correction * KP_LIDAR;
}

// Enhanced forward movement with dynamic speed control
void forward_Update_Target() {
    updateLidarReadings();
     
    // Calculate wall-following correction
    float lidar_correction = calculateWallFollowingCorrection();
    
    // Dynamic speed control based on front distance
    float speed_factor = 1.0f;

    motor1_encoder_position_controller.setTarget(
        (target_motion_rotation_radians * speed_factor)
    );
    motor2_encoder_position_controller.setTarget(
        (-target_motion_rotation_radians * speed_factor)
    );
}


// void turn_Update_Target() {
//     // float ratio_by_experiment = 0.046924;
//     yaw_controller.setTarget(target_angle);
// }

// Replace your main loop's movement control section with this:
void MovementControl() {

    // Execute movement based on current command
    if (prev_cmd == 'f') {
        forward_Update_Target();
        // Compute and apply motor outputs
        motor1_encoder_position_controller_output = motor1_encoder_position_controller.compute(encoder.getLeftRotation())-yaw_controller.compute(current_angle);
        motor2_encoder_position_controller_output = motor2_encoder_position_controller.compute(encoder.getRightRotation())-yaw_controller.compute(current_angle);
    } else if (prev_cmd == 'o') {
        // turn_Update_Target();
        // Compute and apply motor outputs
        // motor1_encoder_position_controller_output = -yaw_controller.compute(current_angle);
        // motor2_encoder_position_controller_output = -yaw_controller.compute(current_angle);
        motor1_encoder_position_controller_output = -20 * (target_angle - current_angle);
        motor2_encoder_position_controller_output = -20 * (target_angle - current_angle);
    }
    
    

    if (delay_between_cmd > 0){
        delay_between_cmd --;
        motor1_encoder_position_controller_output = 0;
        motor2_encoder_position_controller_output = 0;
    }
    
    motor1.setPWM(motor1_encoder_position_controller_output);
    motor2.setPWM(motor2_encoder_position_controller_output);
    Serial.print("motor1_encoder_position_controller_output: ");
    Serial.println(motor1_encoder_position_controller_output);
    Serial.print("motor2_encoder_position_controller_output: ");
    Serial.println(motor2_encoder_position_controller_output);
}

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