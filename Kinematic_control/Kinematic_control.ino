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
    float r = 15.5; // wheel radius
    float target_motion_length = 180.0; // mm
    float target_motion_rotation_radians = target_motion_length / r;
//    target_motion_rotation_radians = (target_motion_length * motion_length_to_rotation_scale) / r  ;

    // target_motion_rotation_radians = 2.0f * M_PIF;

    motor1_encoder_position_controller.zeroAndSetTarget(encoder.getLeftRotation(), target_motion_rotation_radians); 
    motor2_encoder_position_controller.zeroAndSetTarget(encoder.getRightRotation(), target_motion_rotation_radians); // reverse it for vehicle's motion

    yaw_controller.zeroAndSetTarget(0, -90); // negative for CW

}

void loop() {

    // Read the sensors
    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
    mpu.update();

    float current_angle_z = mpu.getAngleZ();

//    float yaw_controller_output =  yaw_controller.compute(current_angle_z);

    motor1_encoder_position_controller.setTarget(target_motion_rotation_radians);
    motor2_encoder_position_controller.setTarget(target_motion_rotation_radians);

    int motor1_encoder_position_controller_output = motor1_encoder_position_controller.compute(encoder.getLeftRotation());
    int motor2_encoder_position_controller_output = motor2_encoder_position_controller.compute(encoder.getRightRotation());


    int speed1 = motor1_encoder_position_controller_output;
    int speed2 = motor2_encoder_position_controller_output;

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

    if (abs(encoder.getLeftRotation() - target_motion_rotation_radians) < 0.1 &&
    abs(encoder.getRightRotation() - target_motion_rotation_radians) < 0.1) {
    motor1.setPWM(0);
    motor2.setPWM(0);
	}

    delay(5);

}