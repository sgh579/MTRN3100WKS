#include "Encoder.hpp"
#include "Encoder2.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

#define MOT1PWM 11 // PIN 11 is motor1's PWM pin
#define MOT1DIR 12

#define MOT2PWM 9// PIN 9 is motor2's PWM pin
#define MOT2DIR 10

// TODO: create a struct for this, avoid using 2 separate motor objects
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

#define EN1_A 2 // PIN 2 is encoder1's interupt // verified
#define EN1_B 7

#define EN2_A  3// PIN 3 is encoder2's interupt
#define EN2_B  8

mtrn3100::Encoder encoder1(EN1_A, EN1_B);
mtrn3100::Encoder2 encoder2(EN2_A, EN2_B);

// this implementation is a PID controller based on location

// we need a new implementation that is unversial and can be used for speed control, path following, etc.

float kP_1 = 100;
float kI_1 = 0.01;
float kD_1 = 0;

float kP_2 = 100;
float kI_2 = 0.01;
float kD_2 = 0;

mtrn3100::PIDController controller1(kP_1, kI_1, kD_1); // Kp, Ki, Kd , motor1's position controller
mtrn3100::PIDController controller2(kP_2, kI_2, kD_2); // Kp, Ki, Kd , motor2's position controller

void setup() {
  Serial.begin(9600);

  float target_motion_length = 200; // 1000 mm, specified by task4
  float motion_length_to_rotation_scale = 1; // to be adjusted based on the motor and encoder specifications
  float r = 15.5;
  float target_motion_rotation_radians = (target_motion_length * motion_length_to_rotation_scale) / r  ;

  // target_motion_rotation_radians = 2.0f * M_PIF;

  controller1.zeroAndSetTarget(encoder1.getRotation(), target_motion_rotation_radians); 
  controller2.zeroAndSetTarget(encoder2.getRotation(), -target_motion_rotation_radians); // reverse it for vehicle's motion
}

void loop() {

  int controller1_output = controller1.compute(encoder1.getRotation());
  int controller2_output = controller2.compute(encoder2.getRotation());
  motor1.setPWM(controller1_output); 
  motor2.setPWM(controller2_output); 

  Serial.print("controller 1 output: ");
  Serial.println(controller1_output);
  Serial.print("controller 2 output: ");
  Serial.println(controller2_output);
  
  Serial.print("encoder1 count: ");
  Serial.println(encoder1.getCount());
  Serial.print("encoder2 count: ");
  Serial.println(encoder2.getCount());


  delay(5);  // loop interval, picked by random, to be adjusted based on the application requirements

}