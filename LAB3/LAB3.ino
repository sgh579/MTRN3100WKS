#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "BangBangController.hpp"

#define MOT1PWM 9 // PIN 9 is a PWM pin
#define MOT1DIR 10
mtrn3100::Motor motor(MOT1PWM,MOT1DIR);

#define EN_A 2 // PIN 2 is an interupt
#define EN_B 4
mtrn3100::Encoder encoder(EN_A, EN_B);

// mtrn3100::BangBangController controller(120,0); // speed, deadband
mtrn3100::PIDController controller(600, 1, 10); // Kp, Ki, Kd 


void setup() {
  Serial.begin(9600);
  controller.zeroAndSetTarget(encoder.getRotation(), 6.3); // Set the target as 2 Radians

}

void loop() {

  Serial.print("controller error: ");
  Serial.println(controller.getError());

  int controller_output = controller.compute(encoder.getRotation());
  
  motor.setPWM(controller_output); 
  Serial.print("controller output: ");
  Serial.println(controller_output);

  delay(50);

}
