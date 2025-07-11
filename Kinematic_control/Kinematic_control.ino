#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include <SPI.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <stdio.h>

MPU6050 mpu(Wire);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(15.5, 82); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH

/*************** SCREEN *******************/

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Mode 1: Show data on the screen
static float debug_float_1 = 123;
static float debug_float_2 = 456;
static float debug_float_3 = 789.123;
static float debug_float_4 = 4;
static float debug_float_5 = 5;
static float debug_float_6 = 6;
static float debug_float_7 = 7;
static float debug_float_8 = 8;
static float debug_float_9 = 9;
static float debug_float_10 = 10;
static float debug_float_11 = 11;
static float debug_float_12 = 12;
static float debug_float_13 = 13;
static float debug_float_14 = 14;
static float debug_float_15 = 15;
static float debug_float_16 = 16;

float* values[] = {
    &debug_float_1, &debug_float_2, &debug_float_3, &debug_float_4,
    &debug_float_5, &debug_float_6, &debug_float_7, &debug_float_8,
    &debug_float_9, &debug_float_10, &debug_float_11, &debug_float_12,
    &debug_float_13, &debug_float_14, &debug_float_15, &debug_float_16
};

int loop_counter = 0;

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
    delay(2000); // Pause for 2 seconds

    // Clear the buffer
    display.clearDisplay();

    

    
    // delay(50);  // 模拟
    screen_mode_1();

}

void loop() {

    delay(5);

    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

    mpu.update();



    if (loop_counter % 10 == 0) {
        Serial.print("[INFO]: Loop count");
        Serial.print(loop_counter);
        Serial.println();
    }



    loop_counter++;
    debug_float_1 = loop_counter;

    if (loop_counter > 30000) {
        Serial.println("[INFO]: Loop count exceeded 60000, resetting to 0.");
        loop_counter = 0;
    }

    if (loop_counter % 1000 == 0) {
            Serial.println("[INFO]: screen mode 1 triggered");
            screen_mode_1();
    }

}

// divide the screen into 2 parts into 13 zones to show data
// it takes around 100ms
void screen_mode_1(){

    // // Serial.println("[INFO] screen mode 1");
    // // auto new line 
    display.clearDisplay();
    display.setTextSize(1);                 // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);    // Draw white text
    display.setCursor(0,0);                 // Start at top-left corner
    display.cp437(true);                    // Use full 256 char 'Code Page 437' font

    
    char temp[16];  // 足够放下 10 字符 + 结束符

    for (int i = 0; i < 16; i++) {
        dtostrf(*values[i], 10, 3, temp);  // 直接右对齐，总长度10

        for (int j = 0; j < 10; j++) {
            display.write(temp[j]);
        }

        if (i % 2 == 0) {
            display.write('|');
        }
    }
    display.display();
    delay(10); // what is this delay for?

}

// // mode 2: Radar map, has anyone seen Dragon Ball?
// void screen_mode_2(){
// }

// // mode 3: Show obstacles around the robot
// void screen_mode_3(){
// }

// // mode 4: Smile for me, along with a series of animations
// void screen_mode_4(){
// }