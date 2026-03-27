#include <Arduino.h>
#include <RoboClaw.h>
#include <Servo.h>

#define ROBOCLAW_BAUD 115200

#define ROBOCLAW_ADDRESS_1 0x82   // Linear Actuators
#define ROBOCLAW_ADDRESS_2 0x83   // Vibe and Excav
#define HOPPER_SERVO_PIN 7  // placeholder
#define KRAKEN_RIGHT_PIN 8  // placeholder
#define KRAKEN_LEFT_PIN 9   // placeholder

// bullshit PWM values
const unsigned int LA_POS = 41100.0 * 0.00762;  // ticks/meter * 0.3 inches in meters
const unsigned int HOP_SERVO_POS = 180;
const int VIBE_SPEED = 127;
const int EXCAV_SPEED = 127;
const int DRIVETRAIN_SPEED = 127;

unsigned int time_limit = 30;

RoboClaw roboclaw(&Serial2, 10000); 
Servo hopper_latch;
Servo kraken_right;
Servo kraken_left;