#include <Arduino.h>
#include <RoboClaw.h>
#include <Servo.h>

#define ROBOCLAW_BAUD 115200

#define ROBOCLAW_ADDRESS_1 0x82   // Linear Actuators
#define ROBOCLAW_ADDRESS_2 0x83   // Vibe and Excav
#define HOPPER_SERVO_PIN 9  // placeholder
#define KRAKEN_RIGHT_PIN 13  // placeholder
#define KRAKEN_LEFT_PIN 14   // placeholder

// bullshit PWM values
// const unsigned int LA_SPEED = 41100.0 * 0.00762;  // ticks/meter * 0.3 inches in meters
const unsigned int LA_SPEED = 1;   // just to see if theyll move - 64 is stop, set to 1 for min
const unsigned int HOP_SERVO_POS = 0; // closed
const unsigned int VIBE_SPEED = 100;
const unsigned int EXCAV_SPEED = 100;
const unsigned int DRIVETRAIN_SPEED = 100; // 90 is stop

unsigned int time_limit = 60 * 1000; // seconds

RoboClaw roboclaw_1(&Serial1, 10000); 
RoboClaw roboclaw_2(&Serial2, 10000);
Servo hopper_latch;
Servo kraken_right;
Servo kraken_left;

void setup() {
    roboclaw_1.begin(ROBOCLAW_BAUD);
    roboclaw_2.begin(ROBOCLAW_BAUD);

    hopper_latch.attach(HOPPER_SERVO_PIN);

    kraken_right.attach(KRAKEN_RIGHT_PIN);
    kraken_left.attach(KRAKEN_LEFT_PIN);

    pinMode(LED_BUILTIN, OUTPUT);
    
    // Ensure all motors are STOPPED on boot
    roboclaw_1.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, 64);
    roboclaw_1.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, 64);
    roboclaw_2.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, 64);
    roboclaw_2.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, 64);
    kraken_left.write(90);
    kraken_right.write(90);

    // Flash LED to show boot is complete
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
}

// goal
// drivetrain forward and LA's EXCAVATE, vibe motor on, Latch closed, Excav on (excav)
void loop() {
    // if (millis() >= time_limit) {
    if (false) {
        // set everything to rest or stop motors
        roboclaw_1.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, 64);
        roboclaw_1.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, 64);
        roboclaw_2.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, 64);
        roboclaw_2.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, 64);
        kraken_left.write(64);
        kraken_right.write(64);
    } else {
        // roboclaw_1.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, LA_SPEED);
        // roboclaw_1.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, LA_SPEED);
        roboclaw_2.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, VIBE_SPEED);
        roboclaw_2.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, EXCAV_SPEED);
        // kraken_left.write(DRIVETRAIN_SPEED);
        // kraken_right.write(DRIVETRAIN_SPEED);
        hopper_latch.write(HOP_SERVO_POS);
    }
}