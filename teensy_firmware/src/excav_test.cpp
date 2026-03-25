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
const unsigned int LA_POS = 1;  // in ticks
const unsigned int HOP_SERVO_POS = 180;
const int VIBE_SPEED = 127;
const int EXCAV_SPEED = 127;
const int DRIVETRAIN_SPEED = 127;

unsigned int time_limit = 30;

RoboClaw roboclaw(&Serial2, 10000); 
Servo hopper_latch;
Servo kraken_right;
Servo kraken_left;

void setup() {
    roboclaw.begin(ROBOCLAW_BAUD);

    hopper_latch.attach(HOPPER_SERVO_PIN);
    hopper_latch.write(HOP_SERVO_POS);  // just close it

    kraken_right.attach(KRAKEN_RIGHT_PIN);
    kraken_left.attach(KRAKEN_LEFT_PIN);

    pinMode(LED_BUILTIN, OUTPUT);
    
    // Ensure all motors are STOPPED on boot
    roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, 64);
    roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, 64);
    roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, 64);
    roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, 64);
    kraken_left.write(64);
    kraken_right.write(64);

    // Flash LED to show boot is complete
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
}

// goal
// drivetrain forward and LA's lowered, vibe motor running, Latch closed, Excav Turning (excavation)
void loop() {
    if (millis() >= time_limit) {
        // set everything to rest or stop motors
        roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, 64);
        roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, 64);
        roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, 64);
        roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, 64);
        kraken_left.write(64);
        kraken_right.write(64);
    } else {
        roboclaw.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS_1, 1000, 5000, 5000, target_la1_ticks, 1);
        roboclaw.SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS_1, 1000, 5000, 5000, target_la2_ticks, 1);
        roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, VIBE_SPEED);
        roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, EXCAV_SPEED);
        kraken_left.write(DRIVETRAIN_SPEED);
        kraken_right.write(DRIVETRAIN_SPEED);
    }
}