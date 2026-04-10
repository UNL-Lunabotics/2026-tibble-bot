#include <Arduino.h>
#include <RoboClaw.h>
#include <Servo.h>

#define ROBOCLAW_BAUD 115200
#define PC_BAUD 115200

#define ROBOCLAW_ADDRESS_1 0x82   // Linear Actuators

const unsigned int LA_SPEED = 10000;   // just to see if theyll move - 64 is stop, set to 1 for min
const unsigned int LA_POS = 15000;

unsigned int time_limit = 60 * 1000; // seconds

RoboClaw roboclaw_1(&Serial1, 10000); 

bool move_commanded = false;
unsigned long last_print_time = 0;

void setup() {
    roboclaw_1.begin(ROBOCLAW_BAUD);
    Serial.begin(PC_BAUD);

    pinMode(LED_BUILTIN, OUTPUT);
    
    // Ensure all motors are STOPPED on boot
    roboclaw_1.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, 64);
    roboclaw_1.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, 64);

    // Flash LED to show boot is complete
    delay(1000);
}

void loop() {
    if (!move_commanded) {
        // roboclaw_1.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, LA_SPEED);
        // roboclaw_1.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, LA_SPEED);
        roboclaw_1.SpeedAccelDeccelPositionM1(ROBOCLAW_ADDRESS_1, 1000, LA_SPEED, 1000, LA_POS, 0);
        roboclaw_1.SpeedAccelDeccelPositionM2(ROBOCLAW_ADDRESS_1, 1000, LA_SPEED, 1000, LA_POS, 0);

        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        
        move_commanded = true;
    }

    if (millis() - last_print_time >= 100) {
        last_print_time = millis();

        bool v1, v2;
        uint8_t status1, status2; 
        
        int32_t enc_la1 = roboclaw_1.ReadEncM1(ROBOCLAW_ADDRESS_1, &status1, &v1);
        int32_t enc_la2 = roboclaw_1.ReadEncM2(ROBOCLAW_ADDRESS_1, &status2, &v2);
        
        Serial.printf("ENC1: %d, ENC2: %d\n", enc_la1, enc_la2);
    }
}