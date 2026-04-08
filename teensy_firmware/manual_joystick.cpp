#include <Arduino.h>
#include <RoboClaw.h>
#include <Servo.h>

#define ROBOCLAW_BAUD 115200
#define PC_BAUD 115200

#define ROBOCLAW_ADDRESS_1 0x82   // Linear Actuators
#define ROBOCLAW_ADDRESS_2 0x83   // Vibe and Excav
#define HOPPER_SERVO_PIN 9  // placeholder
#define KRAKEN_RIGHT_PIN 13  // placeholder
#define KRAKEN_LEFT_PIN 14   // placeholder

// NEVER EVER TOUCH PINS 33 AND 34 THEY'RE JUMPERS NOW

RoboClaw roboclaw_1(&Serial1, 10000); 
RoboClaw roboclaw_2(&Serial2, 10000);
Servo hopper_latch;
Servo kraken_right;
Servo kraken_left;

unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 20; // 50Hz

char rx_buffer[128];
int rx_index = 0;

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

// Handles the execution of a completely received line
void execute_command(const char* cmd) {

    int left, right, excav, la_extend, la_retract, vib, hop_latch;
    // left, right, excav, la_extend, la_retract, vib, hop_latch
    if (sscanf(cmd, "[%d %d %d %d %d %d %d]", &left, &right, &excav, &la_extend, &la_retract, &vib, &hop_latch) == 7) {
        // invert any axes
        left *= -1;
        right *= -1;
        excav *= -1;

        // apply 0-127 scale to excav, 64 is stop
        excav = 64 + (excav * 63);

        // apply 0-180 scale to drivetrain, 90 is stop
        left = 90 + (left * 90);
        right = 90 + (right * 90);

        // process button inputs
        int la_val, vibe_val, hop_val = 0;
        la_val = (la_extend == 1 && la_retract == 0) ? 126 : 64;
        la_val = (la_retract == 1 && la_extend == 0) ? 1 : 64;
        vibe_val = (vibe_val == 1) ? 126 : 64;
        hop_val = (hop_latch == 1) ? 0 : 180;

        // Address 1: Linear Actuators
        roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, la_val);
        roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, la_val);
        
        // Address 2: Vibe and Excav
        roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, vibe_val);
        roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, excav);
        
        hopper_latch.write(hop_val);
        kraken_left.write(left);
        kraken_right.write(right);

        digitalWrite(LED_BUILTIN, HIGH);
    }
}

void loop() {
    // receive commands
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n') {
            // End of line reached, terminate string and execute
            rx_buffer[rx_index] = '\0'; 
            execute_command(rx_buffer);
            rx_index = 0; // Reset buffer for the next message   
        }
    }
}