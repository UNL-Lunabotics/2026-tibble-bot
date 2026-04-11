#include <Arduino.h>
#include <RoboClaw.h>
#include <Servo.h>

#define ROBOCLAW_BAUD 115200
#define PC_BAUD 115200

#define ROBOCLAW_ADDRESS_1 0x82   // Linear Actuators
#define ROBOCLAW_ADDRESS_2 0x83   // Vibe and Excav
#define HOPPER_SERVO_PIN 9        // placeholder
#define KRAKEN_RIGHT_PIN 40       // placeholder
#define KRAKEN_LEFT_PIN 14        // placeholder

// NEVER EVER TOUCH PINS 33 AND 34 THEY'RE JUMPERS NOW

RoboClaw roboclaw_1(&Serial1, 10000); 
RoboClaw roboclaw_2(&Serial2, 10000);
Servo hopper_latch;
Servo kraken_right;
Servo kraken_left;

char rx_buffer[128];
int rx_index = 0;

void setup() {
    roboclaw_1.begin(ROBOCLAW_BAUD);
    roboclaw_2.begin(ROBOCLAW_BAUD);
    Serial.begin(PC_BAUD);

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

// NOTE: Removed 'const' so strtok can safely operate without crashing the Teensy
void execute_command(char* cmd) {
    float left = 0.0, right = 0.0, excav = 0.0;
    int la_extend = 0, la_retract = 0, vibe = 0, hop_latch = 0;

    // Parse values in the exact order Python sends them: 
    // left, right, excav, la_retract, la_extend, vib, hop_latch
    char *ptr = strtok(cmd, ",");
    if (ptr != NULL) left = atof(ptr);
    
    ptr = strtok(NULL, ",");
    if (ptr != NULL) right = atof(ptr);
    
    ptr = strtok(NULL, ",");
    if (ptr != NULL) excav = atof(ptr);
    
    ptr = strtok(NULL, ",");
    if (ptr != NULL) la_retract = atoi(ptr);
    
    ptr = strtok(NULL, ",");
    if (ptr != NULL) la_extend = atoi(ptr);
    
    ptr = strtok(NULL, ",");
    if (ptr != NULL) vibe = atoi(ptr);
    
    ptr = strtok(NULL, ",");
    if (ptr != NULL) hop_latch = atoi(ptr);

    // Invert axes
    left *= -1.0;
    right *= -1.0;
    excav *= -1.0;

    // Apply scaling math
    int excav_val = 64 + (int)(excav * 63);

    int left_us = 1500 + (int)(left * 500.0);
    int right_us = 1500 + (int)(right * 500.0);
    left_us = constrain(left_us, 1000, 2000);
    right_us = constrain(right_us, 1000, 2000);

    // Actuator logic correctly separated so they don't overwrite each other
    int la_val = 64; // Default to stop
    if (la_extend == 1 && la_retract == 0) {
        la_val = 126;
    } else if (la_retract == 1 && la_extend == 0) {
        la_val = 1;
    }

    int vibe_val = (vibe == 1) ? 126 : 64;
    int hop_val = (hop_latch == 1) ? 0 : 180;

    // Address 1: Linear Actuators
    roboclaw_1.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, la_val);
    roboclaw_1.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, la_val);
    
    // Address 2: Vibe and Excav
    roboclaw_2.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, excav_val);
    roboclaw_2.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, vibe_val);
    
    hopper_latch.write(hop_val);
    kraken_left.writeMicroseconds(left_us);
    kraken_right.writeMicroseconds(right_us);

    // Toggle LED to visually confirm parsing is working
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    
    // Print back to PC for debugging (Using proper %f for floats)
    Serial.printf("%.2f %.2f %.2f %d %d %d %d\n", left, right, excav, la_retract, la_extend, vibe, hop_latch);
}

void loop() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\r') continue; 
        
        if (c == '\n') {
            rx_buffer[rx_index] = '\0'; 
            execute_command(rx_buffer);
            rx_index = 0; 
        } else {
            rx_buffer[rx_index] = c;
            rx_index++;
            if (rx_index >= sizeof(rx_buffer) - 1) {
                rx_index = sizeof(rx_buffer) - 2; 
            }
        }
    }
}