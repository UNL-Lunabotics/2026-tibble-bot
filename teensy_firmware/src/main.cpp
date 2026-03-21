#include <Arduino.h>
#include <RoboClaw.h>
#include <Servo.h>

#define ROBOCLAW_BAUD 115200
#define PC_BAUD 9600

#define ROBOCLAW_ADDRESS_1 0x82   // Linear Actuators
#define ROBOCLAW_ADDRESS_2 0x83   // Vibe and Excav
#define HOPPER_SERVO_PIN 7  // placeholder

RoboClaw roboclaw(&Serial2, 10000); 
Servo hopper_latch;

unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 20; // 50Hz

char rx_buffer[128];
int rx_index = 0;

void setup() {
    Serial.begin(PC_BAUD);
    roboclaw.begin(ROBOCLAW_BAUD);

    hopper_latch.attach(HOPPER_SERVO_PIN);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Ensure all motors are STOPPED on boot
    roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, 64);
    roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, 64);
    roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, 64);
    roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, 64);

    // Flash LED to show boot is complete
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
}

// Handles the execution of a completely received line
void execute_command(const char* cmd) {
    char command_type = cmd[0];

    switch (command_type) {
        case 'c': {
            int la1, la2, vibe, excav, latch;
            // Parse the 5 variables sent from the PC
            if (sscanf(cmd, "c %d %d %d %d %d", &la1, &la2, &vibe, &excav, &latch) == 5) {
                // Address 1: Linear Actuators
                roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, la1);
                roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, la2);
                
                // Address 2: Vibe and Excav
                roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, vibe);
                roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, excav);
                
                hopper_latch.write(latch);
            }
            break;
        }
        case 's': {
            // Emergency Stop
            roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, 64);
            roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, 64);
            roboclaw.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, 64);
            roboclaw.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, 64);
            break;
        }
        case 'r': {
            // Reset Encoders
            roboclaw.SetEncM1(ROBOCLAW_ADDRESS_1, 0);
            roboclaw.SetEncM2(ROBOCLAW_ADDRESS_1, 0);
            
            // Visual feedback that reset occurred
            digitalWrite(LED_BUILTIN, HIGH);
            delay(10);
            digitalWrite(LED_BUILTIN, LOW);
            break;
        }
    }
}

void loop() {
    // send telemtry
    if (millis() - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
        last_telemetry_time = millis();
        
        bool v1, v2, v3;
        uint8_t status; // Dummy variable to catch RoboClaw status flags safely
        
        int32_t enc_la1 = roboclaw.ReadEncM1(ROBOCLAW_ADDRESS_1, &status, &v1);
        int32_t enc_la2 = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS_1, &status, &v2);
        int32_t enc_excav = roboclaw.ReadEncM2(ROBOCLAW_ADDRESS_2, &status, &v3);

        // Only send back to PC if the RoboClaw actually returned valid data
        if (v1 && v2 && v3) {
            Serial.printf("e %ld %ld %ld\n", enc_la1, enc_la2, enc_excav);
        }
    }

    // receive commands
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n') {
            // End of line reached, terminate string and execute
            rx_buffer[rx_index] = '\0'; 
            execute_command(rx_buffer);
            rx_index = 0; // Reset buffer for the next message
            
        } else if (c != '\r') {
            // Add to buffer (ignoring carriage returns) and prevent overflow
            if (rx_index < (sizeof(rx_buffer) - 1)) {
                rx_buffer[rx_index++] = c;
            }
        }
    }
}