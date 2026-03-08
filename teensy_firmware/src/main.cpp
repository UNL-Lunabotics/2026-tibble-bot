#include <Arduino.h>
#include <RoboClaw.h>
#include <Servo.h>

#define ROBOCLAW_BAUD (115200)
#define SERIAL_BAUD (9600)

// placeholders
#define ROBOCLAW_ADDRESS_1 (0x82)   // for linear actuators
#define ROBOCLAW_ADDRESS_2 (0x83)   // for excav and vibe
#define HOPPER_SERVO_PIN (7)        // placeholder

RoboClaw roboclaw_1(&Serial2, 10000);
RoboClaw roboclaw_2(&Serial2, 10000);
Servo hopper_latch;

unsigned long last_telemetry_time = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 20;

void setup() {
    Serial.begin(SERIAL_BAUD);
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.setTimeout(20); // ms

    roboclaw_1.begin(ROBOCLAW_BAUD);
    roboclaw_2.begin(ROBOCLAW_BAUD);

    hopper_latch.attach(HOPPER_SERVO_PIN);
 }

void loop() {
    // SEND TELEMETRY
    if (millis() - last_telemetry_time >= TELEMETRY_INTERVAL_MS)
    {
        last_telemetry_time = millis();

        bool valid1, valid2, valid3;

        int32_t la1_enc = (int32_t)roboclaw_1.ReadEncM1(ROBOCLAW_ADDRESS_1, NULL, &valid1);
        int32_t la2_enc = (int32_t)roboclaw_1.ReadEncM2(ROBOCLAW_ADDRESS_1, NULL, &valid2);
        int32_t excav_enc = (int32_t)roboclaw_2.ReadEncM2(ROBOCLAW_ADDRESS_2, NULL, &valid3);

        if (valid1 && valid2 && valid3)
        {
            Serial.printf("e %ld %ld %ld\n", la1_enc, la2_enc, excav_enc);
        }
    }



    // RECEIVE COMMANDS
    if (!Serial.available()) return;

    // DRAIN THE BUFFER
    // We want the VERY LAST complete command sent by the PC.
    // Everything before that is outdated lag.
    String latest_command = "";
    
    // Receive command and save it if it is valid
    while (Serial.available() > 0) {
        // limit how much we read per loop to prevent hanging if data comes in faster than we can read
        String temp_command = Serial.readStringUntil('\n');
        temp_command.trim();
    
        if (temp_command.length() > 0) {
            latest_command = temp_command;
        }
    }

    // If we didn't find a valid command after draining, exit.
    if (latest_command.length() == 0) return;



    // EXECUTE COMMANDS
    // EXECUTE ONLY THE LATEST COMMAND
    const char command_type = latest_command.charAt(0);
    const char* c_str_command = latest_command.c_str();

    // 'c' for command, 's' for stop, 'r' for reset encoders
    switch (command_type)
    {
        case 'c':
        {
            int la_1, la_2, vibe, excav, latch;
            int cmd_count = sscanf(c_str_command, "c %d %d %d %d %d", &la_1, &la_2, &vibe, &excav, &latch);
            // Serial.printf("Recieved left %d right %d\n", left_velocity, right_velocity);

            // segfault guard, need exactly 5 valid commands
            if (cmd_count == 5)
            {
                // directional scale where 0 is reverse, 64 is stop, and 127 is forward
                roboclaw_1.ForwardBackwardM1(ROBOCLAW_ADDRESS_1, la_1);
                roboclaw_1.ForwardBackwardM2(ROBOCLAW_ADDRESS_1, la_2);

                roboclaw_2.ForwardBackwardM1(ROBOCLAW_ADDRESS_2, vibe);
                roboclaw_2.ForwardBackwardM2(ROBOCLAW_ADDRESS_2, excav);

                // assumes latch is an angle
                hopper_latch.write(latch);

                digitalWrite(LED_BUILTIN, LOW); // LED is OFF when success
            }
            else
            {
                digitalWrite(LED_BUILTIN, HIGH); // error
            }
            
            break;
        }
        case 's':   // emergency full stop
        {
            roboclaw_1.ForwardM1(ROBOCLAW_ADDRESS_1, 0);
            roboclaw_1.ForwardM2(ROBOCLAW_ADDRESS_1, 0);
            roboclaw_2.ForwardM1(ROBOCLAW_ADDRESS_2, 0);
            roboclaw_2.ForwardM2(ROBOCLAW_ADDRESS_2, 0);
            break;
        }
        case 'r':
        {
            // Zero out linear actuator encoders to help combat drift
            roboclaw_1.SetEncM1(ROBOCLAW_ADDRESS_1, 0);
            roboclaw_1.SetEncM2(ROBOCLAW_ADDRESS_1, 0);

            // Flash the LED quickly to acknowledge the reset
            digitalWrite(LED_BUILTIN, LOW);
            delay(50);
            digitalWrite(LED_BUILTIN, HIGH);
            break;
        }
        default:
            break;
    }
}
