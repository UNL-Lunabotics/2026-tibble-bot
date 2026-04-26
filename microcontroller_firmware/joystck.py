from inputs import get_gamepad
import threading
import serial
import time

class XboxController(object):
    MAX_TRIG_VAL = 255.0 

    def __init__(self):
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def read(self):
        left_drive = self.LeftJoystickY
        right_drive = self.RightJoystickY
        excav = self.RightTrigger
        la_retract = self.X
        la_extend = self.B
        vib = self.A
        hop_latch = self.Y
        
        return [left_drive, right_drive, excav, la_retract, la_extend, vib, hop_latch]

    def _monitor_controller(self):
        while True:
            events = get_gamepad()
            for event in events:
                # --- LEFT JOYSTICK ---
                if event.code == 'ABS_Y':
                    # Math maps 0 to -0.99, 127 to 0.0, and 255 to 1.0
                    self.LeftJoystickY = (event.state - 127) / 128.0
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = (event.state - 127) / 128.0
                
                # --- RIGHT JOYSTICK (Mapped to Z and RZ in this Linux driver) ---
                elif event.code == 'ABS_RZ':
                    self.RightJoystickY = (event.state - 127) / 128.0 
                elif event.code == 'ABS_Z':
                    self.RightJoystickX = (event.state - 127) / 128.0 

                # --- TRIGGERS (Mapped to BRAKE and GAS) ---
                elif event.code == 'ABS_BRAKE':
                    self.LeftTrigger = event.state / XboxController.MAX_TRIG_VAL 
                elif event.code == 'ABS_GAS': 
                    self.RightTrigger = event.state / XboxController.MAX_TRIG_VAL 

                # --- BUTTONS ---
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_NORTH':
                    self.X = event.state 
                elif event.code == 'BTN_WEST':
                    self.Y = event.state 
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state

if __name__ == '__main__':
    joy = XboxController()
    
    # UNCOMMENT WHEN ARDUINO IS PLUGGED IN
    teensy = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    time.sleep(2) # Give Arduino time to reset after serial connection

    while True:
        state = joy.read()
        
        formatted_state = [round(val, 2) for val in state]
        print(formatted_state)
        
        command_string = f"{formatted_state[0]},{formatted_state[1]},{formatted_state[2]},{formatted_state[3]},{formatted_state[4]},{formatted_state[5]},{formatted_state[6]}\n"
        teensy.write(command_string.encode('utf-8'))
        
        time.sleep(0.05)