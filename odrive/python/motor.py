import can
import struct
import os
import atexit
import time

class Motor:
    def __init__(self, id, name, bus):
        self.id = id
        self.name = name
        self.bus = bus
        print(f"Motor {self.id} \"{self.name}\" initialized")

    def set_mode(self, mode):
        if mode == "CLOSED_LOOP_CONTROL":
            state = 0x08 # 8: AxisState.CLOSED_LOOP_CONTROL
        else:
            print("Error: State not recognized!")
            exit(1)
        self.bus.send(can.Message(
            arbitration_id=(self.id << 5 | 0x07), # 0x07: Set_Axis_State
            is_extended_id=False
        ))

        # Wait for axis to enter closed loop control by scanning heartbeat messages
        for msg in bus:
            if msg.arbitration_id == (self.id << 5 | 0x01): # 0x01: Heartbeat
                _, state, _, _ = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if state == state: # 8: AxisState.CLOSED_LOOP_CONTROL
                    print(f"Motor {self.id} \"{self.name}\" set to mode {mode}")
                    break

    def set_speed(self, speed, torque=0.0):
        ###### Set Velocity
        self.bus.send(can.Message(
            arbitration_id=(self.id << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', speed, torque), 
            is_extended_id=False
        ))
        print(f"Motor {self.id} \"{self.name}\" speed: {speed}")

    def stop(self):
        self.set_speed(0)

    def get_position(self):
        # Print encoder feedback
        for msg in self.bus:
            if msg.arbitration_id == (self.id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
                pos, vel = struct.unpack('<ff', bytes(msg.data))
                print(f"Motor {self.id} \"{self.name}\" Pos: {pos:.3f} Vel: {vel:.3f}")
                return pos, vel



if __name__ == "__main__":
    ###### Check if windows or linux
    linux = True
    if os.name == 'nt':
        linux = False

    ###### Connect to CAN bus
    if linux:
        bus = can.interface.Bus(channel="can0", bitrate=250000, interface="socketcan")
    else:
        bus = can.interface.Bus(index=0, channel=0, interface="gs_usb", bitrate=250000)

    # Make sure CAN interface is closed when script exits
    bus.__enter__()
    atexit.register(lambda: bus.__exit__(None, None, None))

    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None): pass

    # add motors
    motors = []
    motors.append(Motor(1,"Bottom", bus))
    motors.append(Motor(2,"Top", bus))

    # Examples
    for motor in motors:
        motor.set_mode("CLOSED_LOOP_CONTROL")

    for speed in range(1,20,3):
        for motor in motors:
            motor.set_speed(speed)
        time.sleep(1)

    for i in range(100):
        for motor in motors:
            pos, vel = motor.get_position()

    for motor in motors:
        motor.stop()

