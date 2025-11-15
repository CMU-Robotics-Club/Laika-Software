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

    def set_mode(self, control_mode, input_mode):
        if control_mode == "VOLTAGE_CONTROL":
            control_code = 0x00
        elif control_mode == "TORQUE_CONTROL":
            control_code = 0x01
        elif control_mode == "VELOCITY_CONTROL":
            control_code = 0x02
        elif control_mode == "POSITION_CONTROL":
            control_code = 0x03
        else:
            print("Error: Control Mode not recognized!")
            exit(1)

        if input_mode == "INACTIVE":
            input_code = 0x00
        elif input_mode == "PASSTHROUGH":
            input_code = 0x01
        elif input_mode == "VEL_RAMP":
            input_code = 0x02
        elif input_mode == "POS_FILTER":
            input_code = 0x03
        elif input_mode == "TRAP_TRAJ":
            input_code = 0x05
        elif input_mode == "TORQUE_RAMP":
            input_code = 0x06
        elif input_mode == "TUNING":
            input_code = 0x08
        else:
            print("Error: Input Mode not recognized!")
            exit(1)

        self.bus.send(can.Message(
            arbitration_id=(self.id << 5 | 0x0b), # 0x0b: Set_Controller_Mode
            data=struct.pack('<II', control_code, input_code), 
            is_extended_id=False
        ))

        for msg in bus:
            if msg.arbitration_id == (self.id << 5 | 0x01): # 0x01: Heartbeat
                _, res, _, _ = struct.unpack('<IBBB', bytes(msg.data[:7]))
                print(res)
                # if res == code: 
                he, hu = struct.unpack('<ii', bytes(msg.data))
                print(f"Motor {self.id} \"{self.name}\" set to mode {he} {hu}")
                break


    def set_state(self, state):
        if state == "CLOSED_LOOP_CONTROL":
            code = 0x08 # 8: AxisState.CLOSED_LOOP_CONTROL
        elif state == "MOTOR_CALIBRATION":
            code = 0x04
        else:
            print("Error: State not recognized!")
            exit(1)
        self.bus.send(can.Message(
            arbitration_id=(self.id << 5 | 0x07), # 0x07: Set_Axis_State
            data=struct.pack('<I', code), # 8: AxisState.CLOSED_LOOP_CONTROL
            is_extended_id=False
        ))

        for msg in bus:
            if msg.arbitration_id == (self.id << 5 | 0x01): # 0x01: Heartbeat
                _, res, _, _ = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if res == code: 
                    print(f"Motor {self.id} \"{self.name}\" set to mode {state}")
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
        # motor.set_state("CLOSED_LOOP_CONTROL")
        motor.set_mode("TORQUE_CONTROL", "PASSTHROUGH")

    # for speed in range(1,20,3):
    #     for motor in motors:
    #         motor.set_speed(speed)
    #     time.sleep(1)

    # for i in range(100):
    #     for motor in motors:
    #         pos, vel = motor.get_position()

    for motor in motors:
        motor.stop()

