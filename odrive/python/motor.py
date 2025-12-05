import can
import struct
import os
import atexit
import time
import math as m
import json

class Motor:
    def __init__(self, id, name, bus):
        self.id = id
        self.name = name
        self.bus = bus
        with open('flat_endpoints.json', 'r') as f:
            self.endpoints = json.load(f)["endpoints"]
        self.format_lookup = {
            'bool': '?',
            'uint8': 'B', 'int8': 'b',
            'uint16': 'H', 'int16': 'h',
            'uint32': 'I', 'int32': 'i',
            'uint64': 'Q', 'int64': 'q',
            'float': 'f'
        }
        # for msg in self.bus:
        #     if msg.arbitration_id == (self.id << 5 | 0x01): # 0x01: Heartbeat
        #         error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        #         if error
        #         print(f"Error: {error} State: {state} Result: {result} Trajectory: {traj_done}")

        print(f"Motor {self.id} \"{self.name}\" initialized")

    def set_speed_limit(self, value):
        self.write_config("axis0.controller.config.vel_limit", value)

    def set_torque_limit(self, value):
        self.write_config("axis0.controller.config.vel_limit", value)

    def write_config(self, path, value):
        OPCODE_WRITE = 0x01

        # Convert path to endpoint ID
        endpoint_id = self.endpoints[path]['id']
        endpoint_type = self.endpoints[path]['type']

        # Send write command
        self.bus.send(can.Message(
            arbitration_id=(self.id << 5 | 0x04), # 0x04: RxSdo
            data=struct.pack('<BHB' + self.format_lookup[endpoint_type], OPCODE_WRITE, endpoint_id, 0, value),
            is_extended_id=False
        ))

    def read_config(self, path):
        OPCODE_READ = 0x00
        # Convert path to endpoint ID
        endpoint_id = self.endpoints[path]['id']
        endpoint_type = self.endpoints[path]['type']

        # Send read command
        self.bus.send(can.Message(
            arbitration_id=(self.id << 5 | 0x04), # 0x04: RxSdo
            data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
            is_extended_id=False
        ))
        # Await reply
        for msg in self.bus:
            if msg.is_rx and msg.arbitration_id == (self.id << 5 | 0x05): # 0x05: TxSdo
                return_value = struct.unpack_from('<' + self.format_lookup[endpoint_type], msg.data)
                print(f"received: {return_value}")
                break


    def set_control_mode(self, control_mode, input_mode):
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

        # for msg in self.bus:
        #     if msg.arbitration_id == (self.id << 5 | 0x01): # 0x01: Heartbeat
        #         error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
        #         print(f"Error: {error} State: {state} Result: {result} Trajectory: {traj_done}")

        for msg in self.bus:
            if msg.arbitration_id == (self.id << 5 | 0x01): # 0x01: Heartbeat
                _, res, _, _ = struct.unpack('<IBBB', bytes(msg.data[:7]))
                if res == code: 
                    print(f"Motor {self.id} \"{self.name}\" set to mode {state}")
                    break

    ###### Set Velocity
    def set_speed(self, speed, torque=0.0):
        ###### Set Velocity
        self.bus.send(can.Message(
            arbitration_id=(self.id << 5 | 0x0d), # 0x0d: Set_Input_Vel
            data=struct.pack('<ff', speed, torque), 
            is_extended_id=False
        ))
        print(f"Motor {self.id} \"{self.name}\" Speed: {speed} Torque: {torque}")

    ###### Set Position
    def set_position(self, position, speed=0, torque=0):
        self.bus.send(can.Message(
            arbitration_id=(self.id << 5 | 0x0c), # 0x0c: Set_Input_Pos
            data=struct.pack('<fhh', position, speed, torque), 
            is_extended_id=False
        ))
        print(f"Motor {self.id} \"{self.name}\" Position: {position} Speed: {speed} Torque: {torque}")

    def stop(self):
        self.set_speed(0)

    def get_position(self):
        # Print encoder feedback
        for msg in self.bus:
            if msg.arbitration_id == (self.id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
                pos, vel = struct.unpack('<ff', bytes(msg.data))
                print(f"Motor {self.id} \"{self.name}\" Pos: {pos:.3f} Vel: {vel:.3f}")
                return pos, vel
