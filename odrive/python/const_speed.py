import can
import struct
import os
import atexit

node_ids = [1,2] # Add all odrives
speed = 0.0

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

for node_id in node_ids:
    ###### Put axis into closed loop control state
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
        data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

    # Wait for axis to enter closed loop control by scanning heartbeat messages
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x01): # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8: # 8: AxisState.CLOSED_LOOP_CONTROL
                break

    ###### Set Velocity
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
        data=struct.pack('<ff', speed, 0.0), # 1.0: velocity, 0.0: torque feedforward
        is_extended_id=False
    ))

# Print encoder feedback
for msg in bus:
    for node_id in node_ids:
        if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
            pos, vel = struct.unpack('<ff', bytes(msg.data))
            print(f"Odrive: {node_id} pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")

# General Format for sending CANSimple messages:
# bus.send(can.Message(
#     arbitration_id=(i << 5 | <message_type>), --> https://docs.odriverobotics.com/v/latest/manual/can-protocol.html#can-msg-set-input-vel
#     data=struct.pack('<ff', <param1>, <param2>), --> depends on message type
#                                                       ?	bool
#                                                       B	uint8
#                                                       b	int8
#                                                       H	uint16
#                                                       I	uint32
#                                                       f	float
#     is_extended_id=False
# ))
