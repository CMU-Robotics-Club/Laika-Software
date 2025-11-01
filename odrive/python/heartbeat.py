import can
import os
import atexit

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


###### Listen for heartbeat
node_id = 1 # ID of Odrive
cmd_id = 0x01 # heartbeat command ID
message_id = (node_id << 5 | cmd_id)

for msg in bus:
    print(msg.arbitration_id)
    print(msg.data)
    print(msg.dlc)
    # if msg.arbitration_id == message_id:
    #   error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
    #   print(error, state, result, traj_done)
    #   break
