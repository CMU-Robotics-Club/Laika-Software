import can
import json
import os
import atexit
import struct
import time

node_id = 2 # ID of Odrive

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

###### Get Parameter file
# import config file
with open('flat_endpoints.json', 'r') as f:
    endpoint_data = json.load(f)
    endpoints = endpoint_data['endpoints']
# -- start definitions
OPCODE_READ = 0x00
OPCODE_WRITE = 0x01

# See https://docs.python.org/3/library/struct.html#format-characters
format_lookup = {
    'bool': '?',
    'uint8': 'B', 'int8': 'b',
    'uint16': 'H', 'int16': 'h',
    'uint32': 'I', 'int32': 'i',
    'uint64': 'Q', 'int64': 'q',
    'float': 'f'
}

###### Get Version
# Send read command
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x00), # 0x00: Get_Version
    data=b'',
    is_extended_id=False
))

# Await reply
for msg in bus:
    if msg.is_rx and msg.arbitration_id == (node_id << 5 | 0x00): # 0x00: Get_Version
        break

_, hw_product_line, hw_version, hw_variant, fw_major, fw_minor, fw_revision, fw_unreleased = struct.unpack('<BBBBBBBB', msg.data)

# If one of these asserts fail, you're probably not using the right flat_endpoints.json file
assert endpoint_data['fw_version'].partition('-')[0] == f"{fw_major}.{fw_minor}.{fw_revision}"
assert endpoint_data['hw_version'] == f"{hw_product_line}.{hw_version}.{hw_variant}"
print(f"{fw_major}.{fw_minor}.{fw_revision}")
print(f"{hw_product_line}.{hw_version}.{hw_variant}")

###### Write Paramater
path = 'axis0.controller.config.vel_limit'
value_to_write = float("inf")

# Convert path to endpoint ID
endpoint_id = endpoints[path]['id']
endpoint_type = endpoints[path]['type']

# Send write command
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
    data=struct.pack('<BHB' + format_lookup[endpoint_type], OPCODE_WRITE, endpoint_id, 0, value_to_write),
    is_extended_id=False
))

# On firmware 0.6.11 or newer, the ODrive sends a confirmation for write
# requests, so we insert a small delay so that the response doesn't get confused
# for a response for the read request.
time.sleep(0.1)

###### Read Paramater
path = 'axis0.controller.config.vel_limit'

# Convert path to endpoint ID
endpoint_id = endpoints[path]['id']
endpoint_type = endpoints[path]['type']

# Flush CAN RX buffer so there are no more old pending messages
while not (bus.recv(timeout=0) is None): pass

# Send read command
bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
    data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
    is_extended_id=False
))

# Await reply
for msg in bus:
    if msg.is_rx and msg.arbitration_id == (node_id << 5 | 0x05): # 0x05: TxSdo
        break

# Unpack and print reply
_, _, _, return_value = struct.unpack_from('<BHB' + format_lookup[endpoint_type], msg.data)
print(f"received: {return_value}")

###### Run function on odrive
path = "save_configuration"

# Convert path to endpoint ID
endpoint_id = endpoints[path]['id']

bus.send(can.Message(
    arbitration_id=(node_id << 5 | 0x04), # 0x04: RxSdo
    data=struct.pack('<BHB', OPCODE_WRITE, endpoint_id, 0),
    is_extended_id=False
))
