from motor import Motor
import can
import struct
import os
import atexit
import time
import math as m

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
# motors.append(Motor(2,"Top", bus))


# Examples
for motor in motors:
    motor.set_state("CLOSED_LOOP_CONTROL")
    motor.set_control_mode("POSITION_CONTROL", "TRAP_TRAJ")
    motor.set_speed_limit(float("inf"))
    motor.write_config("axis0.trap_traj.config.vel_limit", 60)
    motor.write_config("axis0.trap_traj.config.accel_limit", 40)
    motor.write_config("axis0.trap_traj.config.decel_limit", 40)
    motor.write_config("axis0.controller.config.inertia", 0)
    motor.set_position(0)

# i = 0
while(1):
    motors[0].set_position(-3.5)
    time.sleep(2)
    motors[0].set_position(3.5)
    time.sleep(2)
    # motors[0].set_position(m.sin(i) * 10)
    # i = i+ 0.01
    # time.sleep(0.02)

# t = 0
# while (1):
#     t = t + 0.001
#     if t > 2 * m.pi:
#         t = 0
#     speed = m.sin(t) * 20

#     motors[0].set_speed(speed)
#     time.sleep(0.003)

# for speed in range(1,20,3):
#     for motor in motors:
#         motor.set_speed(speed)
#     time.sleep(1)

# for i in range(100):
#     for motor in motors:
#         pos, vel = motor.get_position()

# for motor in motors:
#     motor.stop()


