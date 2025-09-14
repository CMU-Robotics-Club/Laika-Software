import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from math import pi

# TODO this should be moved to some global library
JOINT_KEYS = ["fr_lat", "fr_hip", "fr_knee", \
                "rr_lat", "rr_hip", "rr_knee", \
                "rl_lat", "rl_hip", "rl_knee", \
                "fl_lat", "fl_hip", "fl_knee",]

JOINT_TORQUE_TOPICS = {
    "fr_lat": "/fr_lat_torque_cmd",
    "fr_hip": "/fr_hip_torque_cmd",
    "fr_knee": "/fr_knee_torque_cmd",
    "rr_lat": "/rr_lat_torque_cmd",
    "rr_hip": "/rr_hip_torque_cmd",
    "rr_knee": "/rr_knee_torque_cmd",
    "rl_lat": "/rl_lat_torque_cmd",
    "rl_hip": "/rl_hip_torque_cmd",
    "rl_knee": "/rl_knee_torque_cmd",
    "fl_lat": "/fl_lat_torque_cmd",
    "fl_hip": "/fl_hip_torque_cmd",
    "fl_knee": "/fl_knee_torque_cmd",
}

torques = {
    "fr_lat": 20.0,
    "fr_hip": 3.0,
    "fr_knee": -3.0,
    "rr_lat": -20.0,
    "rr_hip": 3.0,
    "rr_knee": -3.0,
    "rl_lat": 20.0,
    "rl_hip": -3.0,
    "rl_knee": 3.0,
    "fl_lat": -20.0,
    "fl_hip": -3.0,
    "fl_knee": 3.0,
}

joint_gains = {
    "fr_lat": (30.0, 0.0),
    "fr_hip": (30.0, 0.0),
    "fr_knee": (1.0, 0.0),
    "rr_lat": (1.0, 0.0),
    "rr_hip": (10.0, 0.0),
    "rr_knee": (1.0, 0.0),
    "rl_lat": (1.0, 0.0),
    "rl_hip": (1.0, 0.0),
    "rl_knee": (1.0, 0.0),
    "fl_lat": (1.0, 0.0),
    "fl_hip": (1.0, 0.0),
    "fl_knee": (1.0, 0.0),
}

joint_targets = {
    "fr_lat": (0.0, 0.0),
    "fr_hip": (-pi/4, 0.0),
    "fr_knee": (0.0, 0.0),
    "rr_lat": (0.0, 0.0),
    "rr_hip": (0.0, 0.0),
    "rr_knee": (0.0, 0.0),
    "rl_lat": (0.0, 0.0),
    "rl_hip": (0.0, 0.0),
    "rl_knee": (0.0, 0.0),
    "fl_lat": (0.0, 0.0),
    "fl_hip": (0.0, 0.0),
    "fl_knee": (0.0, 0.0),
}

# torques = {
#     "fr_lat": 20.0,
#     "fr_hip": -3.0,
#     "fr_knee": 3.0,
#     "rr_lat": -20.0,
#     "rr_hip": -3.0,
#     "rr_knee": 3.0,
#     "rl_lat": 20.0,
#     "rl_hip": 3.0,
#     "rl_knee": -3.0,
#     "fl_lat": -20.0,
#     "fl_hip": 3.0,
#     "fl_knee": -3.0,
# }

class PositionController(Node):

    joint_torque_pubs = None
    joint_state_sub = None

    joint_states = None
    joint_targets = None

    controller_timer = None

    def __init__(self):
        super().__init__("position_controller")

        self.joint_torque_pubs = {}
        self.joint_states = {}
        self.joint_targets = {}
        for joint_key in JOINT_KEYS:
            self.joint_torque_pubs[joint_key] = self.create_publisher(Float64, JOINT_TORQUE_TOPICS[joint_key], 10)
            self.joint_states[joint_key] = (0.0, 0.0)
            self.joint_targets[joint_key] = joint_targets[joint_key]
        
        self.joint_state_sub = self.create_subscription(JointState, "/robot_joint_states", self.joint_state_callback, 10)
        
        self.controller_timer = self.create_timer(0.05, self.controller_callback)
    
    def joint_state_callback(self, msg):
        for i, joint_name in enumerate(msg.name):
            if joint_name in self.joint_states.keys():
                self.joint_states[joint_name] = (msg.position[i], msg.velocity[i])
    
    def controller_callback(self):
        for joint_key in JOINT_KEYS:
            current_pos, current_vel = self.joint_states[joint_key]
            target_pos, target_vel = self.joint_targets[joint_key]
            kp, kd = joint_gains[joint_key]

            tau = 0.0
            if joint_key in ["fr_lat", "fr_hip"]:
                tau += kp * (target_pos - current_pos)
                tau += kd * (target_vel - current_vel)
            else:
                tau = torques[joint_key]

            msg = Float64()
            msg.data = tau
            self.joint_torque_pubs[joint_key].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    position_controller = PositionController()
    rclpy.spin(position_controller)

    position_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
