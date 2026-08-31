#!/usr/bin/env python3
# THIS STUFF IS USING THE MODEL MADE FROM ISAAC LAB.
import threading
from typing import Optional
import os
from ament_index_python.packages import get_package_share_directory

import numpy as np
import torch

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState, InterfaceValue
from sensor_msgs.msg import JointState

JOINT_NAMES = ["hip_joint", "knee_joint"]

# From SLIDING_LEG_CFG.init_state.joint_pos
DEFAULT_JOINT_POS = {
    "hip_joint": 0.4,   # rad
    "knee_joint": 1.1,  # rad
}

# From ActionsCfg.leg_joint_pos
ACTION_SCALE = 0.6

# From LegJumpingEnvCfg.__post_init__: sim.dt * decimation
CONTROL_DT = (1.0 / 240.0) * 4.0  # = 1/60 s, 60 Hz

# URDF hard limits, shrunk by the same soft_joint_pos_limit_factor (0.8)
# Isaac Lab applied during training, so the policy never saw targets past
# this range.
_HARD_LIMITS = {
    "hip_joint": (-0.7853981633974483, 1.5707963267948966),
    "knee_joint": (0.0, 3.141592653589793),
}
SOFT_LIMIT_FACTOR = 0.8


def _soft_limits(name: str):
    # Calculating the soft limits from the hard ones
    # Basically provides a buffer region of clamping
    # This function just makes the range 80% smaller (or whatever soft limit factor is set to)
    lo, hi = _HARD_LIMITS[name]
    mid = 0.5 * (lo + hi)
    half_range = 0.5 * (hi - lo) * SOFT_LIMIT_FACTOR
    return mid - half_range, mid + half_range


SOFT_LIMITS = {name: _soft_limits(name) for name in JOINT_NAMES}


class RLPolicyNode(Node):
    def __init__(self, policy_path: str) -> None:
        super().__init__("rl_jump_policy_node")

        # Produced by Isaac Lab's play.py after training (it auto-exports
        # the actor to .../exported/policy.pt via export_policy_as_jit).
        self.policy = torch.jit.load(policy_path)
        self.policy.eval()

        # runtime state
        self.joint_pos = {n: DEFAULT_JOINT_POS[n] for n in JOINT_NAMES}
        self.joint_vel = {n: 0.0 for n in JOINT_NAMES}
        self.last_action = np.zeros(len(JOINT_NAMES), dtype=np.float32)
        self.have_state = False
        self.last_state_stamp = self.get_clock().now()
        self.enabled = False  # software e-stop / arm switch

        # Subscribe to joint states since we need them obviously
        self.state_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, 10
        )
        # publsih to the simple pid controller
        self.cmd_pub = self.create_publisher(
            DynamicJointState, "laika_simple_pid_controller/command", 10
        )

        self.timer = self.create_timer(CONTROL_DT, self._step)

        self.get_logger().info(
            f"RL policy node up. Waiting for /joint_states. "
            f"Control rate: {1.0 / CONTROL_DT:.1f} Hz"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self.joint_pos:
                self.joint_pos[name] = pos
                self.joint_vel[name] = vel
        self.have_state = True
        self.last_state_stamp = self.get_clock().now()

    def _build_observation(self) -> torch.Tensor:
        # Order MUST match ObservationsCfg.PolicyCfg term order.
        pos_rel = [self.joint_pos[n] - DEFAULT_JOINT_POS[n] for n in JOINT_NAMES]
        vel_rel = [self.joint_vel[n] for n in JOINT_NAMES]  # default vel is 0
        obs = np.concatenate([pos_rel, vel_rel, self.last_action]).astype(np.float32)
        return torch.from_numpy(obs).unsqueeze(0)  # add batch dim -> [1, 6]

    def _step(self) -> None:
        
        if not self.have_state:
            return  # no joint feedback yet

        stale = (self.get_clock().now() - self.last_state_stamp).nanoseconds > 0.2e9
        if stale:
            self.get_logger().warn(
                "Joint state feed is stale, skipping step.", throttle_duration_sec=1.0
            )
            return

        if not self.enabled:
            self._publish_targets(DEFAULT_JOINT_POS)
            return  # not armed yet

        obs = self._build_observation()

        with torch.no_grad():
            action = self.policy(obs).squeeze(0).numpy()

        self.last_action = action.astype(np.float32)

        targets = {}
        for i, name in enumerate(JOINT_NAMES):
            # Calcualting deltas here. actions are usually within [-1, 1] and multiplied by action scale to limit jerks
            raw_target = DEFAULT_JOINT_POS[name] + float(action[i]) * ACTION_SCALE
            lo, hi = SOFT_LIMITS[name]
            targets[name] = max(lo, min(hi, raw_target)) # CONSTRAINING OUTPUTS TO BE WITHIN THESE SOFT LIMITS

        self._publish_targets(targets)

    # -----------------------------------------------------------------------
    def _publish_targets(self, targets: dict) -> None:
        msg = DynamicJointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = JOINT_NAMES
        for name in JOINT_NAMES:
            iv = InterfaceValue()
            iv.interface_names = ["position"]
            iv.values = [targets[name]]
            msg.interface_values.append(iv)
        self.cmd_pub.publish(msg)

    # -----------------------------------------------------------------------
    def arm(self) -> None:
        """Call once the leg is confirmed to be at/near its default pose."""
        self.enabled = True
        self.get_logger().info("Policy ARMED — commands are now live.")

    def disarm(self) -> None:
        self.enabled = False
        self.get_logger().info("Policy DISARMED.")


def main(args: Optional[list] = None) -> None:
    import argparse

    rclpy.init(args=args)

    default_policy = os.path.join(
        get_package_share_directory('laika_control'),
        'policies',
        'policy.pt'
    )

    parser = argparse.ArgumentParser()
    parser.add_argument("--policy", help="Path to exported policy.pt", default=default_policy)
    parsed, _ = parser.parse_known_args()

    node = RLPolicyNode(parsed.policy)

    def wait_for_key(node):
        input("Press [ENTER] to arm the policy and start RL control...\n")
        node.arm()

    threading.Thread(target=wait_for_key, args=(node,), daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
