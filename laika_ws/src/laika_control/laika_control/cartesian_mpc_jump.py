import sys
import select
import termios
import tty
import threading
import math
from typing import Tuple, Callable, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import scipy.sparse as sparse
import osqp


class TerminalKeyLogger(threading.Thread):
    """Background thread to monitor standard input for the spacebar."""
    def __init__(self, key_callback: Callable[[str], None]) -> None:
        super().__init__(daemon=True)
        self.key_callback = key_callback
        self.fd = sys.stdin.fileno()
        self.original_settings = termios.tcgetattr(self.fd)

    def run(self) -> None:
        try:
            tty.setcbreak(self.fd)
            while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not rlist:
                    continue
                key = sys.stdin.read(1)
                self.key_callback(key)
        finally:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_settings)


def get_jump_y(t_pred: float, t_curr: float) -> Tuple[float, float]:
    """
    Implements piecewise jumping with Horizon Masking to prevent Look-Ahead Braking.
    """
    hs = 0.25   # Start/Crouch height
    hr = 0.50   # Release height
    hp = 0.70   # Peak height
    g = 9.81

    vr = math.sqrt(2 * g * (hp - hr))
    a_push = g * (hp - hr) / (hr - hs)

    tr = vr / a_push
    t_flight = 2 * vr / g
    t_land = tr + t_flight - 0.125
    t_end = t_land + tr

    # 1. HORIZON MASKING: Blind the MPC during pushoff
    if 0 <= t_curr <= tr:
        y = hs + 0.5 * a_push * (t_pred**2)
        dy = a_push * t_pred
        
        if y > 0.72: 
            y = 0.72
            
        return -y, -dy

    # 2. STANDARD LOOK-AHEAD: For Flight and Landing
    if t_pred < 0:
        y = hr
        dy = 0.0
    elif t_pred <= t_land:
        y = hs
        dy = 0.0
    elif t_pred <= t_end:
        dt = t_pred - t_land
        y = hr - vr * dt + 0.5 * a_push * (dt**2)
        dy = -vr + a_push * dt
    else:
        y = hs
        dy = 0.0

    return -y, -dy


class LaikaMPC(Node):
    def __init__(self) -> None:
        super().__init__('laika_mpc_node')
        
        self.cmd_pub = self.create_publisher(Float64MultiArray, 'laika_cartesian_impedance_controller/command', 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        
        self.l1 = 0.35
        self.l2 = 0.41
        self.knee_offset = 0.04573
        self.m = 3.2     
        self.g = 9.81    

        self.N = 10      
        self.dt = 0.025  
        self.secs_elapsed = 0.0
        
        self.jump_start_time = -999.0 
        
        self.current_state = np.zeros(4)
        self.state_received = False
        
        self.setup_dynamics()
        self.setup_condensed_mpc()
        
        self.prob = osqp.OSQP()
        self.prob.setup(
            P=self.H, q=self.f_dummy, A=self.A_ineq, l=self.l_ineq, u=self.u_ineq, 
            warm_start=True, verbose=False
        )
        
        self.key_logger = TerminalKeyLogger(self.handle_keypress)
        self.key_logger.start()

        self.timer = self.create_timer(self.dt, self.mpc_loop)
        
        self.get_logger().info("LTV-MPC Node Started. Leg will crouch to -0.25m.")
        self.get_logger().info("Press SPACE to trigger the jump sequence!\n")

    def handle_keypress(self, key: str) -> None:
        if key == ' ':
            if self.secs_elapsed - self.jump_start_time > 1.0:
                self.jump_start_time = self.secs_elapsed

    def setup_dynamics(self) -> None:
        self.Ad = np.array([
            [1.0, 0.0, self.dt, 0.0],
            [0.0, 1.0, 0.0, self.dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        
        self.Bd = np.array([
            [(self.dt**2) / (2 * self.m), 0.0],
            [0.0, (self.dt**2) / (2 * self.m)],
            [self.dt / self.m, 0.0],
            [0.0, self.dt / self.m]
        ])
        
        self.gd = np.array([
            0.0, -self.g * (self.dt**2) / 2.0, 
            0.0, -self.g * self.dt
        ])

    def setup_condensed_mpc(self) -> None:
        n_x = 4
        n_u = 2
        
        self.A_bar = np.zeros((n_x * self.N, n_x))
        self.B_bar = np.zeros((n_x * self.N, n_u * self.N))
        self.G_bar = np.zeros(n_x * self.N)
        
        for i in range(self.N):
            row_start = i * n_x
            row_end = (i + 1) * n_x
            
            if i == 0:
                self.A_bar[row_start:row_end, :] = self.Ad
            else:
                self.A_bar[row_start:row_end, :] = self.Ad @ self.A_bar[(i-1)*n_x:i*n_x, :]
                
            for j in range(i + 1):
                col_start = j * n_u
                col_end = (j + 1) * n_u
                if i == j:
                    self.B_bar[row_start:row_end, col_start:col_end] = self.Bd
                else:
                    self.B_bar[row_start:row_end, col_start:col_end] = self.Ad @ self.B_bar[(i-1)*n_x:i*n_x, col_start:col_end]

            if i == 0:
                self.G_bar[row_start:row_end] = self.gd
            else:
                self.G_bar[row_start:row_end] = self.Ad @ self.G_bar[(i-1)*n_x:i*n_x] + self.gd

        Q_step = np.diag([10000.0, 10000.0, 100.0, 100.0])
        R_step = np.diag([1e-5, 1e-5])
        
        self.Q_bar = sparse.kron(sparse.eye(self.N), Q_step)
        self.R_bar = sparse.kron(sparse.eye(self.N), R_step)
        
        B_bar_sparse = sparse.csc_matrix(self.B_bar)
        H_dense = B_bar_sparse.T @ self.Q_bar @ B_bar_sparse + self.R_bar
        self.H = sparse.csc_matrix(H_dense)
        
        self.f_dummy = np.zeros(n_u * self.N)
        self.A_ineq = sparse.eye(n_u * self.N).tocsc()
        self.l_ineq = -500.0 * np.ones(n_u * self.N)
        self.u_ineq = 500.0 * np.ones(n_u * self.N)

    def joint_callback(self, msg: JointState) -> None:
        try:
            hip_idx = msg.name.index('hip_joint')
            knee_idx = msg.name.index('knee_joint')
        except ValueError:
            return

        th_h = msg.position[hip_idx]
        th_k = msg.position[knee_idx]
        dth_h = msg.velocity[hip_idx]
        dth_k = msg.velocity[knee_idx]

        th_k_eff = th_k + self.knee_offset

        x = self.l1 * math.cos(th_h) - self.l2 * math.cos(th_h - th_k_eff)
        y = -self.l1 * math.sin(th_h) + self.l2 * math.sin(th_h - th_k_eff)

        j00 = -self.l1 * math.sin(th_h) + self.l2 * math.sin(th_h - th_k_eff)
        j01 = -self.l2 * math.sin(th_h - th_k_eff)
        j10 = -self.l1 * math.cos(th_h) + self.l2 * math.cos(th_h - th_k_eff)
        j11 = -self.l2 * math.cos(th_h - th_k_eff)

        dx = j00 * dth_h + j01 * dth_k
        dy = j10 * dth_h + j11 * dth_k

        self.current_state = np.array([x, y, dx, dy])
        self.state_received = True

    def mpc_loop(self) -> None:
        if not self.state_received:
            return

        x0 = self.current_state
        self.secs_elapsed += self.dt

        X_ref = np.zeros(4 * self.N)
        for i in range(self.N):
            pred_time = self.secs_elapsed + (i * self.dt)
            
            t_pred_jump = pred_time - self.jump_start_time
            t_curr_jump = self.secs_elapsed - self.jump_start_time
            
            tgt_y, tgt_dy = get_jump_y(t_pred_jump, t_curr_jump)
            X_ref[i*4 : i*4 + 4] = np.array([0.0, tgt_y, 0.0, tgt_dy])

        state_error = (self.A_bar @ x0) + self.G_bar - X_ref
        f = self.B_bar.T @ (self.Q_bar @ state_error)

        self.prob.update(q=f)
        results = self.prob.solve()

        if results.info.status != 'solved':
            self.get_logger().warn("OSQP failed to solve!")
            return

        U_optimal = results.x
        Fx_ff = U_optimal[0]
        Fy_ff = U_optimal[1]

        # --- FLIGHT PHASE FEED-FORWARD MASKING ---
        t_curr_jump = self.secs_elapsed - self.jump_start_time
        
        hp, hr, hs, g_val = 0.70, 0.50, 0.25, 9.81
        vr = math.sqrt(2 * g_val * (hp - hr))
        a_push = g_val * (hp - hr) / (hr - hs)
        tr = vr / a_push
        t_flight = 2 * vr / g_val
        t_land = tr + t_flight - 0.125
        
        if tr < t_curr_jump <= t_land:
            Fx_ff = 0.0
            Fy_ff = 0.0
        # -----------------------------------------

        X_optimal = (self.A_bar @ x0) + (self.B_bar @ U_optimal) + self.G_bar
        x_tgt  = X_optimal[0]
        y_tgt  = X_optimal[1]
        dx_tgt = X_optimal[2]
        dy_tgt = X_optimal[3]

        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(x_tgt), float(y_tgt), float(dx_tgt), float(dy_tgt), float(Fx_ff), float(Fy_ff)]
        self.cmd_pub.publish(cmd_msg)

        mode_str = "JUMPING" if (0 <= self.secs_elapsed - self.jump_start_time <= 1.0) else "IDLE"
        sys.stdout.write(f"\r[{mode_str}] FF -> Fx: {Fx_ff:+.1f} N | Fy: {Fy_ff:+.1f} N | Tgt Y: {y_tgt:+.3f} m      ")
        sys.stdout.flush()

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = LaikaMPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
