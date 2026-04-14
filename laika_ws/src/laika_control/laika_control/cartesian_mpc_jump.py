import os
import sys
import select
import termios
import tty
import threading
import math
import csv
from datetime import datetime
from typing import Tuple, Callable, Optional, List, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import scipy.sparse as sparse
import osqp

# Use Agg backend for headless plotting
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# --- CONFIGURATION PARAMETERS ---
H_CROUCH = 0.25  # Height for initial crouch and idle
H_TUCK   = 0.35  # Target height to pull up to during flight
H_LAND   = 0.50  # Height where pushoff ends and landing begins
H_PEAK   = 0.70  # Peak height of the jump
G_VAL    = 9.81  # Gravity

# --- PRE-COMPUTED KINEMATICS ---
V_R = math.sqrt(2 * G_VAL * (H_PEAK - H_LAND))
A_PUSH = G_VAL * (H_PEAK - H_LAND) / (H_LAND - H_CROUCH)

T_R = V_R / A_PUSH
T_FLIGHT_TOTAL = 2 * V_R / G_VAL
T_LAND = T_R + T_FLIGHT_TOTAL - 0.0
T_END = T_LAND + T_R
# --------------------------------


class TerminalKeyLogger(threading.Thread):
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
    if 0 <= t_curr <= T_R:
        y = H_CROUCH + 0.5 * A_PUSH * (t_pred**2)
        dy = A_PUSH * t_pred
        if y > 0.72: 
            y = 0.72
        return -y, -dy

    if t_pred < 0:
        return -H_LAND, 0.0

    if t_pred <= T_LAND:
        T_f = max(T_LAND - T_R, 0.01)
        tau = max(0.0, min(t_pred - T_R, T_f))
        mid = T_f / 2.0
        
        if tau <= mid:
            s = tau / mid
            y0, v0 = -H_LAND, -V_R
            yf, vf = -H_TUCK, 0.0
            T_segment = mid
        else:
            s = (tau - mid) / mid
            y0, v0 = -H_TUCK, 0.0
            yf, vf = -H_LAND, V_R
            T_segment = mid

        c_v0, c_vf = v0 * T_segment, vf * T_segment
        a = 2 * (y0 - yf) + c_v0 + c_vf
        b = 3 * (yf - y0) - 2 * c_v0 - c_vf
        c = c_v0
        d = y0
        
        y = a * (s**3) + b * (s**2) + c * s + d
        dy = (3 * a * (s**2) + 2 * b * s + c) / T_segment
        return y, dy

    if t_pred <= T_END:
        dt = t_pred - T_LAND
        y = H_LAND - V_R * dt + 0.5 * A_PUSH * (dt**2)
        dy = -V_R + A_PUSH * dt
        return -y, -dy

    return -H_CROUCH, 0.0


class LaikaMPC(Node):
    def __init__(self) -> None:
        super().__init__('laika_mpc_node')
        
        self.cmd_pub = self.create_publisher(Float64MultiArray, 'laika_cartesian_impedance_controller/command', 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        
        self.l1 = 0.35
        self.l2 = 0.41
        self.knee_offset = 0.04573
        self.m = 5.0
        self.g = G_VAL    

        self.N = 25      
        self.mpc_dt = 0.01  
        
        # Timing Variables
        self.start_time_ns = self.get_clock().now().nanoseconds
        self.last_loop_time_ns = self.start_time_ns
        self.secs_elapsed = 0.0
        
        # State Machine Initialization
        self.state = 'LIMP' 
        self.stand_start_time = 0.0
        self.stand_start_y = 0.0
        self.T_STAND = 2.0 # Take 2 seconds to stand up
        
        self.jump_start_time = -999.0 
        self.jump_history: List[float] = [] 
        self.jump_duration_secs = 1.0
        
        # Threading & Decoupled State
        self.current_state = np.zeros(4)
        self.state_received = False
        self.state_lock = threading.Lock()
        self.new_state_event = threading.Event()
        
        self.log_buffer: List[Dict[str, float]] = []
        
        self.setup_dynamics()
        self.setup_condensed_mpc()
        
        self.prob = osqp.OSQP()
        self.prob.setup(
            P=self.H, q=self.f_dummy, A=self.A_ineq, l=self.l_ineq, u=self.u_ineq, 
            warm_start=True, verbose=False
        )
        
        self.key_logger = TerminalKeyLogger(self.handle_keypress)
        self.key_logger.start()

        self.mpc_thread = threading.Thread(target=self.continuous_mpc_loop, daemon=True)
        self.mpc_thread.start()
        
        self.get_logger().info(f"LTV-MPC Node Started in LIMP mode.")
        self.get_logger().info("Press 'u' to gracefully STAND UP.")
        self.get_logger().info("Press 'SPACE' to trigger the JUMP sequence (once idle)!\n")

    def handle_keypress(self, key: str) -> None:
        if key == 'u' and self.state == 'LIMP':
            self.state = 'STAND_UP'
            self.stand_start_time = self.secs_elapsed
            with self.state_lock:
                self.stand_start_y = self.current_state[1]
                
        elif key == ' ' and self.state == 'IDLE':
            self.state = 'JUMP'
            self.jump_start_time = self.secs_elapsed
            self.jump_history.append(self.secs_elapsed)

    def setup_dynamics(self) -> None:
        self.Ad = np.array([
            [1.0, 0.0, self.mpc_dt, 0.0],
            [0.0, 1.0, 0.0, self.mpc_dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        
        self.Bd = np.array([
            [(self.mpc_dt**2) / (2 * self.m), 0.0],
            [0.0, (self.mpc_dt**2) / (2 * self.m)],
            [self.mpc_dt / self.m, 0.0],
            [0.0, self.mpc_dt / self.m]
        ])
        
        # Delta-U formulation: zero internal gravity model
        self.gd = np.array([0.0, 0.0, 0.0, 0.0])

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

        Q_step = np.diag([10000.0, 10000.0, .020, .020])
        R_step = np.diag([0.01, 0.01])
        
        self.Q_bar = sparse.kron(sparse.eye(self.N), Q_step)
        self.R_bar = sparse.kron(sparse.eye(self.N), R_step)
        
        B_bar_sparse = sparse.csc_matrix(self.B_bar)
        self.H = (B_bar_sparse.T @ self.Q_bar @ B_bar_sparse + self.R_bar).tocsc()
        
        self.f_dummy = np.zeros(n_u * self.N)
        self.A_ineq = sparse.eye(n_u * self.N).tocsc()
        
        self.l_ineq = -40.0 * np.ones(n_u * self.N)
        self.u_ineq = 40.0 * np.ones(n_u * self.N)
        
        # Unilateral Contact Constraint shifted by mg
        for i in range(1, n_u * self.N, 2):
            self.u_ineq[i] = self.m * self.g

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

        with self.state_lock:
            self.current_state = np.array([x, y, dx, dy])
            self.state_received = True
            
        self.new_state_event.set()
    
    def _dispatch_command(self, x_tgt: float, y_tgt: float, dx_tgt: float, dy_tgt: float, 
                          Fx_ff: float, Fy_ff: float, current_hz: float, x0: np.ndarray, log: bool = True) -> None:
        """Publishes control targets to the lower-level PD controller and optionally logs telemetry."""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(x_tgt), float(y_tgt), float(dx_tgt), float(dy_tgt), float(Fx_ff), float(Fy_ff)]
        self.cmd_pub.publish(cmd_msg)

        if not log:
            sys.stdout.write(f"\r[{self.state[:7]:<7}] {current_hz:5.0f} Hz | Fx: {Fx_ff:+.1f} N | Fy: {Fy_ff:+.1f} N | Tgt Y: {y_tgt:+.3f} m      ")
            sys.stdout.flush()
            return

        # Calculate current ideal state just for accurate telemetry plotting
        if self.state == 'STAND_UP':
            t_stand = self.secs_elapsed - self.stand_start_time
            s = min(max(t_stand / self.T_STAND, 0.0), 1.0)
            y_ideal = self.stand_start_y + (-H_CROUCH - self.stand_start_y) * (3*s**2 - 2*s**3)
            dy_ideal = (-H_CROUCH - self.stand_start_y) * (6*s - 6*s**2) / self.T_STAND if s < 1.0 else 0.0
        elif self.state == 'JUMP':
            y_ideal, dy_ideal = get_jump_y(self.secs_elapsed - self.jump_start_time, self.secs_elapsed - self.jump_start_time)
        else: # IDLE
            y_ideal, dy_ideal = -H_CROUCH, 0.0

        self.log_buffer.append({
            'time': float(self.secs_elapsed),
            'loop_hz': float(current_hz),
            'x_true': float(x0[0]), 'y_true': float(x0[1]),
            'dx_true': float(x0[2]), 'dy_true': float(x0[3]),
            'x_cmd': float(x_tgt), 'y_cmd': float(y_tgt),
            'dx_cmd': float(dx_tgt), 'dy_cmd': float(dy_tgt),
            'y_ideal': float(y_ideal), 'dy_ideal': float(dy_ideal),    
            'fx_cmd': float(Fx_ff), 'fy_cmd': float(Fy_ff)
        })

        mode_str = f"{self.state[:7]:<7}"
        sys.stdout.write(f"\r[{mode_str}] {current_hz:5.0f} Hz | Fx: {Fx_ff:+.1f} N | Fy: {Fy_ff:+.1f} N | Tgt Y: {y_tgt:+.3f} m      ")
        sys.stdout.flush()
    
    def continuous_mpc_loop(self) -> None:
        while rclpy.ok():
            if not self.new_state_event.wait(timeout=0.1):
                continue
                
            self.new_state_event.clear()

            now_ns = self.get_clock().now().nanoseconds
            actual_dt = (now_ns - self.last_loop_time_ns) / 1e9
            if actual_dt <= 0: actual_dt = 1e-6 
            current_hz = 1.0 / actual_dt
            self.last_loop_time_ns = now_ns
            self.secs_elapsed = (now_ns - self.start_time_ns) / 1e9

            with self.state_lock:
                x0 = self.current_state.copy()

            # --- LIMP MODE ---
            # Bypass MPC entirely. Send current position as target to PD with 0.0 N feedforward
            if self.state == 'LIMP':
                self._dispatch_command(x0[0], x0[1], 0.0, 0.0, 0.0, 0.0, current_hz, x0, log=False)
                continue
            
            # --- STATE TRANSITION CHECKS ---
            if self.state == 'STAND_UP' and (self.secs_elapsed - self.stand_start_time) >= self.T_STAND:
                self.state = 'IDLE'
            if self.state == 'JUMP' and (self.secs_elapsed - self.jump_start_time) > self.jump_duration_secs:
                self.state = 'IDLE'

            # --- FLIGHT PHASE BYPASS ---
            if self.state == 'JUMP':
                t_curr_jump = self.secs_elapsed - self.jump_start_time
                if T_R < t_curr_jump <= T_LAND:
                    y_tgt, dy_tgt = get_jump_y(t_curr_jump, t_curr_jump)
                    self._dispatch_command(0.0, y_tgt, 0.0, dy_tgt, 0.0, 0.0, current_hz, x0, log=True)
                    continue

            # --- POPULATE MPC REFERENCE HORIZON ---
            X_ref = np.zeros(4 * self.N)
            for i in range(self.N):
                pred_time = self.secs_elapsed + (i * self.mpc_dt)

                if self.state == 'STAND_UP':
                    t_stand = pred_time - self.stand_start_time
                    if t_stand >= self.T_STAND:
                        tgt_y, tgt_dy = -H_CROUCH, 0.0
                    elif t_stand <= 0.0:
                        tgt_y, tgt_dy = self.stand_start_y, 0.0
                    else:
                        s = t_stand / self.T_STAND
                        tgt_y = self.stand_start_y + (-H_CROUCH - self.stand_start_y) * (3*s**2 - 2*s**3)
                        tgt_dy = (-H_CROUCH - self.stand_start_y) * (6*s - 6*s**2) / self.T_STAND
                elif self.state == 'IDLE':
                    tgt_y, tgt_dy = -H_CROUCH, 0.0
                elif self.state == 'JUMP':
                    t_pred_jump = pred_time - self.jump_start_time
                    t_curr_jump = self.secs_elapsed - self.jump_start_time
                    tgt_y, tgt_dy = get_jump_y(t_pred_jump, t_curr_jump)

                X_ref[i*4 : i*4 + 4] = np.array([0.0, tgt_y, 0.0, tgt_dy])

            # --- SOLVE MPC ---
            state_error = (self.A_bar @ x0) + self.G_bar - X_ref
            f = self.B_bar.T @ (self.Q_bar @ state_error)

            self.prob.update(q=f)
            results = self.prob.solve()

            if results.info.status != 'solved':
                self.get_logger().warn("OSQP failed to solve!")
                continue

            U_optimal = results.x
            
            # Combine MPC delta with static gravity requirement
            Fx_ff = U_optimal[0]
            Fy_ff = U_optimal[1] - (self.m * self.g) * .25

            # Old masking behavior commented out for debugging reference
            # if self.state == 'JUMP':
            #     t_curr_jump = self.secs_elapsed - self.jump_start_time
            #     if T_R < t_curr_jump <= T_LAND:
            #         Fx_ff = 0.0
            #         Fy_ff = 0.0

            # X_optimal predicts where the MPC expects the leg to be this cycle
            X_optimal = (self.A_bar @ x0) + (self.B_bar @ U_optimal) + self.G_bar
            x_tgt, y_tgt, dx_tgt, dy_tgt = X_optimal[0], X_optimal[1], X_optimal[2], X_optimal[3]

            self._dispatch_command(x_tgt, y_tgt, dx_tgt, dy_tgt, Fx_ff, Fy_ff, current_hz, x0, log=True)

    def export_and_plot_telemetry(self) -> None:
        if not self.log_buffer:
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_base_dir = os.path.expanduser('~/laika_data/jump_logs')
        run_dir = os.path.join(log_base_dir, f"session_{timestamp}")
        
        try:
            os.makedirs(run_dir, exist_ok=True)
        except OSError as e:
            self.get_logger().error(f"\nCould not create log directory: {e}")
            return

        csv_path = os.path.join(run_dir, 'telemetry.csv')
        try:
            with open(csv_path, mode='w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=self.log_buffer[0].keys())
                writer.writeheader()
                writer.writerows(self.log_buffer)
            self.get_logger().info(f"\nSaved full session telemetry to {csv_path}")
        except IOError as e:
            self.get_logger().error(f"\nFailed to save CSV: {e}")
            return

        self._generate_plots(run_dir)

    def _generate_plots(self, save_dir: str) -> None:
        timings = (T_R, T_LAND, T_END)
        t = [row['time'] for row in self.log_buffer]
        hz = [row['loop_hz'] for row in self.log_buffer]
        
        x_t = [row['x_true'] for row in self.log_buffer]
        x_c = [row['x_cmd'] for row in self.log_buffer]
        dx_t = [row['dx_true'] for row in self.log_buffer]
        dx_c = [row['dx_cmd'] for row in self.log_buffer]
        fx_c = [row['fx_cmd'] for row in self.log_buffer]

        y_t = [row['y_true'] for row in self.log_buffer]
        y_c = [row['y_cmd'] for row in self.log_buffer]
        dy_t = [row['dy_true'] for row in self.log_buffer]
        dy_c = [row['dy_cmd'] for row in self.log_buffer]
        y_ideal = [row['y_ideal'] for row in self.log_buffer]
        dy_ideal = [row['dy_ideal'] for row in self.log_buffer]
        fy_c = [row['fy_cmd'] for row in self.log_buffer]

        zero_ideal = [0.0] * len(t)
        self._save_axis_figure(t, x_c, x_t, zero_ideal, dx_c, dx_t, zero_ideal, fx_c, 'X', os.path.join(save_dir, 'x_dynamics.png'), timings)
        self._save_axis_figure(t, y_c, y_t, y_ideal, dy_c, dy_t, dy_ideal, fy_c, 'Y', os.path.join(save_dir, 'y_dynamics.png'), timings)
        self._save_hz_figure(t, hz, os.path.join(save_dir, 'loop_rate.png'), timings)

    def _save_axis_figure(self, t: List[float], pos_c: List[float], pos_t: List[float], pos_i: List[float],
                          vel_c: List[float], vel_t: List[float], vel_i: List[float], force_c: List[float], 
                          axis_name: str, save_path: str, timings: Tuple[float, float, float]) -> None:
        
        tr, t_land, t_end = timings
        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig.suptitle(f'Cartesian MPC Telemetry (Full Session) - {axis_name} Axis', fontsize=14)

        if axis_name == 'Y':
            axs[0].plot(t, pos_i, 'k:', label=f'Ideal {axis_name} (Cost)', linewidth=2, alpha=0.6)
        axs[0].plot(t, pos_c, 'r--', label=f'Cmd {axis_name}', linewidth=2)
        axs[0].plot(t, pos_t, 'b-', label=f'True {axis_name}', linewidth=1.5)
        axs[0].set_ylabel('Position (m)')
        axs[0].grid(True, linestyle=':', alpha=0.7)

        if axis_name == 'Y':
            axs[1].plot(t, vel_i, 'k:', label=f'Ideal d{axis_name} (Cost)', linewidth=2, alpha=0.6)
        axs[1].plot(t, vel_c, 'r--', label=f'Cmd d{axis_name}', linewidth=2)
        axs[1].plot(t, vel_t, 'b-', label=f'True d{axis_name}', linewidth=1.5)
        axs[1].set_ylabel('Velocity (m/s)')
        axs[1].grid(True, linestyle=':', alpha=0.7)

        axs[2].plot(t, force_c, 'g-', label=f'Cmd F{axis_name.lower()} (FF)', linewidth=1.5)
        axs[2].set_xlabel('Total Elapsed Time (s)')
        axs[2].set_ylabel('Force (N)')
        axs[2].grid(True, linestyle=':', alpha=0.7)

        for i, ax in enumerate(axs):
            for j, jst in enumerate(self.jump_history):
                lbl_push = 'Pushoff' if i == 0 and j == 0 else ""
                lbl_flt  = 'Flight (FF=0)' if i == 0 and j == 0 else ""
                lbl_land = 'Landing' if i == 0 and j == 0 else ""

                ax.axvspan(jst, jst + tr, color='orange', alpha=0.10, label=lbl_push)
                ax.axvspan(jst + tr, jst + t_land, color='cyan', alpha=0.10, label=lbl_flt)
                ax.axvspan(jst + t_land, jst + t_end, color='green', alpha=0.10, label=lbl_land)

        axs[0].legend(loc='upper right')
        axs[1].legend(loc='upper right')
        axs[2].legend(loc='upper right')

        plt.tight_layout()
        fig.savefig(save_path, dpi=150)
        plt.close(fig)

    def _save_hz_figure(self, t: List[float], hz: List[float], save_path: str, timings: Tuple[float, float, float]) -> None:
        _, _, t_end = timings
        fig, ax = plt.subplots(figsize=(10, 4))
        fig.suptitle('MPC Execution Rate (Hz)', fontsize=14)
        
        ax.plot(t, hz, 'b-', linewidth=1.0, alpha=0.8)
        ax.set_xlabel('Total Elapsed Time (s)')
        ax.set_ylabel('Loop Rate (Hz)')
        ax.grid(True, linestyle=':', alpha=0.7)
        
        for j, jst in enumerate(self.jump_history):
            lbl_jump = 'Jump Execution' if j == 0 else ""
            ax.axvspan(jst, jst + t_end, color='purple', alpha=0.10, label=lbl_jump)
            
        if self.jump_history:
            ax.legend(loc='upper right')

        plt.tight_layout()
        fig.savefig(save_path, dpi=150)
        plt.close(fig)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = LaikaMPC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("\nShutting down, generating full session telemetry graphs...")
        node.export_and_plot_telemetry()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
