import numpy as np
import casadi as ca
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as patches
import time

# ==========================================
# 1. CONFIGURATION
# ==========================================
class RobotConfig:
    def __init__(self):
        # -- Physical Params --
        self.mb = 2.0; self.m1 = 0.5; self.m2 = 0.5   
        self.l1 = 0.5; self.l2 = 0.5; self.g  = 9.81  
        self.T_max  = 120.0 

        # -- Time & Horizon --
        self.dt_mpc = 0.02      # MPC Frequency (50Hz)
        self.dt_sim = 0.001     # Sim Frequency (1000Hz)
        self.N      = 30        # Horizon length

        # -- Targets (The "Knobs" for Behavior) --
        self.h_squat     = 0.35  # Height while waiting (m)
        self.h_extend    = 0.85  # Extension during push (m) - must be < l1+l2
        self.h_jump_goal = 1.10  # Virtual target to aim for during flight (m)
        self.h_retract   = 0.30  # How high to tuck the foot during flight (m)

        # -- Cost Weights (Tuning Priority) --
        self.w_foot_y    = 100000.0 # Pin foot to ground (Vertical)
        self.w_foot_x    = 5000.0   # Prevent foot sliding (Horizontal)
        self.w_retract   = 1000.0   # Pull leg up during flight
        self.w_height    = 1000.0   # Track the height targets
        
        # -- Regularization (Smoothness) --
        self.w_torque    = 0.01     # Minimize effort
        self.w_vel       = 0.1      # Damping (minimize joint velocity)
        self.w_force     = 0.001    # Force regularization

# ==========================================
# 2. PHYSICS FACTORY
# ==========================================
def get_acceleration_function(cfg):
    """ Returns a CasADi function for Continuous Acceleration: acc = f(x, u, f_ext) """
    q   = ca.MX.sym('q', 3)   # [y, q1, q2]
    dq  = ca.MX.sym('dq', 3)
    u   = ca.MX.sym('u', 2)
    f_y = ca.MX.sym('f_y', 1) # External vertical force
    
    # Kinematics
    p_base = ca.vertcat(0, q[0])
    p_c1   = p_base + 0.5 * cfg.l1 * ca.vertcat(ca.sin(q[1]), -ca.cos(q[1]))
    p_c2   = p_base + cfg.l1 * ca.vertcat(ca.sin(q[1]), -ca.cos(q[1])) + \
             0.5 * cfg.l2 * ca.vertcat(ca.sin(q[1]+q[2]), -ca.cos(q[1]+q[2]))
    p_foot = p_base + cfg.l1 * ca.vertcat(ca.sin(q[1]), -ca.cos(q[1])) + \
             cfg.l2 * ca.vertcat(ca.sin(q[1]+q[2]), -ca.cos(q[1]+q[2]))

    # Velocities
    v_base = ca.jtimes(p_base, q, dq)
    v_c1   = ca.jtimes(p_c1, q, dq)
    v_c2   = ca.jtimes(p_c2, q, dq)
    
    # Energy (Lagrangian)
    I1 = (1/12)*cfg.m1*cfg.l1**2
    I2 = (1/12)*cfg.m2*cfg.l2**2
    w1 = dq[1]
    w2 = dq[1] + dq[2]
    
    T_energy = 0.5*cfg.mb*ca.sumsqr(v_base) + 0.5*cfg.m1*ca.sumsqr(v_c1) + \
               0.5*cfg.m2*ca.sumsqr(v_c2)   + 0.5*I1*w1**2 + 0.5*I2*w2**2
    V_energy = cfg.g * (cfg.mb*p_base[1] + cfg.m1*p_c1[1] + cfg.m2*p_c2[1])
    
    # Dynamics: M*acc + C = Tau
    M = ca.hessian(T_energy, dq)[0]
    
    L = T_energy - V_energy
    dL_dq = ca.gradient(L, q)
    dL_ddq = ca.gradient(L, dq)
    bias = ca.jtimes(dL_ddq, q, dq) - dL_dq
    
    # Forces (Torque + Ground Force + Damping)
    J_foot = ca.jacobian(p_foot[1], q).T
    damping = 0.1 * dq # Joint damping for stability
    Tau = ca.vertcat(0, u[0], u[1]) + J_foot * f_y - ca.vertcat(0, damping[1], damping[2])
    
    acc = ca.solve(M, Tau - bias)
    
    # Return function: f(state, u, f_ext) -> acc
    return ca.Function('calc_acc', [ca.vertcat(q, dq), u, f_y], [acc])

# ==========================================
# 3. MPC CONTROLLER
# ==========================================
class JumpingMPC:
    def __init__(self, config):
        self.cfg = config
        self.opti = ca.Opti()
        
        self.X = self.opti.variable(6, self.cfg.N + 1)
        self.U = self.opti.variable(2, self.cfg.N)     
        self.F_grf = self.opti.variable(1, self.cfg.N) 

        self.P_init = self.opti.parameter(6)      
        self.P_sched = self.opti.parameter(self.cfg.N) 

        self.last_X = np.zeros((6, self.cfg.N + 1))
        self.last_U = np.zeros((2, self.cfg.N))
        self.last_F = np.zeros((1, self.cfg.N))

        self.f_acc = get_acceleration_function(config)
        
        self._setup_constraints()
        self._setup_cost()
        
        # Solver Settings
        p_opts = {
            'expand': False, 
            'print_time': False,
            'jit': True, 
            'jit_options': {'flags': '-O3'} # Maximize C++ optimization
        }   
        s_opts = {'max_iter': 1000, 'print_level': 0, 'sb': 'yes', 'tol': 1e-1}
        self.opti.solver('ipopt', p_opts, s_opts)

    def _get_discrete_step(self, x, u, f):
        # Semi-Implicit Euler
        dq = x[3:]
        acc = self.f_acc(x, u, f)
        dq_next = dq + acc * self.cfg.dt_mpc
        q_next  = x[:3] + dq_next * self.cfg.dt_mpc
        return ca.vertcat(q_next, dq_next)

    def _setup_constraints(self):
        self.opti.subject_to(self.X[:, 0] == self.P_init)
        
        for k in range(self.cfg.N):
            x_next = self._get_discrete_step(self.X[:, k], self.U[:, k], self.F_grf[k])
            self.opti.subject_to(self.X[:, k+1] == x_next)
            
            mode = self.P_sched[k]
            is_flight = (mode - 1)*(mode - 2)*0.5 
            
            self.opti.subject_to(self.F_grf[k] >= -0.1) 
            self.opti.subject_to(is_flight * self.F_grf[k] == 0)

        self.opti.subject_to(self.opti.bounded(-self.cfg.T_max, self.U, self.cfg.T_max))
        self.opti.subject_to(self.X[2, :] > 0.2) 
        self.opti.subject_to(self.X[2, :] < 2.5) 

    def _setup_cost(self):
        cost = 0
        cfg = self.cfg # Short alias
        
        for k in range(cfg.N):
            st = self.X[:, k]
            
            # --- Kinematics ---
            foot_x = cfg.l1*ca.sin(st[1]) + cfg.l2*ca.sin(st[1]+st[2])
            foot_y = st[0] - cfg.l1*ca.cos(st[1]) - cfg.l2*ca.cos(st[1]+st[2])
            
            # --- Mode Logic ---
            mode = self.P_sched[k]
            is_flight = (mode - 1)*(mode - 2)*0.5
            is_ground = 1 - is_flight
            
            # Differentiable switches
            is_wait = ca.exp(-100*(mode-1)**2)
            is_push = ca.exp(-100*(mode-2)**2)
            is_fly  = ca.exp(-100*(mode-0)**2)
            
            # --- Targets ---
            target_h = cfg.h_squat * is_wait + \
                       cfg.h_extend * is_push + \
                       cfg.h_jump_goal * is_fly
            
            # --- Cost Terms ---
            # 1. Height Tracking
            cost += cfg.w_height * (st[0] - target_h)**2
            
            # 2. Foot Constraints (Ground)
            cost += cfg.w_foot_y * is_ground * (foot_y**2)      
            cost += cfg.w_foot_x * is_ground * (foot_x**2) 
            
            # 3. Foot Constraints (Flight)
            cost += cfg.w_retract * is_fly * (foot_y - cfg.h_retract)**2     
            
            # 4. Regularization
            cost += cfg.w_torque * ca.sumsqr(self.U[:, k])          
            cost += cfg.w_vel    * ca.sumsqr(st[3:])                 
            cost += cfg.w_force  * ca.sumsqr(self.F_grf[k])        
            
        self.opti.minimize(cost)

    def solve(self, state, schedule):
        self.opti.set_value(self.P_init, state)
        self.opti.set_value(self.P_sched, schedule)
        guess_X = np.roll(self.last_X, -1, axis=1)
        guess_X[:, -1] = guess_X[:, -2] # Duplicate last state
        
        self.opti.set_initial(self.X, guess_X)
        self.opti.set_initial(self.F_grf, self.last_F)
        try:
            sol = self.opti.solve()
            self.last_X, self.last_U, self.last_F = sol.value(self.X), sol.value(self.U), sol.value(self.F_grf)
            return self.last_U[:, 0], self.last_X, self.last_F, True 
        except:
            try:
                self.last_X = self.opti.debug.value(self.X)
                self.last_U = self.opti.debug.value(self.U)
                self.last_F = self.opti.debug.value(self.F_grf)
                return self.last_U[:, 0], self.last_X, self.last_F, False
            except:
                return np.zeros(2), None, None, False

# ==========================================
# 4. FAST SIMULATOR (Compiled)
# ==========================================
class Simulator:
    def __init__(self, config):
        self.cfg = config
        
        # 1. Define Symbols
        x_sym = ca.MX.sym('x', 6)
        u_sym = ca.MX.sym('u', 2)
        
        # 2. Get the Physics Function
        acc_func = get_acceleration_function(config)
        
        # 3. Build the Loop Graph
        x_next = x_sym
        n_steps = int(self.cfg.dt_mpc / self.cfg.dt_sim)
        
        for _ in range(n_steps):
            y, q1, q2 = x_next[0], x_next[1], x_next[2]
            dy, dq1, dq2 = x_next[3], x_next[4], x_next[5]
            
            # Kinematics
            foot_y = y - self.cfg.l1*ca.cos(q1) - self.cfg.l2*ca.cos(q1+q2)
            J_hip  = self.cfg.l1*ca.sin(q1) + self.cfg.l2*ca.sin(q1+q2)
            J_knee = self.cfg.l2*ca.sin(q1+q2)
            foot_dy = dy + J_hip*dq1 + J_knee*dq2
            
            # Smooth Ground Force (Graph compatible)
            f_spring = -5000 * foot_y - 50 * foot_dy
            f_ext = ca.if_else(foot_y < 0, ca.fmax(0, f_spring), 0)
            
            # Get Accel (Inline the physics function)
            acc = acc_func(x_next, u_sym, f_ext)
            
            # Integration
            x_next = ca.vertcat(
                x_next[:3] + (x_next[3:] + acc * self.cfg.dt_sim) * self.cfg.dt_sim, # q_next
                x_next[3:] + acc * self.cfg.dt_sim                                   # dq_next
            )
            
            # Floor safety
            x_next[0] = ca.fmax(0.05, x_next[0])
            
        # 4. Compile the entire loop
        # 'jit': True forces this to be compiled to C++ on startup
        self.step_func = ca.Function('sim_step', [x_sym, u_sym], [x_next], 
                                     {'jit': True, 'jit_options': {'flags': '-O3'}})

    def step(self, state, u_mpc):
        # Python now only calls one C++ function instead of looping 20 times
        return self.step_func(state, u_mpc).full().flatten()

# ==========================================
# 5. MAIN
# ==========================================
if __name__ == "__main__":
    cfg = RobotConfig()
    mpc = JumpingMPC(cfg)
    sim = Simulator(cfg)
    
    # Schedule: Wait(1) -> Push(2) -> Fly(0) -> Wait(1)
    schedule = np.array([1]*60 + [2]*10 + [0]*15 + [1]*50)
    sim_state = np.array([0.38, 0.0, 2.4, 0.0, 0.0, 0.0])

    fig, ax = plt.subplots(figsize=(6, 8))
    ax.set_xlim(-0.8, 0.8); ax.set_ylim(-0.1, 1.5)
    ax.set_aspect('equal'); ax.grid(True)
    
    def get_coords(st):
        y, q1, q2 = st[0], st[1], st[2]
        hip = np.array([0, y])
        knee = hip + np.array([cfg.l1*np.sin(q1), -cfg.l1*np.cos(q1)])
        foot = knee + np.array([cfg.l2*np.sin(q1+q2), -cfg.l2*np.cos(q1+q2)])
        return hip, knee, foot

    ground = ax.hlines(0, -1, 1, color='k', lw=2)
    rect = patches.Rectangle((-0.15, 0), 0.3, 0.2, color='#4466aa', zorder=10)
    ax.add_patch(rect)
    leg_line, = ax.plot([], [], 'o-', lw=5, color='black', zorder=10)
    ghosts = [ax.plot([], [], 'o-', lw=2, color='#33cc33', alpha=0.3+i*0.2)[0] for i in range(3)]
    txt = ax.text(0.05, 0.85, '', transform=ax.transAxes)

    frame_idx = 0
    last_real_time = time.time()

    def update(frame):
        global frame_idx, sim_state, last_real_time
        
        current_real_time = time.time()
        dt_real = current_real_time - last_real_time
        last_real_time = current_real_time
        rtf = cfg.dt_mpc / (dt_real + 1e-9) 
        sim_time = frame_idx * cfg.dt_mpc

        if frame_idx + cfg.N < len(schedule):
            sched_win = schedule[frame_idx : frame_idx + cfg.N]
        else: sched_win = np.ones(cfg.N)
        
        u, x_pred, f_pred, success = mpc.solve(sim_state, sched_win)
        sim_state = sim.step(sim_state, u)
        
        frame_idx += 1
        
        h, k, f = get_coords(sim_state)
        rect.set_y(h[1])
        leg_line.set_data([h[0], k[0], f[0]], [h[1], k[1], f[1]])
        
        if x_pred is not None:
            for i, idx in enumerate([5, 10, 15]):
                if idx < x_pred.shape[1]:
                    gh, gk, gf = get_coords(x_pred[:, idx])
                    ghosts[i].set_data([gh[0], gk[0], gf[0]], [gh[1], gk[1], gf[1]])
                    ghosts[i].set_visible(True)
        
        status_map = {0: "FLY", 1: "WAIT", 2: "PUSH"}
        mode_str = status_map.get(int(sched_win[0]), "UNK")
        if not success: mode_str = "FAIL"
        
        info_str = (f"Mode:   {mode_str}\n"
                    f"Torque: {u[0]:.1f}, {u[1]:.1f} Nm\n"
                    f"Time:   {sim_time:.2f} s\n"
                    f"Speed:  {rtf:.2f} x")
        
        txt.set_text(info_str)
        ax.set_facecolor('#ffb3b3' if not success else 'white')
        
        if frame_idx >= len(schedule) - cfg.N:
            frame_idx = 0 
            
        return [rect, leg_line, txt] + ghosts

    ani = animation.FuncAnimation(fig, update, frames=len(schedule)-cfg.N, interval=30, blit=False)
    plt.show()