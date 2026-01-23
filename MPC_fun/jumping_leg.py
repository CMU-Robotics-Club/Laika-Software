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
        # -- Physical Params (From URDF) --
        self.mb = 1.696  
        self.m1 = 0.979  
        self.m2 = 0.262  
        
        self.l1 = 0.35   # Hip->Knee
        self.l2 = 0.40   # Knee->Foot
        self.g  = 9.81  
        self.T_max  = 120.0 

        # -- Time & Horizon --
        self.dt_mpc = 0.01      
        self.dt_sim = 0.001     
        self.N      = 30        

        # -- Targets (Scaled for 0.75m Leg) --
        self.h_squat     = 0.25  
        self.h_extend    = 0.65  
        self.h_jump_goal = 0.95  
        self.h_retract   = 0.30  

        # -- Cost Weights --
        # SOFTENED: Reduced from 100,000 to prevent crash on landing
        self.w_foot_y    = 10000.0 
        self.w_foot_x    = 5000.0   
        self.w_retract   = 500.0   
        self.w_height    = 2000.0   
        
        self.w_torque    = 0.01     
        self.w_vel       = 0.1      
        self.w_force     = 0.001    

# ==========================================
# 2. PHYSICS 
# ==========================================
def get_acceleration_function(cfg):
    """ Returns a CasADi function for Continuous Acceleration: acc = f(x, u, f_ext) """
    q   = ca.MX.sym('q', 3)   # [y, q1, q2]
    dq  = ca.MX.sym('dq', 3)
    u   = ca.MX.sym('u', 2)
    f_y = ca.MX.sym('f_y', 1) 
    
    p_base = ca.vertcat(0, q[0])
    p_c1   = p_base + 0.5 * cfg.l1 * ca.vertcat(ca.sin(q[1]), -ca.cos(q[1]))
    p_c2   = p_base + cfg.l1 * ca.vertcat(ca.sin(q[1]), -ca.cos(q[1])) + \
             0.5 * cfg.l2 * ca.vertcat(ca.sin(q[1]+q[2]), -ca.cos(q[1]+q[2]))
    p_foot = p_base + cfg.l1 * ca.vertcat(ca.sin(q[1]), -ca.cos(q[1])) + \
             cfg.l2 * ca.vertcat(ca.sin(q[1]+q[2]), -ca.cos(q[1]+q[2]))

    v_base = ca.jtimes(p_base, q, dq)
    v_c1   = ca.jtimes(p_c1, q, dq)
    v_c2   = ca.jtimes(p_c2, q, dq)
    
    I1 = (1/12)*cfg.m1*cfg.l1**2
    I2 = (1/12)*cfg.m2*cfg.l2**2
    w1 = dq[1]
    w2 = dq[1] + dq[2]
    
    T_energy = 0.5*cfg.mb*ca.sumsqr(v_base) + 0.5*cfg.m1*ca.sumsqr(v_c1) + \
               0.5*cfg.m2*ca.sumsqr(v_c2)   + 0.5*I1*w1**2 + 0.5*I2*w2**2
    V_energy = cfg.g * (cfg.mb*p_base[1] + cfg.m1*p_c1[1] + cfg.m2*p_c2[1])
    
    M = ca.hessian(T_energy, dq)[0]
    L = T_energy - V_energy
    dL_dq = ca.gradient(L, q)
    dL_ddq = ca.gradient(L, dq)
    bias = ca.jtimes(dL_ddq, q, dq) - dL_dq
    
    J_foot = ca.jacobian(p_foot[1], q).T
    damping = 0.1 * dq 
    Tau = ca.vertcat(0, u[0], u[1]) + J_foot * f_y - ca.vertcat(0, damping[1], damping[2])
    
    acc = ca.solve(M, Tau - bias)
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
        
        p_opts = {'expand': False, 'print_time': False, 'jit': True, 'jit_options': {'flags': '-O3'}}   
        s_opts = {'max_iter': 1000, 'print_level': 0, 'sb': 'yes', 'tol': 1e-1}
        self.opti.solver('ipopt', p_opts, s_opts)

    def _get_discrete_step(self, x, u, f):
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
            
            # SOFTENED: Removed hard constraint "is_flight * F_grf == 0"
            # It is now handled by a cost term to allow smooth transitions
            self.opti.subject_to(self.F_grf[k] >= -0.1) 

        self.opti.subject_to(self.opti.bounded(-self.cfg.T_max, self.U, self.cfg.T_max))
        self.opti.subject_to(self.X[2, :] > 0.2) 
        self.opti.subject_to(self.X[2, :] < 2.5) 

    def _setup_cost(self):
        cost = 0
        cfg = self.cfg
        for k in range(cfg.N):
            st = self.X[:, k]
            foot_x = cfg.l1*ca.sin(st[1]) + cfg.l2*ca.sin(st[1]+st[2])
            foot_y = st[0] - cfg.l1*ca.cos(st[1]) - cfg.l2*ca.cos(st[1]+st[2])
            
            mode = self.P_sched[k]
            is_flight = (mode - 1)*(mode - 2)*0.5
            is_ground = 1 - is_flight
            
            is_wait = ca.exp(-100*(mode-1)**2)
            is_push = ca.exp(-100*(mode-2)**2)
            is_fly  = ca.exp(-100*(mode-0)**2)
            
            target_h = cfg.h_squat * is_wait + cfg.h_extend * is_push + cfg.h_jump_goal * is_fly
            
            # --- Costs ---
            cost += cfg.w_height * (st[0] - target_h)**2
            cost += cfg.w_foot_y * is_ground * (foot_y**2)      
            cost += cfg.w_foot_x * (foot_x**2) 
            cost += cfg.w_retract * is_fly * (foot_y - cfg.h_retract)**2     
            
            # SOFTENED: Flight Force Penalty (Instead of hard constraint)
            # Allows tiny forces during transition Push->Fly to prevent crash
            cost += 10000.0 * is_flight * (self.F_grf[k]**2)
            
            cost += cfg.w_torque * ca.sumsqr(self.U[:, k])          
            cost += cfg.w_vel    * ca.sumsqr(st[3:])                 
            cost += cfg.w_force  * ca.sumsqr(self.F_grf[k])        
        self.opti.minimize(cost)

    def solve(self, state, schedule):
        self.opti.set_value(self.P_init, state)
        self.opti.set_value(self.P_sched, schedule)
        guess_X = np.roll(self.last_X, -1, axis=1)
        guess_X[:, -1] = guess_X[:, -2]
        
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
        x_sym = ca.MX.sym('x', 6)
        u_sym = ca.MX.sym('u', 2)
        acc_func = get_acceleration_function(config)
        
        x_next = x_sym
        n_steps = int(self.cfg.dt_mpc / self.cfg.dt_sim)
        
        for _ in range(n_steps):
            y, q1, q2 = x_next[0], x_next[1], x_next[2]
            dy, dq1, dq2 = x_next[3], x_next[4], x_next[5]
            foot_y = y - self.cfg.l1*ca.cos(q1) - self.cfg.l2*ca.cos(q1+q2)
            J_hip  = self.cfg.l1*ca.sin(q1) + self.cfg.l2*ca.sin(q1+q2)
            J_knee = self.cfg.l2*ca.sin(q1+q2)
            foot_dy = dy + J_hip*dq1 + J_knee*dq2
            
            f_spring = -5000 * foot_y - 50 * foot_dy
            f_ext = ca.if_else(foot_y < 0, ca.fmax(0, f_spring), 0)
            
            acc = acc_func(x_next, u_sym, f_ext)
            x_next = ca.vertcat(
                x_next[:3] + (x_next[3:] + acc * self.cfg.dt_sim) * self.cfg.dt_sim,
                x_next[3:] + acc * self.cfg.dt_sim                                   
            )
            x_next[0] = ca.fmax(0.05, x_next[0])
            
        self.step_func = ca.Function('sim_step', [x_sym, u_sym], [x_next], 
                                     {'jit': True, 'jit_options': {'flags': '-O3'}})

    def step(self, state, u_mpc):
        return self.step_func(state, u_mpc).full().flatten()

# ==========================================
# 5. MAIN (With Annotated Plots & Auto-Scale)
# ==========================================
if __name__ == "__main__":
    cfg = RobotConfig()
    mpc = JumpingMPC(cfg)
    sim = Simulator(cfg)
    
    # Schedule: Wait(1) -> Push(2) -> Fly(0) -> Wait(1)
    schedule = np.array([1]*100 + [2]*15 + [0]*40 + [1]*100)
    sim_state = np.array([0.25, 0.0, 2.4, 0.0, 0.0, 0.0]) # Adjusted start height

    # Setup Figures
    fig, (ax_anim, ax_plot) = plt.subplots(1, 2, figsize=(12, 6), gridspec_kw={'width_ratios': [1, 1.5]})
    
    # 1. Animation Axis
    ax_anim.set_xlim(-0.8, 0.8)
    ax_anim.set_ylim(-0.1, 1.5)
    ax_anim.set_aspect('equal')
    ax_anim.grid(True)
    ax_anim.set_title("Jumping Leg Sim")

    def get_coords(st):
        y, q1, q2 = st[0], st[1], st[2]
        hip = np.array([0, y])
        knee = hip + np.array([cfg.l1*np.sin(q1), -cfg.l1*np.cos(q1)])
        foot = knee + np.array([cfg.l2*np.sin(q1+q2), -cfg.l2*np.cos(q1+q2)])
        return hip, knee, foot

    ground = ax_anim.hlines(0, -1, 1, color='k', lw=2)
    rect = patches.Rectangle((-0.15, 0), 0.3, 0.2, color='#4466aa', zorder=10)
    ax_anim.add_patch(rect)
    leg_line, = ax_anim.plot([], [], 'o-', lw=5, color='black', zorder=10)
    ghosts = [ax_anim.plot([], [], 'o-', lw=2, color='#33cc33', alpha=0.3+i*0.2)[0] for i in range(3)]
    txt = ax_anim.text(0.05, 0.85, '', transform=ax_anim.transAxes)

    # 2. Torque Plot Axis
    ax_plot.set_xlim(0, 3)
    ax_plot.set_ylim(-50, 50) # CHANGED: Default scale set to +/- 50 Nm
    ax_plot.grid(True)
    ax_plot.set_title("Joint Torques & Modes")
    ax_plot.set_xlabel("Time (s)")
    ax_plot.set_ylabel("Torque (Nm)")
    
    line_tau0, = ax_plot.plot([], [], label='Hip', color='red')
    line_tau1, = ax_plot.plot([], [], label='Knee', color='blue')
    ax_plot.legend(loc='lower left')
    
    t_data, tau0_data, tau1_data = [], [], []
    mode_annotations = [] 
    
    frame_idx = 0
    last_real_time = time.time()
    last_mode = -1 

    def update(frame):
        global frame_idx, sim_state, last_real_time, last_mode
        
        current_real_time = time.time()
        dt_real = current_real_time - last_real_time
        last_real_time = current_real_time
        rtf = cfg.dt_mpc / (dt_real + 1e-9) 
        sim_time = frame_idx * cfg.dt_mpc

        if frame_idx + cfg.N < len(schedule):
            sched_win = schedule[frame_idx : frame_idx + cfg.N]
        else: sched_win = np.ones(cfg.N)
        
        # --- Mode Annotation Logic ---
        current_mode = int(sched_win[0])
        if current_mode != last_mode and frame_idx > 0:
            status_map = {0: "FLY", 1: "WAIT", 2: "PUSH"}
            label = status_map.get(current_mode, "UNK")
            vline = ax_plot.axvline(x=sim_time, color='gray', linestyle='--', alpha=0.5)
            # Place label at current graph top
            y_lim_top = ax_plot.get_ylim()[1]
            t_lbl = ax_plot.text(sim_time, y_lim_top - 5, f" {label}", rotation=90, verticalalignment='top', color='#555555')
            mode_annotations.append(vline)
            mode_annotations.append(t_lbl)
            last_mode = current_mode
        elif frame_idx == 0:
            last_mode = current_mode 

        u, x_pred, f_pred, success = mpc.solve(sim_state, sched_win)
        sim_state = sim.step(sim_state, u)
        
        frame_idx += 1
        
        # Update Anim
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
        ax_anim.set_facecolor('#ffb3b3' if not success else 'white')
        
        # Update Plot Data
        t_data.append(sim_time)
        tau0_data.append(u[0])
        tau1_data.append(u[1])
        
        # --- Dynamic Resizing Logic ---
        curr_max = max(abs(u[0]), abs(u[1]))
        current_ylim = ax_plot.get_ylim()[1]
        if curr_max > current_ylim:
            new_lim = curr_max + 10 # Add margin
            ax_plot.set_ylim(-new_lim, new_lim)
        
        # Scroll Plot Window
        window = 2.5 
        if sim_time > window:
            ax_plot.set_xlim(sim_time - window, sim_time + 0.1)
        else:
            ax_plot.set_xlim(0, max(window, sim_time))
            
        line_tau0.set_data(t_data, tau0_data)
        line_tau1.set_data(t_data, tau1_data)
        
        if frame_idx >= len(schedule) - cfg.N:
            frame_idx = 0 
            t_data.clear(); tau0_data.clear(); tau1_data.clear()
            for artist in mode_annotations: artist.remove()
            mode_annotations.clear()
            last_mode = -1 
            ax_plot.set_ylim(-50, 50)
            
        return [rect, leg_line, txt, line_tau0, line_tau1] + ghosts + mode_annotations

    ani = animation.FuncAnimation(fig, update, frames=len(schedule)-cfg.N, interval=30, blit=False)
    plt.show()