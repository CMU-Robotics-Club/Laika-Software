import numpy as np
import jumping_leg as leg

# 1. Setup
cfg = leg.RobotConfig()
mpc = leg.JumpingMPC(cfg)

# 2. Define Test Scenario
# Initial State: [y, q1, q2, dy, dq1, dq2]
# Using a "safe" sitting position
sim_state = np.array([0.38, 0.0, 2.4, 0.0, 0.0, 0.0]) 

# Schedule: 20 ticks of "Wait" (Mode 1)
test_schedule = np.array([1] * 20)

# 3. Run Debugger
print("Imported jumping_leg.py successfully.")
print("Running Debug Constraints check...")
mpc.debug_constraints(sim_state, test_schedule)