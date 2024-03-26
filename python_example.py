import rlcis_py
from rlcis_py import Vec, RotMat, CarControls, SolverCarState, SolverConfig, SolveResult

# Let's make up two car states to test the solver

# The delta time between our states will be 1/30th of a second, the replay frame interval
delta_time = 1 / 30

states = []
for i in range(2):
	# On ground at center of field
	pos = Vec(0, 0, 17)
	
	# Identity rotmat, forward is +X
	rot = RotMat(Vec(1, 0, 0), Vec(0, 1, 0), Vec(0, 0, 1)) 
	
	# Moving forward slowly along +X
	vel = Vec(300, 0, 0)
	
	# Second state has added X velocity, scaled by delta time
	if i == 1:
		vel.x += 1000 * delta_time
		
	ang_vel = Vec(0, 0, 0)
	
	states.append(
		SolverCarState(pos, rot, vel, ang_vel)
	)

# Make a configuration for the solve
config = SolverConfig()
# Reasonable deadzone, analog inputs lower than this will be zeroed
config.input_deadzone = 0.1 

# Solve it!
solve_result = rlcis_py.solve(states[0], states[1], delta_time, config)

print("Solved controls:", solve_result.controls)
print("Solved on ground:", solve_result.is_on_ground)