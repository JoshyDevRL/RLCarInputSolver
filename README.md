# RLCarInputSolver
An easy-to-use C++ library for determining the controller inputs between two sequential Rocket League car states

## * WORK IN PROGRESS *
This library is still a work in progress and will likely change heavily as I continue to improve it.
Let me know about any bugs you encounter so I can fix them.

## Usage
Simply call the `RLCIS::Solver::Solve()` function with two car states, and the deltatime between them.
Car states are in the following format:
```cpp
struct SolverCarState {
	Vec pos, vel, angVel;
	RotMat rot;
}
```
That's it. There is no persistent data between solves.

For Python: [python_example.py](python_example.py)

## How it works
The solver uses a variety of heuristics and math to determine:
- Throttle input (all surfaces, and air)
- Steering input (all surfaces)
- Powerslide input (all surfaces)
- Boost input (ground and air)
- Jumping from the ground (including held jumps and short jumps)
- Double-jumping
- Aerial pitch, yaw, and roll inputs
- Flips, and their input
- Flip cancels
- Stalls (empty flips with no torque or force)

Partial analog input is fully supported.

## Accuracy
For 99% of gameplay, the accuracy *should* be enough for most purposes, however I am sure there are many bugs I am not yet aware of.
There are certain situations that will consistently confuse the solver, as well as quite a few ambiguous cases where it is impossible to know what input was used.
Here are some known problems:
 - (Ambiguous Case) Flip cancels of diagonal flips are only detected until the car no longer has pitch torque (this means speedflip cancels will not be detected once the car is only spinning along the roll axis)
 - (Ambiguous Case) Detected of stalls is delayed by the flip z-damping delay (0.15s), as that is the only indication that a stall is occuring
 - (Difficult To Discern) Powerslide is currenty not detected when there is no angular velocity
 - (Difficult To Discern) Jump detection during landings at varied angles is inconsistent
 - (Difficult To Discern) Strong collision against cars or the arena when in the air can cause the solver to assume the car is flipping or double-jumping

Having optional persistent data will likely be implemented in the future to address many of these problems.
You should test and check the outputs of this library to make sure the accuracy is good enough for your use case, and feel free to contact me if it isn't so that I know what to focus on.