#pragma once
#include "Solver.h"

namespace RLCIS {
	SolverResult SolveGround(const SolverCarState& fromState, const SolverCarState& toState, float deltaTime, const SolverConfig& config, Car* car);
}