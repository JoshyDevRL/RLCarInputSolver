#pragma once
#include "Solver.h"

namespace RLCIS {
	SolveResult SolveGround(const SolverCarState& fromState, const SolverCarState& toState, float deltaTime, const SolverConfig& config, Car* car);
}