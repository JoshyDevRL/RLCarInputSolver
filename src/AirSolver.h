#pragma once
#include "Solver.h"

namespace RLCIS {
	SolverResult SolveAir(const SolverCarState& fromState, const SolverCarState& toState, float deltaTime, const SolverConfig& config);
}