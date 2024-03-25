#pragma once
#include "Solver.h"

namespace RLCIS {
	SolveResult SolveAir(const SolverCarState& fromState, const SolverCarState& toState, float deltaTime, const SolverConfig& config);
}