#include "PYB.h"

using namespace RLCIS;


PYB_INIT_F(SolverTypes) {
#define PYB_CUR_CLASS SolverCarState
	PYB_CLASS(SolverCarState)
		PYB_DEFAULT_INITS()
		PYBP(pos)
		PYBP_RN(rot, rot_mat)
		PYBP(vel)
		PYBP(angVel)
		;

#define PYB_CUR_CLASS SolverConfig
	PYB_CLASS(SolverConfig)
		PYB_DEFAULT_INITS()
		PYBP(inputDeadzone)
		PYBP(inputInverseDeadzone)

		PYBP(applyDeadzones)
		PYBP(clampControls)

		PYBP(steerIsYaw)
		;

#define PYB_CUR_CLASS SolveResult
	PYB_CLASS(SolveResult)
		PYB_DEFAULT_INITS()
		PYBP(controls)

		PYBP(isOnGround)

		PYBP(flipStarted)
		PYBP(isFlipping)
		PYBP(doubleJumping)
		;
}