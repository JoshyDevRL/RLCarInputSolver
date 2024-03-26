#pragma once
#include "Framework.h"

namespace RLCIS {
	constexpr float
		RL_TICKRATE = 120.f,
		RL_TICKTIME = 1 / RL_TICKRATE;

	void Init();

	// All state information needed for the solver
	struct SolverCarState {
		Vec pos;
		RotMat rot;
		Vec vel, angVel;

		SolverCarState() = default;
		SolverCarState(Vec pos, RotMat rot, Vec vel, Vec angVel) 
			: pos(pos), rot(rot), vel(vel), angVel(angVel) {}
		explicit SolverCarState(const CarState& carState);

		explicit operator CarState() const;
	};

	struct SolverConfig {
		// Analog inputs below this theshold will be zeroed
		float inputDeadzone = 0.1f;

		// Analog inputs above this threshold will be made 1.0
		float inputInverseDeadzone = 0.95f;

		bool applyDeadzones = true; // Apply inputDeadzone and inputInverseDeadzone
		bool clampControls = true; // Clamp analog inputs from -1 to 1

		// Steer and yaw will be assumed to always be equal
		// When on the ground, yaw will be set to steer
		// When in the air, steer will be set to yaw
		bool steerIsYaw = true;
	};

	struct SolveResult {
		CarControls controls;

		bool isOnGround;

		bool flipStarted, isFlipping;
		bool doubleJumping;

		SolveResult() = default;
	};

	SolveResult Solve(const SolverCarState& fromState, const SolverCarState& toState, float deltaTime, const SolverConfig& config);
}