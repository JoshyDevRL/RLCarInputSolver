#include "AirSolver.h"

#include "Util.h"

using namespace RLCIS;
using namespace RLConst;
using namespace Util;

// Code by the legendary Sam Mish, from https://www.smish.dev/rocket_league/inverse_aerial_control/
// NOTE: Modified to produce full-magnitude inputs when at max angvel
Vec ReverseAirOrientInputs(Vec angVelBefore, Vec angVelAfter, RotMat rot, float dt) {

	if (IsNear(angVelAfter.LengthSq(), CAR_MAX_ANG_SPEED * CAR_MAX_ANG_SPEED, 0.01f)) {
		// Scale up resulting angular vel
		// Otherwise this function will output partial inputs if you are turning at max speed
		angVelAfter *= 1.25f;
	}

	RotMat rotTrans = rot.Transpose();

	constexpr Vec T = { -36.0796f, -12.1460f, 8.9196f };
	constexpr Vec D = { -4.47166,  -2.7982f, -1.8865f };

	// net torque in world coordinates
	Vec tau = (angVelAfter - angVelBefore) / dt;

	// net torque in local coordinates
	tau = rotTrans * tau;

	// beginning-step angular velocity, in local coordinates
	Vec omega_local = rotTrans * angVelBefore;

	Vec rhs = tau - D * omega_local;

	// user inputs: roll, pitch, yaw
	Vec result = {
		rhs[0] / T[0],
		rhs[1] / (T[1] + RS_SGN(rhs[1]) * omega_local[1] * D[1]),
		rhs[2] / (T[2] - RS_SGN(rhs[2]) * omega_local[2] * D[2])
	};

	for (int i = 0; i < 3; i++)
		result[i] = RS_CLAMP(result[i], -1, 1);

	return result;
}

SolveResult RLCIS::SolveAir(const SolverCarState& fromState, const SolverCarState& toState, float deltaTime, const SolverConfig& config) {
	constexpr Vec GRAVITY = Vec(0, 0, GRAVITY_Z);
	constexpr float TICK_FORCES_SCALE = 1 / 2.4f;

	int tickDelta = deltaTime / RL_TICKTIME;
	int forceTickDelta = RS_MAX(tickDelta - 1, 1);

	float forcesScale = TICK_FORCES_SCALE * tickDelta;

	SolveResult result = {};
	result.isOnGround = false;
	CarControls& controls = result.controls;

	Vec extrapPos = fromState.pos + (GRAVITY * (deltaTime * deltaTime) / 2);
	Vec extrapVel = LimitToMaxCarSpeed(fromState.vel + (GRAVITY * deltaTime));

	Vec deltaVel = toState.vel - extrapVel;
	Vec deltaVelLocal = fromState.rot.Dot(deltaVel);

	// Determine throttle and boost
	{
		// If other local deltas are greater than this value, it is probably not boosting
		constexpr float MIN_BOOST_OR_THROTTLE_OTHER_DELTAS = 6;

		if ((abs(deltaVelLocal.y) + abs(deltaVelLocal.z)) < MIN_BOOST_OR_THROTTLE_OTHER_DELTAS) {

			// If the forward vel delta is less than this amount, we are probably not boosting forward
			// This will miss cases where we are max speed in the direction we are boosting,
			//	but the alternative is to assume boosting in those cases, which is worse.
			constexpr float MIN_FORWARD_DELTA = 2;
			if (deltaVelLocal.x > MIN_FORWARD_DELTA) {
				float expectedBoostAccel = BOOST_ACCEL * forcesScale;
				Vec expectedBoostVel = LimitToMaxCarSpeed(extrapVel + fromState.rot.forward * expectedBoostAccel);

				if (expectedBoostVel.Dist(toState.vel) < (expectedBoostAccel / 2))
					controls.boost = true;
			}

			if (!controls.boost) {
				float expectedThrottleAccel = THROTTLE_AIR_FORCE * forcesScale;
				if (IsNear(abs(deltaVelLocal.x), expectedThrottleAccel, 0.2f, 0.6f))
					controls.throttle = deltaVelLocal.x / expectedThrottleAccel;
			}
		}
	}

	// Check flip
	result.flipStarted = false;
	{
		float flatDeltaVel = deltaVel.Dist2D({});
		if (IsNear(
			flatDeltaVel,
			FLIP_INITIAL_VEL_SCALE, 0.3f, FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE + 0.3f
		)) {

			float forwardSpeed = fromState.vel.Dot(fromState.rot.forward);
			float forwardSpeedRatio = abs(forwardSpeed) / CAR_MAX_SPEED;

			RotMat flatRot = RotMat::LookAt((fromState.rot.forward * Vec(1, 1, 0)).Normalized(), Vec(0, 0, 1));

			Vec deltaVelFlip = flatRot.Dot(deltaVel);

			bool isBackwardsDodge;
			if (abs(forwardSpeed) < 100) {
				isBackwardsDodge = deltaVelFlip.x < 0;
			} else {
				isBackwardsDodge = (deltaVelFlip.x >= 0) != (forwardSpeed >= 0);
			}

			float maxSpeedScaleX = isBackwardsDodge ? FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE : FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE;
			deltaVelFlip.x /= ((maxSpeedScaleX - 1) * forwardSpeedRatio) + 1.f;
			deltaVelFlip.y /= ((FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE - 1) * forwardSpeedRatio) + 1.f;

			float flipDirForward = deltaVelFlip.x / RS_MAX(FLIP_INITIAL_VEL_SCALE, flatDeltaVel);
			float flipDirRight = deltaVelFlip.y / RS_MAX(FLIP_INITIAL_VEL_SCALE, flatDeltaVel);

			float pitch = -flipDirForward;
			float yaw = flipDirRight;

			// Assume flip was done with maximum input
			// The truth, as far as I can tell, is impossible to know
			float scaleRatio = 1 / RS_MAX(abs(pitch), abs(yaw));
			controls.pitch = pitch * scaleRatio;
			controls.yaw = yaw * scaleRatio;
			controls.jump = true;

			result.flipStarted = true;
		}
	}

	// Check double jump
	result.doubleJumping = false;
	if (!result.flipStarted && IsNear(deltaVelLocal.z, RLConst::JUMP_IMMEDIATE_FORCE, 0.3f)) {
		result.doubleJumping = true;
		controls.jump = true;
	}

	if (!result.flipStarted && !result.doubleJumping) {
		Vec aerialInputs = ReverseAirOrientInputs(fromState.angVel, toState.angVel, fromState.rot, deltaTime);

		controls.roll = aerialInputs[0];
		controls.pitch = aerialInputs[1];
		controls.yaw = aerialInputs[2];
	}

	// In-flip detection
	result.isFlipping = false;
	if (!result.flipStarted && !result.doubleJumping) {

		constexpr float TICK_GRAV = GRAVITY_Z * RL_TICKTIME;
		constexpr float FLIP_Z_DAMP_SCALE = (1 - FLIP_Z_DAMP_120);
		float expectedZVel;
		if (tickDelta > 1) {
			// Thank you Lie Algebra Cow!
			// "m^n * x + b * (1-m^n)/(1-m)"
			expectedZVel =
				powf(FLIP_Z_DAMP_SCALE, tickDelta) * (fromState.vel.z) +
				TICK_GRAV * ((1 - powf(FLIP_Z_DAMP_SCALE, tickDelta)) / (1 - FLIP_Z_DAMP_SCALE));
		} else {
			expectedZVel = fromState.vel.z * FLIP_Z_DAMP_SCALE + TICK_GRAV;
		}

		if (IsNear(toState.vel.z, expectedZVel, 0.4f)) {
			// We are in a flip
			result.isFlipping = true;

			Vec angVelLocalFrom = fromState.angVel * fromState.rot;

			float yawAngVel = angVelLocalFrom.z;
			float rollAngVel = -angVelLocalFrom.x;

			// Two conditions need to be met for this to be a stall
			bool isStall = 
				// 1. We have some velocity for both yaw and roll
				(abs(yawAngVel) > 0.2f && abs(rollAngVel) > 0.2f) &&

				// 2. We are rotating along yaw and roll in opposite input directions
				(RS_SGN(yawAngVel) != RS_SGN(rollAngVel));

			if (isStall) {
				controls.yaw = RS_SGN(yawAngVel);
				controls.roll = -controls.yaw;

				controls.jump = true;
			} else {
				// Check for flip cancel

				Vec angVelLocalTo = toState.angVel * toState.rot;

				float pitchDelta = abs(angVelLocalFrom.y - angVelLocalTo.y);
				constexpr float MIN_PITCH_DELTA_PER_TICK = 0.05f;
				if (abs(angVelLocalFrom.y) > abs(angVelLocalTo.y) + (MIN_PITCH_DELTA_PER_TICK * tickDelta)) {
					// We are cancelling
					// TODO: Detect/reproduce partial cancels?
					controls.pitch = RS_SGN(angVelLocalFrom.y);
				}
			}
		}
	}

	// Detect continued jump from when on ground
	// It is important that this detection does not produce false positives,
	//	otherwise inputs will be produced that could cause random airborne flips/double jumps
	if (!result.flipStarted && !result.isFlipping && !result.doubleJumping) {
		float expectedJumpAccel = JUMP_ACCEL * deltaTime;
		float maxOtherAccel = expectedJumpAccel / 10;
		if (IsNear(deltaVelLocal.z, expectedJumpAccel, 0.35f) && (abs(deltaVelLocal.x) + abs(deltaVelLocal.y)) < maxOtherAccel)
			controls.jump = true;

	}

	if (config.steerIsYaw)
		controls.steer = controls.yaw;

	return result;
}