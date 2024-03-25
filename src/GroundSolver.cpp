#include "GroundSolver.h"

#include "Util.h"

using namespace RLCIS;
using namespace RLConst;
using namespace Util;

SolverResult RLCIS::SolveGround(const SolverCarState& fromState, const SolverCarState& toState, float deltaTime, const SolverConfig& config, Car* car) {
	constexpr Vec GRAVITY = Vec(0, 0, GRAVITY_Z);

	int tickDelta = deltaTime / RL_TICKTIME;
	int forceTickDelta = RS_MAX(tickDelta - 1, 1);

	SolverResult result = {};
	result.isOnGround = true;
	CarControls& controls = result.controls;

	Vec localVelFrom = fromState.rot.Dot(fromState.vel),
		localVelTo = toState.rot.Dot(toState.vel);
	Vec localAngVelFrom = fromState.rot.Dot(fromState.angVel),
		localAngVelTo = toState.rot.Dot(toState.angVel);
	float forwardSpeed = localVelFrom.x;

	Vec extrapVel = fromState.vel;

	{ 
		// Predicting future vel is hard because car suspension math is complicated, 
		//	but we can make a good-enough approximation by recognizing the following:

		// 1. Gravity is present
		extrapVel += GRAVITY * deltaTime * 2;

		// 2. Our wheels push back against gravity
		extrapVel -= fromState.rot.up * GRAVITY_Z * deltaTime;

		// 3. We lose some velocity that was in the previous direction when turning
		float velConserveFraction = RS_MAX(fromState.rot.forward.Dot(toState.rot.forward), 0);
		extrapVel *= velConserveFraction;

		// 4. We can't exceed max car speed
		extrapVel = LimitToMaxCarSpeed(extrapVel);
	}

	Vec deltaVel = toState.vel - extrapVel;
	Vec deltaVelLocal = fromState.rot.Dot(deltaVel);
	Vec localAccel = deltaVelLocal / deltaTime;
	float totalAccel = localAccel.Length();

	{ // Solve steer and set simToVel
		float maxSteerAngle = STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(abs(forwardSpeed));

		Vec localAngVelFrom = fromState.angVel * fromState.rot;
		Vec localAngVelTo = toState.angVel * toState.rot;
		float turnAccel = (localAngVelTo.z - localAngVelFrom.z) / deltaTime;

		// Sample 2 ang vel deltas for steer inputs 0 and 1
		// We don't need -1 because it is the same as (deltas[0] - (deltas[1] - deltas[0]))
		// This should be pretty inexpensive as there are no collision resolutions or ray traces occuring
		float steerTurnAccels[2];
		for (int i = 0; i < 2; i++) {
			float steer = i;
			float steerAngle = steer * maxSteerAngle;

			btVehicleRL& vehicle = car->_bulletVehicle;
			btRigidBody* rb = vehicle.m_chassisBody;

			for (int j = 0; j < 2; j++) {
				vehicle.m_wheelInfo[j].m_steerAngle = steerAngle;
				vehicle.updateWheelTransform(j);
			}

			btVector3
				backupLinVel = rb->m_linearVelocity,
				backupAngVel = rb->m_angularVelocity,
				backupForce = rb->m_totalForce,
				backupTorque = rb->m_totalTorque;

			btTransform backupTransform = rb->m_worldTransform;

			vehicle.calcFrictionImpulses(deltaTime);
			vehicle.applyFrictionImpulses(deltaTime);

			btMatrix3x3 rot = rb->m_worldTransform.m_basis;

			btVector3 localAngVelBefore = backupAngVel * rot;
			btVector3 localAngVelAfter = rb->m_angularVelocity * rot;
			float simTurnAccel = (localAngVelAfter.z() - localAngVelBefore.z()) / deltaTime;

			steerTurnAccels[i] = simTurnAccel;

			rb->m_linearVelocity = backupLinVel;
			rb->m_angularVelocity = backupAngVel;
			rb->m_totalForce = backupForce;
			rb->m_totalTorque = backupTorque;
			rb->m_worldTransform = backupTransform;
		}

		float zeroSteerTurnAccel = steerTurnAccels[0];
		float fullSteerTurnAccelDelta = steerTurnAccels[1] - zeroSteerTurnAccel;

		constexpr float MIN_TURN_ACCEL_DELTA = 0.02f;
		if (abs(fullSteerTurnAccelDelta) < MIN_TURN_ACCEL_DELTA)
			fullSteerTurnAccelDelta = MIN_TURN_ACCEL_DELTA * RS_SGN(fullSteerTurnAccelDelta);

		controls.steer = (turnAccel - zeroSteerTurnAccel) / fullSteerTurnAccelDelta;
	}

	float forwardAccel = localAccel.x;
	float forwardDir = RS_SGN(localVelFrom.x);
	float relForwardAccel = forwardAccel * forwardDir;

	{ // Solve throttle and boost

		constexpr float TORQUE_CONVERT_FACTOR = 4.f / 3.f; // Forget exactly why its this
		constexpr float DRIVE_ACCEL = THROTTLE_TORQUE_AMOUNT / (CAR_MASS_BT / 3) * TORQUE_CONVERT_FACTOR;
		constexpr float BRAKE_ACCEL = BRAKE_TORQUE_AMOUNT * TORQUE_CONVERT_FACTOR;

		float forwardSpeedFrom = fromState.vel.Dot(toState.rot.forward);
		float forwardSpeedTo = toState.vel.Dot(toState.rot.forward);

		// To account for gravity affecting our driving accel when on walls, we need to subtract gravity from our drive speed
		float gravitySpeed = (GRAVITY * deltaTime).Dot(toState.rot.forward);

		float driveAccel = ((forwardSpeedTo - gravitySpeed) - forwardSpeedFrom) / deltaTime;
		float driveSpeedScale = DRIVE_SPEED_TORQUE_FACTOR_CURVE.GetOutput(abs(forwardSpeed));

		if (relForwardAccel > 0) {
			// Accelerating

			float expectedThrottleAccel = DRIVE_ACCEL * driveSpeedScale;

			float throttleMag = abs(driveAccel) / expectedThrottleAccel;

			// Hard turns can cause a slight under-prediction of throttle
			constexpr float TURNING_THROTTLE_SCALE_ADD = 0.2f;
			constexpr float MAX_TURN_ANGVEL = 4.4f;
			float turnScale = RS_MIN(abs(localAngVelFrom.z) / MAX_TURN_ANGVEL, 1);
			throttleMag *= 1 + TURNING_THROTTLE_SCALE_ADD * turnScale;

			controls.throttle = RS_CLAMP(throttleMag, 0, 1) * forwardDir;

			constexpr float BOOST_ACCEL_THRESH = 100;
			if (controls.throttle == 1 && relForwardAccel > expectedThrottleAccel + BOOST_ACCEL_THRESH) {
				// We are accelerating much faster than driving acceleration
				// It is probably boost
				controls.boost = 1;
			}
		} else {
			// We are slowing down
			// Could either be from brake input or from coasting-brake

			float brakeAccel = abs(driveAccel);

			float expectedBrakeAccel = BRAKE_TORQUE_AMOUNT * TORQUE_CONVERT_FACTOR;
			float expectedCoastAccel = expectedBrakeAccel * COASTING_BRAKE_FACTOR;

			// Since this is a binary option, we'll decide based on which accel value is closer
			if (brakeAccel - expectedCoastAccel > (expectedBrakeAccel - expectedCoastAccel) / 2) {
				controls.throttle = -1 * forwardDir;

				if (controls.throttle == 1 && brakeAccel > expectedBrakeAccel * 1.25f) {
					// We are de-accelerating faster than brake
					// It is probably boost
					controls.boost = 1;
				}

			} else {

				// Special case for ambiguous case when driving at max drive speed
				constexpr float MIN_COAST_BRAKE_ACCEL = 30.f;
				if (brakeAccel < MIN_COAST_BRAKE_ACCEL && driveSpeedScale < 0.01f) {
					controls.throttle = 1 * forwardDir;
				} else {
					controls.throttle = 0;
				}
			}
		}
	}

	{ // Solve handbrake

		// Don't bother if we aren't really moving
		// Misalignment can occur easily when we are operating on tiny values
		// It's virtually impossible to determine powerslide when the car is near-still anyway
		constexpr float MIN_HANDBRAKE_VEL = 100;
		constexpr float MIN_HANDBRAKE_ANGVEL = 0.5f;

		if (toState.vel.Length() > MIN_HANDBRAKE_VEL || toState.angVel.Length() > MIN_HANDBRAKE_VEL) {

			Vec localVelFrom = fromState.rot.Dot(fromState.vel),
				localVelTo = toState.rot.Dot(toState.vel);

			// How aligned our velocity is with the direction we are facing
			float
				velAlignmentFrom = abs(localVelFrom.x) / (abs(localVelFrom.x) + abs(localVelFrom.y)),
				velAlignmentTo = abs(localVelTo.x) / (abs(localVelTo.x) + abs(localVelTo.y));

			constexpr float HANDBRAKE_ALIGNMENT_SCALE = 0.15f;
			float
				expectedAlignmentDrop = (POWERSLIDE_RISE_RATE * deltaTime) * HANDBRAKE_ALIGNMENT_SCALE,
				expectedAlignmentRise = (POWERSLIDE_FALL_RATE * deltaTime) * HANDBRAKE_ALIGNMENT_SCALE;

			if (velAlignmentTo < velAlignmentFrom - expectedAlignmentDrop) {
				// Alignment is decreasing
				controls.handbrake = true;
			} else if (velAlignmentTo > velAlignmentFrom + expectedAlignmentRise) {
				// Alignment is increasing
				controls.handbrake = false;
			} else {
				// If alignment is below this factor, and not increasing, we are almost certainly powersliding
				constexpr float HANDBRAKE_ALIGNMENT_THRESH = 0.94f;
				controls.handbrake = velAlignmentTo < HANDBRAKE_ALIGNMENT_THRESH;
			}
		}
	}

	{ // Solve jumping

		// TODO: Lame magic numbers
		constexpr float MIN_JUMPING_VEL_DELTA = 100;
		constexpr float MAX_FROM_Z_VEL = 50;
		constexpr float MIN_TO_Z_VEL = MIN_JUMPING_VEL_DELTA * 0.7f;
		
		if (abs(localVelFrom.z) < MAX_FROM_Z_VEL && localVelTo.z > MIN_TO_Z_VEL && deltaVelLocal.z > MIN_JUMPING_VEL_DELTA) {
			controls.jump = true;
		}
	}

	if (config.steerIsYaw)
		controls.yaw = controls.steer;

	return result;
}