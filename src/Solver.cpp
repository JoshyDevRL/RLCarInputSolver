#include "Solver.h"

#include "AirSolver.h"
#include "GroundSolver.h"
#include "Util.h"

using namespace RLCIS;

RLCIS::SolverCarState::SolverCarState(const CarState& carState)
	: pos(carState.pos), rot(carState.rotMat), vel(carState.vel), angVel(carState.angVel) {

}

RLCIS::SolverCarState::operator CarState() const {
	CarState cs = {};
	cs.pos = pos;
	cs.rotMat = rot;
	cs.vel = vel;
	cs.angVel = angVel;
	return cs;
}

//////////////////////////////////////////////////////////////////

thread_local Arena* g_ThreadArena = NULL;
thread_local Car* g_ThreadCar = NULL;

void RLCIS::Init() {
	RocketSim::Init("./collision_meshes");
}

//////////////////////////////////////////////////////////////////

bool CheckOnGround(const SolverCarState& carState) {
	if (!g_ThreadArena) {
		g_ThreadArena = Arena::Create(GameMode::SOCCAR, ArenaMemWeightMode::HEAVY, RL_TICKRATE);
		g_ThreadCar = g_ThreadArena->AddCar(Team::BLUE);

		// Remove ball from arena so it doesn't get in our way
		BallState bs = {};
		bs.pos.z = -500;
		g_ThreadArena->ball->SetState(bs);
	}

	g_ThreadCar->SetState((CarState)carState);

	// We don't want to waste performance actually stepping the arena
	// Plus, that would move the car!
	// Instead we can call _PreTickUpdate() on the car to cause the wheel traces to run, 
	//	and _PostTickUpdate() to update the car's isOnGround
	g_ThreadCar->_PreTickUpdate(
		g_ThreadArena->gameMode, g_ThreadArena->tickTime,
		g_ThreadArena->_mutatorConfig, &g_ThreadArena->_suspColGrid
	);
	g_ThreadCar->_PostTickUpdate(
		g_ThreadArena->gameMode, g_ThreadArena->tickTime, g_ThreadArena->_mutatorConfig
	);

	bool onGround = g_ThreadCar->_internalState.isOnGround;

	// Undo potential changes to the simulated car state
	// We might want to use this simulated car later on
	g_ThreadCar->SetState((CarState)carState);

	return onGround;
}

SolveResult RLCIS::Solve(const SolverCarState& fromState, const SolverCarState& toState, float deltaTime, const SolverConfig& config) {
	using namespace Util;

	bool onGround = CheckOnGround(toState);
	// We don't need to check for the other state, it's not very helpful

	if (deltaTime < g_ThreadArena->tickTime)
		RS_ERR_CLOSE("RLCIS::Solve(): Cannot solve for delta time less than RL tick time (1 / " << RL_TICKRATE << ")");

	SolveResult result = {};
	CarControls& controls = result.controls;
	if (onGround) {
		assert(g_ThreadCar != NULL);
		result = SolveGround(fromState, toState, deltaTime, config, g_ThreadCar);
	} else {
		result = SolveAir(fromState, toState, deltaTime, config);
	}

	{ // Apply input deadzone
		Deadzone(
			{
				&controls.throttle,
				&controls.steer,

				&controls.pitch,
				&controls.yaw,
				&controls.roll
			},

			config.inputDeadzone,
			config.inputInverseDeadzone
		);
	}

	controls.ClampFix();
	return result;
}