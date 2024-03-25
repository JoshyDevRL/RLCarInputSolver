#pragma once
#include "Framework.h"

namespace RLCIS {
	namespace Util {
		inline bool IsNear(float val, float target, float marginLess, float marginMore) {
			return (val > target - abs(target * marginLess)) && (val < target + abs(target * marginMore));
		}

		inline bool IsNear(float val, float target, float margin) {
			return IsNear(val, target, margin, margin);
		}

		inline float Deadzone(float val, float deadzone, float inverseDeadzone) {
			float absVal = abs(val);
			if (absVal < deadzone)
				return 0;

			if (absVal > inverseDeadzone)
				return 1 * RS_SGN(val);

			return val;
		} 

		inline void Deadzone(std::initializer_list<float*> vals, float deadzone, float inverseDeadzone) {
			for (float* val : vals)
				*val = Deadzone(*val, deadzone, inverseDeadzone);
		}

		inline Vec LimitToMaxCarSpeed(Vec vel) {
			using namespace RLConst;

			float lenSq = vel.LengthSq();
			if (lenSq > CAR_MAX_SPEED * CAR_MAX_SPEED) {
				return vel / sqrtf(lenSq) * CAR_MAX_SPEED;
			} else {
				return vel;
			}
		}
	}
};