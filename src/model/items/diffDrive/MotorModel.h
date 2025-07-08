#pragma once
#include "model/utils/constants.h"
#include <numbers>

namespace model::diffDrive {
	static constexpr double kgCmToNm(double kgCm) {
		return kgCm * model::g / 100;
	}
	static constexpr double rpmToRadPerSec(double rpm) {
		return rpm * 2 * std::numbers::pi / 60;
	}
	static constexpr double radPerSecToRPM(double radPerSec) {
		return radPerSec / (2 * std::numbers::pi) * 60;
	}
	struct MotorModel {
		static constexpr double maxVoltage = 12; // V
		static constexpr double angularVelNoLoad = rpmToRadPerSec(130); // RPM to rad/sec
		static constexpr double currentNoLoad = 0.11; // A
		static constexpr double angularVelRated = rpmToRadPerSec(100); // RPM to rad/sec
		static constexpr double currentRated = 0.18; // A
		static constexpr double torqueRated = kgCmToNm(1.5); // kg * cm to N*m
		static constexpr double torqueLocked = kgCmToNm(9); // kg * cm to N*m
		static constexpr double currentLocked = 1; // A
	};
}