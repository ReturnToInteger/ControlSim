#pragma once  
#include <cmath>

namespace model::diffDrive {  
	static constexpr double diskInertia(double mass, double radius) {
		return 0.5 * mass * radius*radius;
	}
	struct WheelModel {  
		static constexpr double diameter = 0.055; // m  
		static constexpr double radius = diameter / 2;
		static constexpr double mass = 0.070; // kg  
		static constexpr double inertia = diskInertia(mass, radius); // kg * m^2, for a disk
	};  
}