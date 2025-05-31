#pragma once
#include <utility>
#include <model/utils/ModelUtils.h>

namespace model {
	class IVehicleModel {
	public:
		virtual std::pair<Pose, Twist> updateCoords() = 0;
	};
}