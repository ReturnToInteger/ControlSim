#pragma once
#include <utility>
#include "model/utils/ModelUtils.h"
#include "model/utils/Pose.h"

namespace model {
	class IVehicleModel {
	public:
		virtual std::pair<Pose, Twist> updateCoords() = 0;
		virtual ~IVehicleModel() = default;
	};
}