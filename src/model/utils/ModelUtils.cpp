#pragma once
#include <utility>
#include "ModelUtils.h"

namespace model {

	double clamp(double value, double min, double max) {
		return std::max(min, std::min(value, max));
	}
}