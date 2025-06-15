#pragma once
#include <vector>
//#include <model/utils/ModelUtils.h>
#include <corecrt_math_defines.h>
#include "model/items/Cone.h"
#include "model/utils/Angle.h"
#include <unordered_set>

namespace model {
	struct Pose;

	// Simple perception model simulating the detection of cones and a camera
	class Perception
	{
	private:
		std::vector<model::Cone> const& _cones;
		const double _viewAngle;
		const double _depth;
	public:
		Perception(std::vector<model::Cone> const& cones, Angle viewAngle=M_PI*2.0/3.0, double depth=40);
		~Perception() = default;
		std::unordered_set<model::Cone const*> detect(model::Pose const& pose);

	};
}
