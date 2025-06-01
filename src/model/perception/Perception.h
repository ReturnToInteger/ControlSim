#pragma once
#include <vector>
//#include <model/utils/ModelUtils.h>
#include <corecrt_math_defines.h>
#include "model/items/Cone.h"

namespace model {
	struct Pose;
	class Perception
	{
	private:
		const std::vector<model::Cone>& _cones;
		const double _viewAngle;
		const double _depth;
	public:
		Perception(const std::vector<model::Cone>& cones, double viewAngle=M_PI*2.0/3.0, double depth=40);
		~Perception() = default;
		std::vector<const model::Cone*> detect(const model::Pose & pose);
		//std::vector<model::Cone> getCones() { return _cones; }

	};
}
