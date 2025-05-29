#pragma once
#include <vector>
#include <model/items/Cone.h>
#include <corecrt_math_defines.h>
#include <model/utils/ModelUtils.h>

namespace model {
	class Perception
	{
	private:
		const std::vector<model::Cone>& _cones;
		const double _viewAngle;
		const double _depth;
	public:
		Perception(const std::vector<model::Cone>& cones, double viewAngle=M_PI, double depth=40);
		~Perception() = default;
		std::vector<const model::Cone*> detect(const model::Pose & pose);
		std::vector<model::Cone> getCones() { return _cones; }

	};
}
