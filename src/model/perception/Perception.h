#pragma once
#include <vector>
#include <src/model/Cone.h>
#include <corecrt_math_defines.h>
#include <src/model/utils/ModelUtils.h>

namespace model {
	class Perception
	{
	private:
		const std::vector<model::Cone>& _cones;
		const double _angle;
		const double _depth;
	public:
		Perception(const std::vector<model::Cone>& cones, double angle=M_PI, double depth=40);
		~Perception() = default;
		std::vector<const model::Cone*> detect(const model::Pose & pose);
		std::vector<model::Cone> getCones() { return _cones; }

	};
}
