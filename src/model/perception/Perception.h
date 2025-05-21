#pragma once
#include <vector>
#include <src/model/Cone.h>
#include <corecrt_math_defines.h>
namespace model {
	class Perception
	{
	private:
		std::vector<model::Cone>& _cones;
		const double _angle;
		const double _depth;
	public:
		Perception(std::vector<model::Cone>& cones, double angle=M_PI, double depth=20);
		~Perception() = default;
		std::vector<model::Cone*> detect(const model::Point& position, double orientation);

	};
}
