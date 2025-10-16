#pragma once
#include <vector>
//#include <model/utils/ModelUtils.h>
#include <numbers>
#include "model/items/obstacles/Obstacle.h"
#include "model/utils/Angle.h"
#include <unordered_set>
#include "model/mapReaders/Map.h"

namespace model {
	struct Pose;

	// Simple perception model simulating the detection of cones and a camera
	class Perception
	{
	private:
		model::Map const& m_map;
		const double m_viewAngle;
		const double m_depth;
	public:
		Perception(model::Map const & map, Angle viewAngle=std::numbers::pi*2.0/3.0, double depth=40);
		~Perception() = default;
		std::unordered_set<model::Obstacle const*> detect(model::Pose const& pose);

	};
}
