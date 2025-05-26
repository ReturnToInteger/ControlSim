#pragma once
#include <memory>
#include "src/model/VehicleState.h"

namespace model {
	struct PathNode
	{
		VehicleState state;
		double cost;
		double heuristic;
		std::shared_ptr<PathNode> parent;
		PathNode(const VehicleState& state, double c, double h, PathNode* p)
			: state(state), cost(c), heuristic(h), parent(p) {
		}
		PathNode() : cost(INFINITY), heuristic(INFINITY), parent(nullptr) {}

		bool operator<(const PathNode& other) const {
			return heuristic < other.heuristic;
		}
	};
}