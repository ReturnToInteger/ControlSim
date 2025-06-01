#pragma once
#include <memory>
#include "model/VehicleState.h"

namespace model {
	struct PathNode
	{
		VehicleState state;
		double gCost;
		double fCost;
		std::shared_ptr<PathNode> parent;
		PathNode(const VehicleState& state, double g, double f, PathNode* p)
			: state(state), gCost(g), fCost(f), parent(p) {
		}
		PathNode() : gCost(INFINITY), fCost(INFINITY), parent(nullptr) {}

		bool operator<(const PathNode& other) const {
			return fCost< other.fCost;
		}
	};

	struct PQNode {
		double fCost;
		std::tuple<int,int,int> key;

		bool operator>(const PQNode& other) const {
			return fCost > other.fCost;
		}
	};

}