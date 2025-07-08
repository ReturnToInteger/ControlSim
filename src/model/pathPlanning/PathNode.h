#pragma once
#include <memory>
#include "model/IVehicleState.h"

namespace model {
	
	struct PathNode
	{
		// State at the node
		std::unique_ptr<IVehicleState> state;
		// Current cost from start
		double gCost=0;
		// Estimated goal (Current cost + heuristics)
		double fCost= INFINITY;
		// Parent for reconstruction
		std::shared_ptr<PathNode const> parent=nullptr;
		// For multistage support
		int stage=0;

		bool operator<(PathNode const& other) const {
			return fCost< other.fCost;
		}
		// Default constructor
		PathNode() = default;

		// Copy constructor
		PathNode(const PathNode& other)
			: state(other.state ? other.state->clone() : nullptr),
			gCost(other.gCost),
			fCost(other.fCost),
			parent(other.parent),
			stage(other.stage)
		{
		}

		// Copy assignment
		PathNode& operator=(const PathNode& other) {
			if (this != &other) {
				state = other.state ? other.state->clone() : nullptr;
				gCost = other.gCost;
				fCost = other.fCost;
				parent = other.parent;
				stage = other.stage;
			}
			return *this;
		}

		// Move constructor (defaulted)
		PathNode(PathNode&&) noexcept = default;

		// Move assignment (defaulted)
		PathNode& operator=(PathNode&&) noexcept = default;

		// Destructor (defaulted)
		~PathNode() = default;
	};

	// Used in the priority queue, to find the minimum fCost in O(1) time
	struct PQNode {
		// Estimated goal (Current cost + heuristics)
		double fCost;
		// Key for the hashmap, discretization of the pose variable
		std::tuple<int,int,int,int> key;
		// Comparison to find the best candidate from the open list, used in the priority queue
		bool operator>(PQNode const& other) const {
			return fCost > other.fCost;
		}
	};

}