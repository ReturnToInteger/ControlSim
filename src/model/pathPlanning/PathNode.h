#pragma once
#include <memory>
#include "model/VehicleState.h"

namespace model {
	
	struct PathNode
	{
		// State at the node
		VehicleState state;
		// Current cost from start
		double gCost;
		// Estimated goal (Current cost + heuristics)
		double fCost;
		// Parent for reconstruction
		std::shared_ptr<PathNode const> parent;
		int stage;
		PathNode(VehicleState const& state, double g, double f, PathNode* p,int stage)
			: state(state), gCost(g), fCost(f), parent(p), stage(stage){
		}
		PathNode() : gCost(INFINITY), fCost(INFINITY), parent(nullptr), stage(0) {}
		bool operator<(PathNode const& other) const {
			return fCost< other.fCost;
		}

		void* operator new(std::size_t size) {
			std::cout << "[PathNode new] " << size << " bytes\n";
			return std::malloc(size);
		}

		void operator delete(void* ptr) {
			std::cout << "[PathNode delete]\n";
			std::free(ptr);
		}
		//~PathNode() {
		//	std::cout << "[PathNode dtor] @" << this << "\n";
		//}

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