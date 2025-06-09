#pragma once  
#include <vector>  
#include <iostream>  
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>

#include "model/VehicleState.h"
#include "model/pathPlanning/PathNode.h"
#include "model/pathPlanning/heuristics/DubinsStateSpace.h"
#include "model/items/Path.h"
#include "model/utils/FixSizedQueue.h"

#ifndef MAX_CONTAINER_SIZE
#define MAX_CONTAINER_SIZE 10000
#endif // !MAX_CONTAINER_SIZE
namespace model {
	class Cone;
	namespace pathPlanning {
		class PathPlanner
		{
		public:
			PathPlanner(int iterations = 5, double deltaSpace = 1.5, double cellSize = 0.5, double steeringStep = M_PI / 18);
			~PathPlanner() = default;
			void planPath(std::vector<const model::Cone*> const& cones, model::VehicleState const& vehicleState);
			//Sets planned path based on current node
			void setPlannedPath();
			model::FixSizedQueue<Path> getPlannedPath() const;
			void setGoal(Point const& goal);
			void clear();
			double getCellSize() const { return _cellSize; }

		private:
			// Storing nodes based on discrete pair of coordinates (x,y)
			std::unordered_map<std::tuple<int, int, int>, PathNode> _openList;
			std::priority_queue<PQNode,
				std::vector<PQNode>,
				std::greater<PQNode>> _openQueue;
			std::unordered_set<std::tuple<int, int, int>> _closedList;
			std::unordered_set<std::tuple<int, int, int>> _collidingList;
			model::FixSizedQueue<Path> _plannedPaths;
			std::vector<double> _plannedOrientation;
			const int _iterations;
			const double _deltaSpace;
			const double _steeringStep;
			const double _cellSize;
			std::array<double, 11> _steeringAngles;
			Point _goal;
			PathNode _currentNode;
			DubinsStateSpace _dubins;

			void _updateNeightbours(std::vector<const model::Cone*> const& cones, PathNode& node);
			VehicleState _stepByDistance(VehicleState const& state, double distance, double steeringAngle);
			std::tuple<int, int, int> _discretizePoint(model::Pose const& pose) const;
			std::pair<bool, model::Pose> _detectCollision(VehicleState const& state, std::vector<const model::Cone*> const& cones) const;
			std::pair<bool, model::Pose> _lazyDetectCollision(VehicleState const& state, std::vector<const model::Cone*> const& cones) const;
			std::vector<model::Point> _getBoundary(VehicleState const& state) const;
			Point _rotatePoint(model::Point const& point, Angle const& angle) const;
			double _getHeuristics(model::VehicleState const& start, model::Point const& goal);
			// gets position-orientation pairs until new position is reached
			std::vector<model::VehicleState> _stepUntilNew(VehicleState& state, double distanceStep);

		};
	}
}
