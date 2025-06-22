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

namespace model {
	class Cone;
	namespace pathPlanning {
		enum class SteeringMode
		{
			LOW,
			MEDIUM,
			HIGH,
			SQUARED
		}; 

		enum class Waypoints 
		{
			SINGLE,
			DOUBLE
		};

		struct SteeringPresets {
			static constexpr std::array<double, 5> steeringModeLOW = { 0, -0.5, 0.5, -1, 1 };
			static constexpr std::array<double, 9> steeringModeMEDIUM = { 0, -0.25, 0.25, -0.5, 0.5, -0.75, 0.75, -1, 1 };
			static constexpr std::array<double, 11> steeringModeHIGH = { 0.0, 0.2, -0.2, 0.4, -0.4, 0.6, -0.6, 0.8, -0.8, 1.0, -1.0 };
			static constexpr std::array<double, 13> steeringModeSQUARED = { 0.0, 0.05, -0.05, 0.2, -0.2, 0.4, -0.4, 0.6, -0.6, 1.0, -1.0 };
		};

		// cellSize
		// stepSize
		// angleBins
		// SteeringMode: 
		// --- linear=LOW, MEDIUM, HIGH
		// --- higher density near smaller angles: SQUARED
		// Waypoints: SINGLE - single goal planning. DOUBLE - goal + waypoint planning
		// maxContainerSize: limit the visited and unvisited nodes
		struct PlannerConfig {
			double cellSize;
			double stepSize;
			int angleBins;
			SteeringMode steeringMode;
			// Not tested in SINGLE mode currently
			Waypoints waypoints;
			int maxContainerSize = 1000;
		};

		class PathPlanner
		{
		public:
			PathPlanner(PlannerConfig config);
			bool planPath(std::unordered_set<const model::Cone*> const& cones, model::VehicleState const& vehicleState);
			//Sets planned path based on current node
			void setPlannedPath();
			Path getPlannedPath() const;
			void setGoal(Point const& goal);
			void clear();
			double getCellSize() const { return _cellSize; }


		private:
			// Storing nodes based on discrete pair of coordinates (x,y)
			std::unordered_map<std::tuple<int, int, int,int>, PathNode> _openList;
			std::priority_queue<PQNode,
				std::vector<PQNode>,
				std::greater<PQNode>> _openQueue;
			std::unordered_set<std::tuple<int, int, int,int>> _closedList;
			#ifdef ENABLE_DEBUG_DRAW
			std::unordered_set<std::tuple<int, int, int,int>> _collidingList;
			#endif // ENABLE_DEBUG_DRAW
			Path _plannedPath;
			const int _angleBins;
			const double _stepSize;
			const double _cellSize;
			const double* _steeringInputs;
			const int _maxContainerSize;
			size_t _anglesSize;

			FixSizedQueue<Point> _goals;
			PathNode _finalNode;
			DubinsStateSpace _dubins;

			void _selectSteeringMode(SteeringMode const& mode, double const*& steeringInputs, size_t& size);
			void _updateNeightbours(std::unordered_set<const model::Cone*> const& cones, PathNode const& node, int stage);
			VehicleState _stepByDistance(VehicleState const& state, double distance, double steeringInput);
			std::tuple<int, int, int,int> _discretizePoint(model::Pose const& pose, int stage) const;
			// Returns if it collides with 100% accuracy, and vehicle pose
			// More computationally expensive
			std::pair<bool, model::Pose> _detectCollision(VehicleState const& state, std::unordered_set<const model::Cone*> const& cones) const;

			// Simple collision detection that uses vehicle width
			// Less computationally expensive
			std::pair<bool, model::Pose> _lazyDetectCollision(VehicleState const& state, std::unordered_set<const model::Cone*> const& cones) const;

			//		Detect collision between a node and one step away by interpolating
			// Returns the first collision if it happens, and the vehicle's pose
			// If there is no collision, the return is the vehicle's new pose
			std::pair<bool, model::Pose> _checkCollisionWithinStep(int stepCount,VehicleState const& state, std::unordered_set<const model::Cone*> const& cones);

			PathNode _createNewNode(PathNode const& node, double steeringInput, int stage);
			void _processValidNode(PathNode & node, std::tuple<int, int, int,int> const& key, int stage);
			
			std::vector<model::Point> _getBoundary(VehicleState const& state) const;
			Point _rotatePoint(model::Point const& point, Angle const& angle) const;
			// Dubins
			double _getHeuristics(model::VehicleState const& start, model::Point const& goal);
			// Eucledian only
			double _getHeuristics(model::Point const& startPoint, model::Point const& goalPoint);
			// Multiple waypoints
			double _getHeuristics(model::VehicleState const& start, model::Point const& wayPoint, model::Point const& goalPoint);

			// gets position-orientation pairs until new position is reached
			std::vector<model::VehicleState> _stepUntilNew(VehicleState const& state, double distanceStep);

			bool _isAtGoal(const model::VehicleState& state, int index) const;
			bool _hasMoreNodes() const; 
			PQNode _popAndCloseNextNode();

			
			void _setDubins(Angle maxSteering, double wheelBase);

		};
	}
}
