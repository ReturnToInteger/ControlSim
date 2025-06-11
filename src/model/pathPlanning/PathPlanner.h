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
#define MAX_CONTAINER_SIZE 500
#endif // !MAX_CONTAINER_SIZE
namespace model {
	class Cone;
	namespace pathPlanning {
		enum class SteeringMode
		{
			LOW,
			MEDIUM,
			HIGH,
			EXTREME
		}; 

		struct SteeringPresets {
			static constexpr std::array<double, 5> steeringModeLOW = { 0, -0.5, 0.5, -1, 1 };
			static constexpr std::array<double, 7> steeringModeMEDIUM = { 0, -0.33, 0.33 - 0.67, 0.67, -1, 1 };
			static constexpr std::array<double, 9> steeringModeHIGH = { 0, -0.25, 0.25, -0.5, 0.5, -0.75, 0.75, -1, 1 };
			static constexpr std::array<double, 11> steeringModeEXTREME = { 0.0, 0.2,-0.2 , 0.4,-0.4 , 0.6,-0.6 , 0.8,-0.8 , 1.0, -1.0 };
		};

		// cellSize
		// stepSize
		// angleBins
		// SteeringMode: LOW, MEDIUM, HIGH, EXTREME
		struct PlannerConfig {
			double cellSize;
			double stepSize;
			int angleBins;
			SteeringMode steeringMode;
		};

		class PathPlanner
		{
		public:
			PathPlanner(PlannerConfig config);
			void planPath(std::vector<const model::Cone*> const& cones, model::VehicleState const& vehicleState);
			//Sets planned path based on current node
			void setPlannedPath();
			Path getPlannedPath() const;
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
#ifdef ENABLE_DEBUG_DRAW
			std::unordered_set<std::tuple<int, int, int>> _collidingList;
#endif // ENABLE_DEBUG_DRAW
			Path _plannedPath;
			const int _angleBins;
			const double _stepSize;
			const double _cellSize;
			const double* _steeringInputs;
			size_t _anglesSize;
			void _selectSteeringMode(SteeringMode const& mode, double const*& steeringInputs, size_t& size);

			Point _goal;
			PathNode _finalNode;
			DubinsStateSpace _dubins;

			void _updateNeightbours(std::vector<const model::Cone*> const& cones, PathNode const& node);
			VehicleState _stepByDistance(VehicleState const& state, double distance, double steeringAngle);
			std::tuple<int, int, int> _discretizePoint(model::Pose const& pose) const;
			// Returns if it collides with 100% accuracy, and vehicle pose
			// More computationally expensive
			std::pair<bool, model::Pose> _detectCollision(VehicleState const& state, std::vector<const model::Cone*> const& cones) const;

			// Simple collision detection that uses vehicle width
			// Less computationally expensive
			std::pair<bool, model::Pose> _lazyDetectCollision(VehicleState const& state, std::vector<const model::Cone*> const& cones) const;

			//		Detect collision between a node and one step away by interpolating
			// Returns the first collision if it happens, and the vehicle's pose
			// If there is no collision, the return is the vehicle's new pose
			std::pair<bool, model::Pose> _checkCollisionWithinStep(int stepCount,VehicleState const& state, std::vector<const model::Cone*> const& cones);

			PathNode _createNewNode(PathNode const& node, double steeringInput);
			void _processValidNode(PathNode & node, std::tuple<int, int, int> const& key);
			
			std::vector<model::Point> _getBoundary(VehicleState const& state) const;
			Point _rotatePoint(model::Point const& point, Angle const& angle) const;
			double _getHeuristics(model::VehicleState const& start, model::Point const& goal);
			// gets position-orientation pairs until new position is reached
			std::vector<model::VehicleState> _stepUntilNew(VehicleState const& state, double distanceStep);

			bool _isAtGoal(const model::VehicleState& state) const;
			bool _hasMoreNodes() const; 
			PQNode _getNextOpenNode();
			PathNode _extractNode(decltype(_openList)::iterator it);

			
			void _setDubins(double maxSteering, double wheelBase);

		};
	}
}
