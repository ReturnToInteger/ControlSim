#pragma once  
#include <vector>  
#include <model/items/Cone.h>  
#include <model/VehicleState.h>  
#include "model/pathPlanner/PathNode.h"
#include <iostream>  
#include <map>
#include <unordered_map>

#ifndef MAX_CONTAINER_SIZE
#define MAX_CONTAINER_SIZE 10000
#endif // !MAX_CONTAINER_SIZE
#include <unordered_set>
#include "model/utils/Heuristics/DubinsSpace.h"
#include <queue>
namespace model {  
class PathPlanner  
{  
public:  
	PathPlanner(int iterations = 5, double deltaSpace = 1.5, double cellSize = 0.5,double steeringStep=M_PI/18);
	~PathPlanner() = default;  
	void planPath(const std::vector<const model::Cone*>& cones, const model::VehicleState & vehicleState);
	//Sets planned path based on current node
	void setPlannedPath();
	std::vector<model::Pose> getPlannedPath() const;
	void setGoal(const Point &goal);
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
	std::vector<model::Pose> _plannedPath;
	std::vector<double> _plannedOrientation;
	const int _iterations;  
	const double _deltaSpace;  
	const double _steeringStep;
	const double _cellSize;
	std::array<double, 9> _steeringAngles;
	Point _goal;
	PathNode _currentNode;
	model::DubinsStateSpace _dubins;

	void _updateNeightbours(const std::vector<const model::Cone*>& cones, PathNode& node); 
	model::VehicleState _stepByDistance(const VehicleState& state, double distance, double steeringAngle);
	std::tuple<int,int,int> _discretizePoint(const model::Pose & pose) const;
	std::pair<bool, model::Pose> _detectCollision(const VehicleState& state, const std::vector<const model::Cone*>& cones) const;
	std::pair<bool, model::Pose> _lazyDetectCollision(const VehicleState& state, const std::vector<const model::Cone*>& cones) const;
	std::vector<model::Point> _getBoundary(const VehicleState& state) const;
	model::Point _rotatePoint(const model::Point& point, const Angle &angle) const;
	double _getHeuristics(const model::VehicleState& start, const model::Point& goal);
	// gets position-orientation pairs until new position is reached
	std::vector<model::VehicleState> _stepUntilNew(VehicleState& state, double distanceStep);

};  
}
