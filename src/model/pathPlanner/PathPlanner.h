#pragma once  
#include <vector>  
#include <model/items/Cone.h>  
#include <model/VehicleState.h>  
#include "model/pathPlanner/PathNode.h"
#include <iostream>  
#include <map>
#include <unordered_map>

#ifndef MAX_CONTAINER_SIZE
#define MAX_CONTAINER_SIZE 5000
#endif // !MAX_CONTAINER_SIZE

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

private:
	// Storing nodes based on discrete pair of coordinates (x,y)
	std::unordered_map<std::pair<int,int>, PathNode> _openList;
	std::unordered_map<std::pair<int,int>, PathNode> _closedList;
	std::unordered_map<std::pair<int, int>, PathNode> _collidingList;
	std::vector<model::Pose> _plannedPath;
	std::vector<double> _plannedOrientation;
	const int _iterations;  
	const double _deltaSpace;  
	const double _steeringStep;
	const double _cellSize;
	std::array<double, 9> _steeringAngles;
	Point _goal;
	PathNode _currentNode;

	void _updateNeightbours(const std::vector<const model::Cone*>& cones, PathNode& node); 
	model::VehicleState _stepByDistance(const VehicleState& state, double distance, double steeringAngle);
	std::pair<int,int> _discretizePoint(const model::Point& point) const;
	std::pair<bool,model::Point> _detectCollision(const VehicleState& state, const std::vector<const model::Cone*>& cones) const;
	std::vector<model::Point> _getBoundary(const VehicleState& state) const;
	model::Point _rotatePoint(const model::Point& point, const Angle &angle) const;
	double _getHeuristics(const model::Point& point, const model::Point& goal) const;
	// gets position-orientation pairs until new position is reached
	std::vector<model::VehicleState> _stepUntilNew(VehicleState& state, double distanceStep);

};  
}
