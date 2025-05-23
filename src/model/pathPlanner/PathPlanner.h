#pragma once  
#include <vector>  
#include <src/model/Cone.h>  
#include <src/model/VehicleState.h>  
#include <iostream>  
#include <map>
#include <unordered_map>

#ifndef MAX_CONTAINER_SIZE
#define MAX_CONTAINER_SIZE 1000
#endif // !MAX_CONTAINER_SIZE

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

class PathPlanner  
{  
public:  
	PathPlanner(int iterations = 5, double deltaSpace = 1.5, double cellSize = 0.5,double steeringStep=M_PI/18);
	~PathPlanner() = default;  
	void planPath(const std::vector<model::Cone*>& cones, const model::VehicleState& vehicleState);  
	std::vector<model::Point> getPlannedPath() const;  
	void clear();

private:
	std::unordered_map<std::pair<int,int>, PathNode> _openList;
	std::unordered_map<std::pair<int,int>, PathNode> _closedList;
	std::unordered_map<std::pair<int, int>, PathNode> _collidingList;
	std::vector<model::Point> _plannedPath;  
	const int _iterations;  
	const double _deltaSpace;  
	const double _steeringStep;
	const double _cellSize;
	std::array<double, 5> _steeringAngles;

	void _setPlannedPath(PathNode& node);
	void _updateNeightbours(const std::vector<model::Cone*>& cones, PathNode& node); 
	model::VehicleState _stepByDistance(const VehicleState& state, double distance, double steeringAngle);
	std::pair<int,int> _discretizePoint(const model::Point& point) const;
	std::pair<bool,model::Point> _detectCollision(const VehicleState& state, const std::vector<model::Cone*>& cones) const;
	std::vector<model::Point> _getBoundary(const VehicleState& state) const;
	model::Point _rotatePoint(const model::Point& point, double angle) const;
	double _getHeuristics(const model::Point& point, const model::Point& goal) const;
	bool _detectGoal(const model::VehicleState& state, const model::Cone& cone) const;

};  
}
