#include "PathPlanner.h"

model::PathPlanner::PathPlanner(int iterations, double deltaSpace,double cellSize, double steeringStep) : 
	_iterations(iterations), _deltaSpace(deltaSpace), _cellSize(cellSize), _steeringStep(steeringStep), _goal(-47.5, 10.0)
{
}

void model::PathPlanner::planPath(const std::vector<const model::Cone*>& cones, const model::VehicleState & vehicleState)
{
	double maxSteeringAngle = vehicleState.getMaxSteeringAngle();
	
	_steeringAngles = { 0.0,
		maxSteeringAngle *0.2,-maxSteeringAngle *0.2 ,
		maxSteeringAngle *0.4,-maxSteeringAngle *0.4 ,
		maxSteeringAngle *0.75,-maxSteeringAngle *0.75 ,
		maxSteeringAngle,-maxSteeringAngle };
	
	PathNode startNode(vehicleState, 0, _getHeuristics(vehicleState.getPosition(),_goal), nullptr);
	if (startNode.heuristic < _deltaSpace / 2.0) {
		std::cout << "Reached Goal" << std::endl;
		return;
	}
	auto [isColliding, collisionPoint] = _detectCollision(startNode.state, cones);
	// Check collision for start node
	if (isColliding) {
		_collidingList.emplace(_discretizePoint(startNode.state.getPosition()), startNode);
		return;
	}
	_openList.emplace(_discretizePoint(startNode.state.getPosition()), startNode);
	//while (!_openList.empty()) {
	while (!_openList.empty() && _openList.size() < MAX_CONTAINER_SIZE && _closedList.size() < MAX_CONTAINER_SIZE) {
		// Find the node with the lowest cost in the open list
		auto minNodeIt = std::min_element(_openList.begin(), _openList.end(),
			[](const auto& a, const auto& b) { return a.second < b.second; });
		PathNode currentNode = minNodeIt->second;

		_closedList.emplace(minNodeIt->first, minNodeIt->second); // Add the node to the closed list
		_openList.erase(minNodeIt); // Remove the node from the open list
		// or (currentNode.cost > _iterations)
		if (_getHeuristics(currentNode.state.getPosition(),_goal)<_deltaSpace/2.0) {
			_currentNode=currentNode;
			return;
		}
		else {
			_updateNeightbours(cones, currentNode);
		}


	}
	_currentNode = startNode;
}

std::vector<model::Pose> model::PathPlanner::getPlannedPath() const
{
	return _plannedPath;
}

void model::PathPlanner::setGoal(const Point& goal)
{
	_goal = goal;
}



void model::PathPlanner::setPlannedPath()
{
	_plannedPath.clear();
	_plannedPath.reserve(_iterations);
	PathNode* currentNode = &_currentNode;
	_plannedPath.emplace_back(currentNode->state.getPose());
	currentNode = currentNode->parent.get();
	while (currentNode != nullptr && currentNode->parent.get() !=currentNode) {
		_plannedPath.emplace_back(currentNode->state.getPose());
		currentNode = currentNode->parent.get();
	}
	std::reverse(_plannedPath.begin(), _plannedPath.end());
}

void model::PathPlanner::_updateNeightbours(const std::vector<const model::Cone*>& cones, PathNode& node)
{
	double maxSteeringAngle = node.state.getMaxSteeringAngle();

	for (auto& steeringAngle: _steeringAngles) {
		//std::cout << "steering angle: " << steeringAngle << std::endl;
		PathNode newNode = node;
		newNode.state = _stepByDistance(node.state, _deltaSpace, steeringAngle);
		newNode.parent = std::make_shared<PathNode>(node);

		// Check if the new node is in the closed list
		std::pair<int, int> newNodeKey = _discretizePoint(newNode.state.getPosition());
		if (_closedList.find(newNodeKey) == _closedList.end()) {
			// Collision check
			bool isColliding = false;
			model::Point contactPoint;
			//		Interpolate the path between the current node and the new node
			std::vector<VehicleState> interpolatedState = _stepUntilNew(node.state, _cellSize * 2);
			for (auto& state : interpolatedState) {
				auto collisionResult = _detectCollision(state, cones);
				if (collisionResult.first) {
					isColliding = true;
					contactPoint = collisionResult.second;
					break;
				}
			}
			if (!isColliding) {
				auto collisionResult = _detectCollision(newNode.state, cones);
				isColliding=collisionResult.first;
				contactPoint = collisionResult.second;
			}
			// if collision add to closed list
			if (isColliding) {
				std::pair<int, int> collidingKey = _discretizePoint(contactPoint);
				_closedList.emplace(newNodeKey,newNode);
			}
			else {
				//calculate cost and heuristics
				newNode.cost = node.cost + _deltaSpace;
				newNode.heuristic = newNode.cost + _getHeuristics(node.state.getPosition(),_goal);
				// if not in open list, add it
				auto [it,isInserted]=_openList.emplace(newNodeKey, newNode);

				// if in open list, replace if cost is lower
				if (!isInserted) {
					if (newNode.cost < _openList[newNodeKey].cost) {
						_openList[newNodeKey]=newNode;
					}
				}

			}
		}




	}
}

model::VehicleState model::PathPlanner::_stepByDistance(const VehicleState& state, double distance, double steeringAngle)
{
	VehicleState copy = state;
	double deltaTime;
	deltaTime = distance / state.getMaxSpeed();
	//if (deltaTime*state.getSpeed()<distance/2)
	//	deltaTime *=2;

	copy.setTargetSpeed(copy.getMaxSpeed());
	copy.setTargetSteeringAngle(steeringAngle);
	copy.updateState(deltaTime);
	return copy;


}

std::pair<int, int> model::PathPlanner::_discretizePoint(const model::Point& point) const
{
	int x = static_cast<int>(point.X() / _cellSize);
	int y = static_cast<int>(point.Y() / _cellSize);
	return std::make_pair(x, y);
}

std::pair<bool, model::Point> model::PathPlanner::_detectCollision(const VehicleState& state, const std::vector<const model::Cone*>& cones) const
{
	model::Point vehiclePosition(state.getPosition());
	Angle vehicleOrientation = state.getOrientation();
	if (cones.empty()) return { false, Point() };
	double radius = cones[0]->getRadius();
	double vhclLength = state.getLength(), vhclWidth = state.getWidth();
	for (const auto& cone : cones) {
		if (cone->getType() == model::ConeType::UNKNOWN) {
			continue; // Skip unknown cones
		}
		Point relPos(cone->getPosition() - vehiclePosition);
		if (relPos.magnitude() > (vhclLength * 1.5 + radius)) continue;
		relPos = _rotatePoint(relPos, -vehicleOrientation);
		model::Point clampedPoint(model::clamp(relPos.X(), -vhclLength / 2, vhclLength / 2), 
			model::clamp(relPos.Y(), -vhclWidth / 2, vhclWidth / 2));
		double distance = (relPos - clampedPoint).magnitude();
		if (distance < radius) {
			Point contactPoint = _rotatePoint(clampedPoint, vehicleOrientation) + vehiclePosition;
			return { true, contactPoint };
		}
	}
	return { false,Point() }; // No collision
}

std::vector<model::Point> model::PathPlanner::_getBoundary(const VehicleState& state) const
{
	throw std::runtime_error("getBoundary not implemented");
}

model::Point model::PathPlanner::_rotatePoint(const model::Point& point, const Angle& angle) const
{
	return model::Point(point.X() * model::cos(angle) - point.Y() * model::sin(angle), point.X() * model::sin(angle) + point.Y() * model::cos(angle));
}

double model::PathPlanner::_getHeuristics(const model::Point& point, const model::Point& goal=model::Point(10.0,10.0)) const
{
	return (point - goal).magnitude();
}

std::vector<model::VehicleState> model::PathPlanner::_stepUntilNew(VehicleState& state, double distanceStep)
{
	distanceStep = std::min(std::max(distanceStep, _cellSize * 1.5), _deltaSpace);

	std::vector<VehicleState> states;
	double currentStep = distanceStep;
	while (currentStep < _deltaSpace-1e-8) {
		//VehicleState currentState = _stepByDistance(state, currentStep, state.getSteeringAngle());
		states.push_back(_stepByDistance(state, currentStep, state.getSteeringAngle()));
		currentStep += distanceStep;
	}
	return states;
}

void model::PathPlanner::clear()
{
	_openList.clear();
	_closedList.clear();
	_openList.reserve(MAX_CONTAINER_SIZE);
	_closedList.reserve(MAX_CONTAINER_SIZE);
	//std::cout << "Cleared PathPlanner data" << std::endl;
}

