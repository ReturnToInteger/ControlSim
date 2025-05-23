#include "PathPlanner.h"

model::PathPlanner::PathPlanner(int iterations, double deltaSpace,double cellSize, double steeringStep) : 
	_iterations(iterations), _deltaSpace(deltaSpace), _cellSize(cellSize), _steeringStep(steeringStep)
{
}

void model::PathPlanner::planPath(const std::vector<model::Cone*>& cones, const model::VehicleState& vehicleState)
{
	double maxSteeringAngle = vehicleState.getMaxSteeringAngle();
	
	_steeringAngles = { 0.0,maxSteeringAngle / 2.0,-maxSteeringAngle / 2 ,maxSteeringAngle,-maxSteeringAngle };
	_plannedPath.reserve(_iterations);
	
	double x = vehicleState.getPosition().X();
	double y = vehicleState.getPosition().Y();
	double orientation = vehicleState.getOrientation();
	Point goal(40.0, 0.0);
	PathNode startNode(vehicleState, 0, _getHeuristics(vehicleState.getPosition(),goal), nullptr);
	auto [isColliding, collisionPoint] = _detectCollision(startNode.state, cones);
	// Check collision for start node
	if (isColliding) {
		_collidingList.emplace(_discretizePoint(startNode.state.getPosition()), startNode);
		return;
	}
	_openList.emplace(_discretizePoint(startNode.state.getPosition()), startNode);
	while (!_openList.empty()&&_openList.size()< MAX_CONTAINER_SIZE && _closedList.size()< MAX_CONTAINER_SIZE) {

		// Find the node with the lowest cost in the open list
		auto minNodeIt = std::min_element(_openList.begin(), _openList.end(),
			[](const auto& a, const auto& b) { return a.second < b.second; });
		PathNode currentNode = minNodeIt->second;

		_closedList.emplace(minNodeIt->first, minNodeIt->second); // Add the node to the closed list
		_openList.erase(minNodeIt); // Remove the node from the open list
		// or (currentNode.cost > _iterations)
		if ((currentNode.state.getPosition()-goal).magnitude()<_deltaSpace) {
			_setPlannedPath(currentNode);
			return;
		}
		else {
			_updateNeightbours(cones, currentNode);
		}


	}
	_plannedPath.clear();
}

std::vector<model::Point> model::PathPlanner::getPlannedPath() const
{
	return _plannedPath;
}

void model::PathPlanner::_setPlannedPath(PathNode& node)
{
	_plannedPath.clear();
	PathNode* currentNode = &node;
	_plannedPath.emplace_back(currentNode->state.getPosition());
	currentNode = currentNode->parent.get();
	while (currentNode != nullptr && currentNode->parent.get() !=currentNode) {
		_plannedPath.emplace_back(currentNode->state.getPosition());
		currentNode = currentNode->parent.get();
	}
	std::reverse(_plannedPath.begin(), _plannedPath.end());
}

void model::PathPlanner::_updateNeightbours(const std::vector<model::Cone*>& cones, PathNode& node)
{
	double maxSteeringAngle = node.state.getMaxSteeringAngle();
	Point goal(40.0, 0.0);

	for (auto& steeringAngle: _steeringAngles) {
		//std::cout << "steering angle: " << steeringAngle << std::endl;
		PathNode newNode = node;
		newNode.state = _stepByDistance(node.state, _deltaSpace, steeringAngle);
		newNode.parent = std::make_shared<PathNode>(node);
		newNode.cost = node.cost + _deltaSpace;
		newNode.heuristic = newNode.cost + _getHeuristics(node.state.getPosition(),goal);

		// Check if the new node is in the closed list
		std::pair<int, int> newNodeKey = _discretizePoint(newNode.state.getPosition());
		if (_closedList.find(newNodeKey) == _closedList.end()) {
				// Collision check
			auto collisionResult = _detectCollision(newNode.state, cones);
			bool isColliding(collisionResult.first);
			model::Point contactPoint(collisionResult.second);
			// if collision add to closed list
			if (isColliding) {
				std::pair<int, int> collidingKey = _discretizePoint(contactPoint);
				_closedList.emplace(newNodeKey,newNode);
			}
			else {
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

std::pair<bool, model::Point> model::PathPlanner::_detectCollision(const VehicleState& state, const std::vector<model::Cone*>& cones) const
{
	model::Point vehiclePosition(state.getPosition());
	double vehicleOrientation = state.getOrientation();
	for (const auto& cone : cones) {
		if (cone->getType() == model::ConeType::UNKNOWN) {
			continue; // Skip unknown cones
		}

		Point relativePosition = _rotatePoint(cone->getPosition() - vehiclePosition, -vehicleOrientation);
		double clampedX = model::clamp(relativePosition.X(), -state.getLength() / 2, state.getLength() / 2);
		double clampedY = model::clamp(relativePosition.Y(), -state.getWidth() / 2, state.getWidth() / 2);
		model::Point clampedPoint(clampedX, clampedY);
		double distance = (relativePosition - clampedPoint).magnitude();
		if (distance < cone->getRadius()) {
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

model::Point model::PathPlanner::_rotatePoint(const model::Point& point, double angle) const
{
	return model::Point(point.X() * cos(angle) - point.Y() * sin(angle), point.X() * sin(angle) + point.Y() * cos(angle));
}

double model::PathPlanner::_getHeuristics(const model::Point& point, const model::Point& goal=model::Point(10.0,10.0)) const
{
	return (point - goal).magnitude();
}

void model::PathPlanner::clear()
{
	_openList.clear();
	_closedList.clear();
	_plannedPath.clear();
	_openList.reserve(MAX_CONTAINER_SIZE);
	_closedList.reserve(MAX_CONTAINER_SIZE);
	//std::cout << "Cleared PathPlanner data" << std::endl;
}

//std::tuple<std::vector<model::Point>, std::vector<model::Point>, std::vector<model::Point>> model::PathPlanner::getCones(const std::vector<model::Cone*>& cones) const
//{
//	std::vector<model::Point> leftCones;
//	std::vector<model::Point> rightCones;
//	std::vector<model::Point> unknownCones;
//	for (const auto& cone : cones) {
//		switch (cone->getType()) {
//		case model::ConeType::LEFT:
//			leftCones.push_back(cone->getPosition());
//			break;
//		case model::ConeType::RIGHT:
//			rightCones.push_back(cone->getPosition());
//			break;
//		case model::ConeType::UNKNOWN:
//			unknownCones.push_back(cone->getPosition());
//			break;
//		default:
//			throw std::invalid_argument("Invalid ConeType in getCones");
//		}
//	}
//	return { leftCones,rightCones,unknownCones };
//}
