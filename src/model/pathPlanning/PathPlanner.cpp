#include "PathPlanner.h"
#include "model/items/Cone.h"
#include "model/utils/Angle.h"
#include "model/utils/Pose.h"
#include <thread>
#include <algorithm>
#include <ranges>
#include <stdexcept>

#ifdef ENABLE_DEBUG_DRAW
#include "view/DebugDraw.h"
#endif // ENABLE_DEBUG_DRAW


namespace model::pathPlanning {
	PathPlanner::PathPlanner(PlannerConfig config) :
		_stepSize(config.stepSize), 
		_cellSize(config.cellSize), 
		_angleBins(config.angleBins), 
		_goals(static_cast<int>(config.waypoints)+1), 
		_maxContainerSize(config.maxContainerSize),
		_dubins(0.0),
		_steeringInputs{},
		_anglesSize(0)
	{
		_goals.push(Point(0.0, 0.0));
		_selectSteeringMode(config.steeringMode, _steeringInputs, _anglesSize);
	}

	bool PathPlanner::planPath(std::unordered_set<const model::Cone*> const& cones, model::VehicleState const& vehicleState)
	{
		_setDubins(vehicleState.getMaxSteeringAngle(),vehicleState.getWheelBase());

		// If start is within goal, return
		if (_isAtGoal(vehicleState, 1)) {
			std::cout << "Reached Goal\n";
			return true;
		}
			

		// Process first node
		PQNode startPQ {
			.fCost = _getHeuristics(vehicleState, _goals.front()),
			.key = _discretizePoint(vehicleState.getPose(),0) 
		};
		PathNode startNode {
			.state = vehicleState,
			.gCost = 0,
			.fCost = startPQ.fCost,
			.parent = nullptr,
			.stage = 0 
		};

		// Check collision for start node
		auto [isColliding, collisionPoint] = _detectCollision(vehicleState, cones);
		if (!isColliding) {
			// Add startNode to open list
			_openList.emplace(startPQ.key, startNode);
			_openQueue.emplace(startPQ);

		}
		else {
			#ifdef ENABLE_DEBUG_DRAW
			_collidingList.emplace(startPQ.key);
			#endif // ENABLE_DEBUG_DRAW
			std::cout << "Inside a cone.\n";
			return false;
		}

		int iter = 0;
		bool reachedFirst = false;
		while (_hasMoreNodes()) {
			// Find the node with the lowest cost in the open list, mark it visited, and explore it

			// Get smallest cost and pop
			PQNode top = _popAndCloseNextNode();

			auto it = _openList.find(top.key);
			if (it == _openList.end()|| top.fCost > it->second.fCost + 1e-8)
			{
				continue; // stale node
			}

			// Set the current node
			PathNode currentNode = std::move(it->second);
			_openList.erase(it);
			//if (reachedFirst && currentNode.stage == 0) continue;
			if (currentNode.stage == 0) {
				// Goal Reached
				if (_isAtGoal(currentNode.state, 0)) {
					_finalNode = currentNode;
					//return;
					reachedFirst = true;
					_updateNeightbours(cones, currentNode, 1);
				}
				else {
					_updateNeightbours(cones, currentNode, 0);
				}

			}
			else {
				// Goal Reached
				if (_isAtGoal(currentNode.state, 1)) {
					_finalNode = currentNode;
					return true;
				}
				_updateNeightbours(cones, currentNode, 1);
			}
			iter++;
		}
		//std::cout << "Path not found after " << iter << " iterations.\n";
		// if it reaches the first goal, we still return a valid path
		if (!reachedFirst)
		{
			_finalNode = startNode;
		}
		return false;
	}

	PQNode model::pathPlanning::PathPlanner::_popAndCloseNextNode() {
		PQNode top = _openQueue.top();
		_openQueue.pop();
		_closedList.emplace(top.key);
		return top;
	}


	Path PathPlanner::getPlannedPath() const
	{
		return _plannedPath;
	}

	void PathPlanner::setGoal(Point const& goal)
	{
		_goals.push(goal);
		//std::cout << "Goals: ";
		//std::cout << _goals.front() << "\n";
		//std::cout << _goals.back()<<"\n";

	}



	void PathPlanner::setPlannedPath()
	{
#ifdef ENABLE_DEBUG_DRAW
		std::vector<Point> drawVect;
		drawVect.reserve(_openList.size());
		for (auto& node : _openList) {
			drawVect.emplace_back(node.second.state.getPosition());
		}
		view::DebugDraw::instance().points(drawVect);
		drawVect.clear();
		drawVect.reserve(_closedList.size());
		for (const auto& node : _closedList) {
			drawVect.emplace_back(std::get<0>(node) * _cellSize, std::get<1>(node) * _cellSize);
		}
		view::DebugDraw::instance().points2(drawVect, sf::Color::Red);
		drawVect.clear();
		drawVect.reserve(_collidingList.size());
		for (const auto& node : _collidingList) {
			drawVect.emplace_back(std::get<0>(node) * _cellSize, std::get<1>(node) * _cellSize);
		}
		view::DebugDraw::instance().points3(drawVect, sf::Color::White);
#endif // ENABLE_DEBUG_DRAW

		std::vector<VehicleState> path;
		const PathNode* currentNode = &_finalNode;
		path.emplace_back(currentNode->state);
		currentNode = currentNode->parent.get();
		while (currentNode != nullptr && currentNode->parent.get() != currentNode) {
			path.emplace_back(currentNode->state);
			currentNode = currentNode->parent.get();
		}
		std::ranges::reverse(path);
		_plannedPath=Path(path);
	}

	void PathPlanner::_selectSteeringMode(SteeringMode const& mode, double const*& inputs, size_t & size)
	{
		switch (mode) {
		case SteeringMode::LOW:
			inputs = SteeringPresets::steeringModeLOW.data();
			size = SteeringPresets::steeringModeLOW.size();
			break;
		case SteeringMode::MEDIUM:
			inputs = SteeringPresets::steeringModeMEDIUM.data();
			size = SteeringPresets::steeringModeMEDIUM.size();
			break;
		case SteeringMode::HIGH:
			inputs = SteeringPresets::steeringModeHIGH.data();
			size = SteeringPresets::steeringModeHIGH.size();
			break;
		case SteeringMode::SQUARED:
			inputs = SteeringPresets::steeringModeSQUARED.data();
			size = SteeringPresets::steeringModeSQUARED.size();
			break;
		default:
			throw std::out_of_range("Invalid planner config (SteeringMode).");
		}
	}

	void model::pathPlanning::PathPlanner::_updateNeightbours(std::unordered_set<const model::Cone*> const& cones, PathNode const& node, int stage)
	{
		//std::cout << "Start of _uNb\n";
		auto sharedParent = std::make_shared<PathNode>(node);

		for (int i = 0; i < _anglesSize;i++) {
			// Set leaf partly
			PathNode newNode;
			newNode.state = _stepByDistance(node.state, _stepSize, _steeringInputs[i]);
			newNode.stage = stage;

			std::tuple<int, int, int,int> newNodeKey = _discretizePoint(newNode.state.getPose(),stage);

			// Check if the new node is in the closed list
			if (_closedList.find(newNodeKey) == _closedList.end()) {
				//	Interpolate the path between the current node and the new node and check each collision
				auto [isColliding, contactPose] = _checkCollisionWithinStep(3, node.state, cones);
				// if collision add to closed list
				if (isColliding) {
					std::tuple<int, int, int,int> collidingKey = _discretizePoint(contactPose,stage);
					_closedList.emplace(collidingKey);
				}
				else {
					newNode.parent = sharedParent;
					newNode.gCost = node.gCost + _stepSize;

					_processValidNode(newNode, newNodeKey,stage);
				}
			}
		}
		//std::cout << "End of _uNb\n";
	}

	model::VehicleState PathPlanner::_stepByDistance(VehicleState const& state, double distance, double steeringInput)
	{
		VehicleState copy = state;
		double traveled = 0;
		double dt = 0.01; // small fixed step
		copy.setTargetSpeed(copy.getMaxSpeed());
		copy.setTargetSteeringAngle(steeringInput);

		while (traveled < distance) {
			copy.updateState(dt);
			traveled += copy.getSpeed() * dt;
		}

		return copy;
	}

	std::tuple<int, int, int,int> PathPlanner::_discretizePoint(model::Pose const& pose,int stage) const
	{
		int x = static_cast<int>(pose.x / _cellSize);
		int y = static_cast<int>(pose.y / _cellSize);
		double shiftedTheta = radian(pose.theta) + std::numbers::pi;  // now in [0, 2π)
		double binWidth = 2 * std::numbers::pi / _angleBins;
		int angle = static_cast<int>(std::floor(shiftedTheta / binWidth));
		return std::make_tuple(x, y, angle,stage);
	}


	std::pair<bool, model::Pose> PathPlanner::_detectCollision(VehicleState const& state, std::unordered_set<const model::Cone*> const& cones) const
	{
		if (cones.empty()) 
		{
			return { false, state.getPose() };
		}
		model::Point vehiclePosition = state.getPosition();
		Angle vehicleOrientation = state.getOrientation();
		double radius = (*cones.begin())->getRadius();
		double vhclLength = state.getLength();
		double vhclWidth = state.getWidth();
		for (auto const& cone : cones) {
			Point relPos(cone->getPosition() - vehiclePosition);
			if (relPos.magnitude() > (vhclLength * 1.5 + radius)) 
			{
				continue; // If the cone is more than sqrt(2)*length away, skip it
			}
			relPos = rotatePoint(relPos, -vehicleOrientation);
			model::Point clampedPoint(model::clamp(relPos.X(), -vhclLength / 2, vhclLength / 2),
				model::clamp(relPos.Y(), -vhclWidth / 2, vhclWidth / 2));
			double distance = (relPos - clampedPoint).magnitude();
			// within radius and margin for error
			if (distance < radius) 
			{
				//Point contactPoint = _rotatePoint(clampedPoint, vehicleOrientation) + vehiclePosition;
				return { true, state.getPose() };
			}
		}
		return { false,state.getPose() }; // No collision
	}

	std::pair<bool, model::Pose> PathPlanner::_lazyDetectCollision(VehicleState const& state, std::unordered_set<const model::Cone*> const& cones) const
	{
		if (cones.empty()) 
		{
			return { false, state.getPose() };
		}
		model::Point vehiclePosition(state.getPosition());
		double radius = (*cones.begin())->getRadius();
		double vhclLength = state.getLength();
		double vhclWidth = state.getWidth();
		for (auto const& cone : cones) {
			Point relPos(cone->getPosition() - vehiclePosition);
			if (relPos.magnitude() > (vhclLength * 1.5 + radius)) 
			{
				continue; // If the cone is more than sqrt(2)*length away, skip it
			}
			// within radius and margin for error
			if (relPos.magnitude() < radius + vhclWidth / 2.0) 
			{
				return { true, Pose(state.getPose()) };
			}
		}
		return { false,state.getPose() }; // No collision
	}

	std::pair<bool, model::Pose> PathPlanner::_checkCollisionWithinStep(int stepCount, VehicleState const& state, std::unordered_set<const model::Cone*> const& cones)
	{
		if (cones.empty()) 
		{
			return { false, Pose() };
		}
		double minAllowedStep = _cellSize * 1.5; // diagonal step size
		double smallStepSize = clamp(_stepSize / stepCount, minAllowedStep, _stepSize);
		bool isColliding=false;
		Pose contactPose;
		double currentStep = smallStepSize;
		while (currentStep < _stepSize - 1e-8) {
			auto collisionResult = _detectCollision(_stepByDistance(state, currentStep, state.getTarget().normSteering), cones);
			if (collisionResult.first) 
			{
				isColliding = true;
				contactPose = collisionResult.second;
				break;
			}
			currentStep += smallStepSize;
		}
		if (!isColliding) 
		{
			auto collisionResult = _detectCollision(_stepByDistance(state, _stepSize, state.getTarget().normSteering), cones);
			isColliding = collisionResult.first;
			contactPose = collisionResult.second;
		}

		return { isColliding,contactPose };
	}

	PathNode model::pathPlanning::PathPlanner::_createNewNode(PathNode const& node, double steeringInput, int stage) const
	{
		PathNode newNode;
		newNode.state = _stepByDistance(node.state, _stepSize, steeringInput);
		newNode.stage = stage;
		return newNode;
	}

	void PathPlanner::_processValidNode(PathNode & node, std::tuple<int, int, int,int> const& key,int stage)
	{
		double heuristics;
		if (stage == 0) 
		{
			heuristics = _getHeuristics(node.state, _goals.front(), _goals.back());
		} else
		{
			heuristics = _getHeuristics(node.state, _goals.back());
		}
		double fCost = node.gCost + heuristics;
		node.fCost = fCost;
		// if not in open list, add it
		auto [it, isInserted] = _openList.emplace(key, std::move(node));

		if (isInserted) 
		{
			_openQueue.emplace(fCost, key);
		}
		else if (node.gCost < _openList[key].gCost) 
		{
			// if in open list, replace if cost is lower
			_openList[key] = std::move(node);
			_openQueue.emplace(fCost, key);
		}

	}

	std::vector<model::Point> PathPlanner::_getBoundary(VehicleState const& state) const
	{
		throw std::runtime_error("getBoundary not implemented");
	}

	model::Point PathPlanner::rotatePoint(model::Point const& point, Angle const& angle)
	{
		return { point.X() * model::cos(angle) - point.Y() * model::sin(angle), point.X() * model::sin(angle) + point.Y() * model::cos(angle) };
	}

	double PathPlanner::_getHeuristics(model::VehicleState const& start, model::Point const& goalPoint)
	{
		return _dubins.simpleDistance(start.getPose(), goalPoint)
			+ (start.getSteeringAngle() / start.getMaxSteeringAngle()) * (start.getSteeringAngle() / start.getMaxSteeringAngle());
	}

	double PathPlanner::_getHeuristics(model::Point const& startPoint, model::Point const& goalPoint)
	{
		return (startPoint - goalPoint).magnitude();

	}

	double PathPlanner::_getHeuristics(model::VehicleState const& start, model::Point const& wayPoint, model::Point const& goalPoint)
	{
		return _dubins.multipleDistance(start.getPose(), wayPoint, goalPoint)
			+ (start.getSteeringAngle() / start.getMaxSteeringAngle()) * (start.getSteeringAngle() / start.getMaxSteeringAngle());
	}

	std::vector<model::VehicleState> PathPlanner::_stepUntilNew(VehicleState const& state, double distanceStep) const
	{
		double diagonalStepSize = _cellSize * 1.5;
		distanceStep = std::min(std::max(distanceStep, diagonalStepSize), _stepSize);

		std::vector<VehicleState> states;
		double currentStep = distanceStep;
		while (currentStep < _stepSize - 1e-8) {
			//VehicleState currentState = _stepByDistance(state, currentStep, state.getSteeringAngle());
			states.push_back(_stepByDistance(state, currentStep, state.getTarget().normSteering));
			currentStep += distanceStep;
		}
		return states;
	}

	bool model::pathPlanning::PathPlanner::_isAtGoal(const model::VehicleState& state, int index) const
	{
		if (index == 0) {
			return (state.getPosition() - _goals.front()).magnitude() < _stepSize;

		}

		Point relPos = state.getPosition() - _goals.back();
		Angle theta = state.getOrientation();
		double xDist = relPos.X() * cos(theta) + relPos.Y() * sin(theta);
		double yDist = - relPos.X() * sin(theta) + relPos.Y() * cos(theta);
		return abs(yDist) < _stepSize && abs(xDist) < _stepSize;


	}

	bool PathPlanner::_hasMoreNodes() const {
		return !_openList.empty() &&
			_openList.size() < _maxContainerSize &&
			_closedList.size() < _maxContainerSize;
	}


	void PathPlanner::_setDubins(Angle maxSteering, double wheelBase)
	{
		Angle beta = atan(tan(radian(maxSteering)) / 2);
		double radius = wheelBase / (tan(maxSteering) * cos(beta));
		_dubins = DubinsStateSpace(radius);
	}

	void PathPlanner::clear()
	{
		//Point lastGoal= _goals.back();
		//_goals.clear();
		//_goals.emplace_back(lastGoal);
		_openList.clear();
		std::vector<PQNode> PQTemp;
		PQTemp.reserve(_maxContainerSize);
		_openQueue = std::priority_queue<PQNode,
			std::vector<PQNode>,
			std::greater<>>(std::greater<>(), std::move(PQTemp));
		_closedList.clear();
		_openList.reserve(_maxContainerSize);
		_closedList.reserve(_maxContainerSize);
		//std::cout << "Cleared PathPlanner data\n";
	}
}