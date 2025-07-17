#include "PathPlanner.h"
#include "model/items/Cone.h"
#include "model/utils/Angle.h"
#include "model/utils/Pose.h"
#include <thread>
#include <algorithm>
#include <ranges>
#include <stdexcept>
#include "model/utils/ModelUtils.h"

#ifdef ENABLE_DEBUG_DRAW
#include "view/DebugDraw.h"
#endif // ENABLE_DEBUG_DRAW


namespace model::pathPlanning {
	PathPlanner::PathPlanner(PlannerConfig config) :
		m_stepSize(config.stepSize), 
		m_cellSize(config.cellSize), 
		m_angleBins(config.angleBins), 
		m_goals(static_cast<int>(config.waypoints)+1), 
		m_maxContainerSize(config.maxContainerSize),
		m_dubins(0.0),
		m_steeringInputs{},
		m_anglesSize(0)
	{
		m_goals.push(Point(0.0, 0.0));
		selectSteeringMode(config.steeringMode, m_steeringInputs, m_anglesSize);
	}

	bool PathPlanner::planPath(std::unordered_set<const model::Cone*> const& cones, model::IVehicleState const& vehicleState)
	{
		setDubins(vehicleState.minimumTurningRadius());

		// If start is within goal, return
		if (isAtGoal(vehicleState, 1)) {
			std::cout << "Reached Goal\n";
			return true;
		}
			

		// Process first node
		PQNode startPQ {
			.fCost = getHeuristics(vehicleState, m_goals.front()),
			.key = discretizePoint(vehicleState.getPose(),0) 
		};
		PathNode startNode;
		startNode.state = vehicleState.clone();
		startNode.gCost = 0;
		startNode.fCost = startPQ.fCost;
		startNode.parent = nullptr;
		startNode.stage = 0;

		// Check collision for start node
		auto [isColliding, collisionPoint] = detectCollision(vehicleState, cones);
		if (!isColliding) {
			// Add startNode to open list
			m_openList.emplace(startPQ.key, startNode);
			m_openQueue.emplace(startPQ);

		}
		else {
			#ifdef ENABLE_DEBUG_DRAW
			m_collidingList.emplace(startPQ.key);
			#endif // ENABLE_DEBUG_DRAW
			std::cout << "Inside a cone.\n";
			return false;
		}

		int iter = 0;
		bool reachedFirst = false;
		while (hasMoreNodes()) {
			// Find the node with the lowest cost in the open list, mark it visited, and explore it

			// Get smallest cost and pop
			PQNode top = popAndCloseNextNode();

			auto it = m_openList.find(top.key);
			if (it == m_openList.end()|| top.fCost > it->second.fCost + 1e-8)
			{
				continue; // stale node
			}

			// Set the current node
			PathNode currentNode = std::move(it->second);
			m_openList.erase(it);
			//if (reachedFirst && currentNode.stage == 0) continue;
			if (currentNode.stage == 0) {
				// Goal Reached
				if (isAtGoal(*currentNode.state, 0)) {
					m_finalNode = currentNode;
					//return;
					reachedFirst = true;
					updateNeightbours(cones, currentNode, 1);
				}
				else {
					updateNeightbours(cones, currentNode, 0);
				}

			}
			else {
				// Goal Reached
				if (isAtGoal(*currentNode.state, 1)) {
					m_finalNode = currentNode;
					return true;
				}
				updateNeightbours(cones, currentNode, 1);
			}
			iter++;
		}
		//std::cout << "Path not found after " << iter << " iterations.\n";
		// if it reaches the first goal, we still return a valid path
		if (!reachedFirst)
		{
			m_finalNode = startNode;
		}
		return false;
	}

	PQNode model::pathPlanning::PathPlanner::popAndCloseNextNode() {
		PQNode top = m_openQueue.top();
		m_openQueue.pop();
		m_closedList.emplace(top.key);
		return top;
	}


	const Path& PathPlanner::getPlannedPath() const
	{
		return m_plannedPath;
	}

	void PathPlanner::setGoal(Point const& goal)
	{
		m_goals.push(goal);
		//std::cout << "Goals: ";
		//std::cout << m_goals.front() << "\n";
		//std::cout << m_goals.back()<<"\n";

	}



	void PathPlanner::setPlannedPath()
	{
#ifdef ENABLE_DEBUG_DRAW
		std::vector<Point> drawVect;
		drawVect.reserve(m_openList.size());
		for (auto& node : m_openList) {
			drawVect.emplace_back(node.second.state->getPosition());
		}
		view::DebugDraw::instance().points(drawVect);
		drawVect.clear();
		drawVect.reserve(m_closedList.size());
		for (const auto& node : m_closedList) {
			drawVect.emplace_back(std::get<0>(node) * m_cellSize, std::get<1>(node) * m_cellSize);
		}
		view::DebugDraw::instance().points2(drawVect, sf::Color::Red);
		drawVect.clear();
		drawVect.reserve(m_collidingList.size());
		for (const auto& node : m_collidingList) {
			drawVect.emplace_back(std::get<0>(node) * m_cellSize, std::get<1>(node) * m_cellSize);
		}
		view::DebugDraw::instance().points3(drawVect, sf::Color::White);
#endif // ENABLE_DEBUG_DRAW

		std::vector<std::unique_ptr<IVehicleState>> path;
		const PathNode* currentNode = &m_finalNode;
		path.emplace_back(currentNode->state->clone());
		currentNode = currentNode->parent.get();
		while (currentNode != nullptr && currentNode->parent.get() != currentNode) {
			path.emplace_back(currentNode->state->clone());
			currentNode = currentNode->parent.get();
		}
		std::ranges::reverse(path);
		m_plannedPath=Path(std::move(path));
	}

	void PathPlanner::selectSteeringMode(SteeringMode const& mode, double const*& inputs, size_t & size)
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

	void model::pathPlanning::PathPlanner::updateNeightbours(std::unordered_set<const model::Cone*> const& cones, PathNode const& node, int stage)
	{
		//std::cout << "Start of m_uNb\n";
		auto sharedParent = std::make_shared<PathNode>(node);

		for (int i = 0; i < m_anglesSize;i++) {
			// Set leaf partly
			PathNode newNode;
			newNode.state = stepByDistance(*node.state, m_stepSize, m_steeringInputs[i]);
			newNode.stage = stage;

			std::tuple<int, int, int,int> newNodeKey = discretizePoint(newNode.state->getPose(),stage);

			// Check if the new node is in the closed list
			if (m_closedList.find(newNodeKey) == m_closedList.end()) {
				//	Interpolate the path between the current node and the new node and check each collision
				auto [isColliding, contactPose] = checkCollisionWithinStep(3, *node.state, cones);
				// if collision add to closed list
				if (isColliding) {
					std::tuple<int, int, int,int> collidingKey = discretizePoint(contactPose,stage);
					m_closedList.emplace(collidingKey);
				}
				else {
					newNode.parent = sharedParent;
					newNode.gCost = node.gCost + m_stepSize;

					processValidNode(newNode, newNodeKey,stage);
				}
			}
		}
		//std::cout << "End of m_uNb\n";
	}

	std::unique_ptr<IVehicleState> PathPlanner::stepByDistance(IVehicleState const& state, double distance, double steeringInput)
	{
		auto copy = state.clone();
		double traveled = 0;
		double dt = 0.01; // small fixed step
		copy->setTarget(VelocityCommand(1,steeringInput));

		while (traveled < distance) {
			copy->updateState(dt);
			traveled += copy->getSpeed() * dt;
		}

		return copy;
	}

	std::tuple<int, int, int,int> PathPlanner::discretizePoint(model::Pose const& pose,int stage) const
	{
		int x = static_cast<int>(pose.x / m_cellSize);
		int y = static_cast<int>(pose.y / m_cellSize);
		double shiftedTheta = radian(pose.theta) + std::numbers::pi;  // now in [0, 2π)
		double binWidth = 2 * std::numbers::pi / m_angleBins;
		int angle = static_cast<int>(std::floor(shiftedTheta / binWidth));
		return std::make_tuple(x, y, angle,stage);
	}


	std::pair<bool, model::Pose> PathPlanner::detectCollision(IVehicleState const& state, std::unordered_set<const model::Cone*> const& cones) const
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
				//Point contactPoint = rotatePoint(clampedPoint, vehicleOrientation) + vehiclePosition;
				return { true, state.getPose() };
			}
		}
		return { false,state.getPose() }; // No collision
	}

	std::pair<bool, model::Pose> PathPlanner::lazyDetectCollision(IVehicleState const& state, std::unordered_set<const model::Cone*> const& cones) const
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

	std::pair<bool, model::Pose> PathPlanner::checkCollisionWithinStep(int stepCount, IVehicleState const& state, std::unordered_set<const model::Cone*> const& cones)
	{
		if (cones.empty()) 
		{
			return { false, Pose() };
		}
		double minAllowedStep = m_cellSize * 1.5; // diagonal step size
		double smallStepSize = clamp(m_stepSize / stepCount, minAllowedStep, m_stepSize);
		bool isColliding=false;
		Pose contactPose;
		double currentStep = smallStepSize;
		while (currentStep < m_stepSize - 1e-8) {
			auto collisionResult = detectCollision(*stepByDistance(state, currentStep, state.getTarget().angular), cones);
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
			auto collisionResult = detectCollision(*stepByDistance(state, m_stepSize, state.getTarget().angular), cones);
			isColliding = collisionResult.first;
			contactPose = collisionResult.second;
		}

		return { isColliding,contactPose };
	}

	PathNode model::pathPlanning::PathPlanner::createNewNode(PathNode const& node, double steeringInput, int stage) const
	{
		PathNode newNode;
		newNode.state = stepByDistance(*node.state, m_stepSize, steeringInput);
		newNode.stage = stage;
		return newNode;
	}

	void PathPlanner::processValidNode(PathNode & node, std::tuple<int, int, int,int> const& key,int stage)
	{
		double heuristics;
		if (stage == 0) 
		{
			heuristics = getHeuristics(*node.state, m_goals.front(), m_goals.back());
		} else
		{
			heuristics = getHeuristics(*node.state, m_goals.back());
		}
		double fCost = node.gCost + heuristics;
		node.fCost = fCost;
		// if not in open list, add it
		auto [it, isInserted] = m_openList.emplace(key, std::move(node));

		if (isInserted) 
		{
			m_openQueue.emplace(fCost, key);
		}
		else if (node.gCost < m_openList[key].gCost) 
		{
			// if in open list, replace if cost is lower
			m_openList[key] = std::move(node);
			m_openQueue.emplace(fCost, key);
		}

	}

	std::vector<model::Point> PathPlanner::getBoundary(IVehicleState const& state) const
	{
		throw std::runtime_error("getBoundary not implemented");
	}

	model::Point PathPlanner::rotatePoint(model::Point const& point, Angle const& angle)
	{
		return { point.X() * model::cos(angle) - point.Y() * model::sin(angle), point.X() * model::sin(angle) + point.Y() * model::cos(angle) };
	}

	double PathPlanner::getHeuristics(model::IVehicleState const& start, model::Point const& goalPoint)
	{
		return m_dubins.simpleDistance(start.getPose(), goalPoint) + start.getVelocity().omega;
			//+ (start.getSteeringAngle() / start.getMaxSteeringAngle()) * (start.getSteeringAngle() / start.getMaxSteeringAngle());
	}

	double PathPlanner::getHeuristics(model::Point const& startPoint, model::Point const& goalPoint)
	{
		return (startPoint - goalPoint).magnitude();

	}

	double PathPlanner::getHeuristics(model::IVehicleState const& start, model::Point const& wayPoint, model::Point const& goalPoint)
	{
		return m_dubins.multipleDistance(start.getPose(), wayPoint, goalPoint) + start.getVelocity().omega;
			//+ (start.getSteeringAngle() / start.getMaxSteeringAngle()) * (start.getSteeringAngle() / start.getMaxSteeringAngle());
	}

	std::vector<std::unique_ptr<model::IVehicleState>> PathPlanner::stepUntilNew(IVehicleState const& state, double distanceStep) const
	{
		double diagonalStepSize = m_cellSize * 1.5;
		distanceStep = std::min(std::max(distanceStep, diagonalStepSize), m_stepSize);

		std::vector<std::unique_ptr<IVehicleState>> states;
		double currentStep = distanceStep;
		while (currentStep < m_stepSize - 1e-8) {
			//VehicleState currentState = stepByDistance(state, currentStep, state.getSteeringAngle());
			states.push_back(stepByDistance(state, currentStep, state.getTarget().angular));
			currentStep += distanceStep;
		}
		return states;
	}

	bool model::pathPlanning::PathPlanner::isAtGoal(const model::IVehicleState& state, int index) const
	{
		if (index == 0) {
			return (state.getPosition() - m_goals.front()).magnitude() < m_stepSize;

		}

		Point relPos = state.getPosition() - m_goals.back();
		Angle theta = state.getOrientation();
		double xDist = relPos.X() * cos(theta) + relPos.Y() * sin(theta);
		double yDist = - relPos.X() * sin(theta) + relPos.Y() * cos(theta);
		return abs(yDist) < m_stepSize && abs(xDist) < m_stepSize;


	}

	bool PathPlanner::hasMoreNodes() const {
		return !m_openList.empty() &&
			m_openList.size() < m_maxContainerSize &&
			m_closedList.size() < m_maxContainerSize;
	}


	void PathPlanner::setDubins(double minTurningRadius)
	{
		m_dubins = DubinsStateSpace(minTurningRadius);
	}

	void PathPlanner::clear()
	{
		//Point lastGoal= m_goals.back();
		//m_goals.clear();
		//m_goals.emplace_back(lastGoal);
		m_openList.clear();
		std::vector<PQNode> PQTemp;
		PQTemp.reserve(m_maxContainerSize);
		m_openQueue = std::priority_queue<PQNode,
			std::vector<PQNode>,
			std::greater<>>(std::greater<>(), std::move(PQTemp));
		m_closedList.clear();
		m_openList.reserve(m_maxContainerSize);
		m_closedList.reserve(m_maxContainerSize);
		//std::cout << "Cleared PathPlanner data\n";
	}
}