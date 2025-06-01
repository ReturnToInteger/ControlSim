#include "PathPlanner.h"
#include "model/items/Cone.h"
#include "model/utils/Angle.h"
#include "model/utils/Pose.h"

namespace model {
	namespace pathPlanning {
		PathPlanner::PathPlanner(int iterations, double deltaSpace, double cellSize, double steeringStep) :
			_iterations(iterations), _deltaSpace(deltaSpace), _cellSize(cellSize), _steeringStep(steeringStep), _goal(-47.5, 10.0),
			_dubins(3.0 / tan(M_PI / 6.0))
		{
		}

		void PathPlanner::planPath(const std::vector<const model::Cone*>& cones, const model::VehicleState& vehicleState)
		{

			double maxSteeringAngle = vehicleState.getMaxSteeringAngle();
			_dubins = DubinsStateSpace(vehicleState.getLength() / tan(maxSteeringAngle));
			int iter = 0;
			_steeringAngles = { 0.0,
				maxSteeringAngle * 0.2,-maxSteeringAngle * 0.2 ,
				maxSteeringAngle * 0.4,-maxSteeringAngle * 0.4 ,
				maxSteeringAngle * 0.75,-maxSteeringAngle * 0.75 ,
				maxSteeringAngle,-maxSteeringAngle };

			PathNode startNode(vehicleState,
				0,
				0 + _getHeuristics(vehicleState, _goal),
				nullptr);
			// If start is within goal, return
			if ((startNode.state.getPosition() - _goal).magnitude() < _deltaSpace / 2.0) {
				std::cout << "Reached Goal" << std::endl;
				return;
			}
			PQNode startPQ{ startNode.fCost,_discretizePoint(vehicleState.getPose()) };
			// Check collision for start node
			auto [isColliding, collisionPoint] = _detectCollision(startNode.state, cones);
			if (isColliding) {
				_collidingList.emplace(startPQ.key);
				return;
			}
			// Add startNode to open list
			_openList.emplace(startPQ.key, startNode);
			_openQueue.emplace(startPQ);
			//while (!_openList.empty()) {
			while (!_openList.empty() && _openList.size() < MAX_CONTAINER_SIZE && _closedList.size() < MAX_CONTAINER_SIZE) {
				// Find the node with the lowest cost in the open list
				//auto minNodeIt = std::min_element(_openList.begin(), _openList.end(),
				//	[](const auto& a, const auto& b) { return a.second < b.second; });
				//PathNode currentNode = minNodeIt->second;

				// Get smallest cost
				PQNode top = _openQueue.top();
				_openQueue.pop();
				_closedList.emplace(top.key); // Add the node to the closed list
				auto it = _openList.find(top.key);
				if (it == _openList.end()) continue; // stale node
				if (top.fCost > it->second.fCost + 1e-8) continue;
				PathNode currentNode(it->second);
				_openList.erase(it->first); // Remove the node from the open list

				// or (currentNode.cost > _iterations)
				if ((currentNode.state.getPosition() - _goal).magnitude() < _deltaSpace / 2.0) {
					_currentNode = currentNode;
					return;
				}
				else {
					_updateNeightbours(cones, currentNode);
				}

				iter++;
			}
			std::cout << "Path not found after " << iter << " iterations." << std::endl;
			_currentNode = startNode;
		}

		Path PathPlanner::getPlannedPath() const
		{
			return _plannedPath;
		}

		void PathPlanner::setGoal(const Point& goal)
		{
			_goal = goal;
		}



		void PathPlanner::setPlannedPath()
		{
			std::vector<Pose> path;
			path.reserve(_iterations);
			PathNode* currentNode = &_currentNode;
			path.emplace_back(currentNode->state.getPose());
			currentNode = currentNode->parent.get();
			while (currentNode != nullptr && currentNode->parent.get() != currentNode) {
				path.emplace_back(currentNode->state.getPose());
				currentNode = currentNode->parent.get();
			}
			std::reverse(path.begin(), path.end());
			_plannedPath = Path(path);
		}

		void PathPlanner::_updateNeightbours(const std::vector<const model::Cone*>& cones, PathNode& node)
		{
			double maxSteeringAngle = node.state.getMaxSteeringAngle();

			for (auto& steeringAngle : _steeringAngles) {
				//std::cout << "steering angle: " << steeringAngle << std::endl;
				PathNode newNode = node;
				newNode.state = _stepByDistance(node.state, _deltaSpace, steeringAngle);
				newNode.parent = std::make_shared<PathNode>(node);

				// Check if the new node is in the closed list
				std::tuple<int, int, int> newNodeKey = _discretizePoint(newNode.state.getPose());
				if (_closedList.find(newNodeKey) == _closedList.end()) {
					// Collision check
					bool isColliding = false;
					model::Pose contactPoint;
					//		Interpolate the path between the current node and the new node
					std::vector<VehicleState> interpolatedState = _stepUntilNew(node.state, _deltaSpace / 2.0);
					for (auto& state : interpolatedState) {
						auto collisionResult = _lazyDetectCollision(state, cones);
						if (collisionResult.first) {
							isColliding = true;
							contactPoint = collisionResult.second;
							break;
						}
					}
					if (!isColliding) {
						auto collisionResult = _detectCollision(newNode.state, cones);
						isColliding = collisionResult.first;
						contactPoint = collisionResult.second;
					}
					// if collision add to closed list
					if (isColliding) {
						std::tuple<int, int, Angle> collidingKey = _discretizePoint(contactPoint);
						_closedList.emplace(newNodeKey);
					}
					else {
						//calculate cost and heuristics
						newNode.gCost = node.gCost + _deltaSpace;
						newNode.fCost = newNode.gCost + _getHeuristics(node.state, _goal);
						// if not in open list, add it
						auto [it, isInserted] = _openList.emplace(newNodeKey, newNode);

						if (isInserted) {
							_openQueue.emplace(PQNode{ newNode.gCost,newNodeKey });
						}
						else if (newNode.gCost < _openList[newNodeKey].gCost) {
							// if in open list, replace if cost is lower
							_openList[newNodeKey] = newNode;
							_openQueue.emplace(PQNode{ newNode.gCost,newNodeKey });
						}

					}
				}




			}
		}

		model::VehicleState PathPlanner::_stepByDistance(const VehicleState& state, double distance, double steeringAngle)
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

		std::tuple<int, int, int> PathPlanner::_discretizePoint(const model::Pose& pose) const
		{
			int x = static_cast<int>((pose.x - _cellSize / 2.0) / _cellSize);
			int y = static_cast<int>((pose.y - _cellSize / 2.0) / _cellSize);
			int angle = static_cast<int>(std::floor((pose.theta - M_PI / 20.0) / (2 * M_PI)) * 20) % 20;
			return std::make_tuple(x, y, angle);
		}

		std::pair<bool, model::Pose> PathPlanner::_detectCollision(const VehicleState& state, const std::vector<const model::Cone*>& cones) const
		{
			model::Point vehiclePosition(state.getPosition());
			Angle vehicleOrientation = state.getOrientation();
			if (cones.empty()) return { false, Pose() };
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
				// within radius and margin for error
				if (distance < radius) {
					Point contactPoint = _rotatePoint(clampedPoint, vehicleOrientation) + vehiclePosition;
					return { true, Pose(contactPoint.X(),contactPoint.Y(), vehicleOrientation) };
				}
			}
			return { false,Pose() }; // No collision
		}

		std::pair<bool, model::Pose> PathPlanner::_lazyDetectCollision(const VehicleState& state, const std::vector<const model::Cone*>& cones) const
		{
			model::Point vehiclePosition(state.getPosition());
			Angle vehicleOrientation = state.getOrientation();
			if (cones.empty()) return { false, Pose() };
			double radius = cones[0]->getRadius();
			double vhclLength = state.getLength(), vhclWidth = state.getWidth();
			for (const auto& cone : cones) {
				if (cone->getType() == model::ConeType::UNKNOWN) {
					continue; // Skip unknown cones
				}
				Point relPos(cone->getPosition() - vehiclePosition);
				if (relPos.magnitude() > (vhclLength * 1.5 + radius)) continue;
				// within radius and margin for error
				if (relPos.magnitude() < radius + vhclWidth / 2.0) {
					return { true, Pose(vehiclePosition.X(),vehiclePosition.Y(), vehicleOrientation) };
				}
			}
			return { false,Pose() }; // No collision
		}

		std::vector<model::Point> PathPlanner::_getBoundary(const VehicleState& state) const
		{
			throw std::runtime_error("getBoundary not implemented");
		}

		model::Point PathPlanner::_rotatePoint(const model::Point& point, const Angle& angle) const
		{
			return model::Point(point.X() * model::cos(angle) - point.Y() * model::sin(angle), point.X() * model::sin(angle) + point.Y() * model::cos(angle));
		}

		double PathPlanner::_getHeuristics(const model::VehicleState& start, const model::Point& goalPoint)
		{
			//return (point - goal).magnitude();
			return _dubins.simpleDistance(start.getPose(), goalPoint) + abs(start.getSteeringAngle()) / start.getMaxSteeringAngle();
		}

		std::vector<model::VehicleState> PathPlanner::_stepUntilNew(VehicleState& state, double distanceStep)
		{
			distanceStep = std::min(std::max(distanceStep, _cellSize * 1.5), _deltaSpace);

			std::vector<VehicleState> states;
			double currentStep = distanceStep;
			while (currentStep < _deltaSpace - 1e-8) {
				//VehicleState currentState = _stepByDistance(state, currentStep, state.getSteeringAngle());
				states.push_back(_stepByDistance(state, currentStep, state.getSteeringAngle()));
				currentStep += distanceStep;
			}
			return states;
		}

		void PathPlanner::clear()
		{
			_openList.clear();
			_openQueue = std::priority_queue<PQNode,
				std::vector<PQNode>,
				std::greater<PQNode>>();
			_closedList.clear();
			_openList.reserve(MAX_CONTAINER_SIZE);
			_closedList.reserve(MAX_CONTAINER_SIZE);
			//std::cout << "Cleared PathPlanner data" << std::endl;
		}

	}
}