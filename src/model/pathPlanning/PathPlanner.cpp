#include "PathPlanner.h"
#include "model/items/Cone.h"
#include "model/utils/Angle.h"
#include "model/utils/Pose.h"
#include <thread>

#ifdef ENABLE_DEBUG_DRAW
#include "view/DebugDraw.h"
#endif // ENABLE_DEBUG_DRAW


namespace model {
	namespace pathPlanning {
		PathPlanner::PathPlanner(int iterations, double deltaSpace, double cellSize, double steeringStep) :
			_iterations(iterations), _deltaSpace(deltaSpace), _cellSize(cellSize), _steeringStep(steeringStep), _goal(-47.5, 10.0),
			_dubins(3.0 / tan(M_PI / 6.0))
		{
			
		}

		void PathPlanner::planPath(std::vector<const model::Cone*> const& cones, model::VehicleState const& vehicleState)
		{
			Angle maxSteeringAngle = vehicleState.getMaxSteeringAngle();
			Angle beta = atan(tan(maxSteeringAngle) / 2);
			double radius = vehicleState.getWheelBase() / (tan(maxSteeringAngle) * cos(beta));

			_dubins = DubinsStateSpace(radius);
			int iter = 0;
			_steeringAngles = { 0.0,
				0.05,-0.05 ,
				0.25,-0.25 ,
				0.5,-0.5 ,
				0.75,-0.75 ,
				1.0,-1.0 };

			PathNode startNode(
				vehicleState,
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
#ifdef ENABLE_DEBUG_DRAW
				_collidingList.emplace(startPQ.key);
#endif // ENABLE_DEBUG_DRAW
				std::cout << "Inside a cone." << std::endl;
				return;
			}
			// Add startNode to open list
			_openList.emplace(startPQ.key, startNode);

			_openQueue.emplace(startPQ);
			//while (!_openList.empty()) {
			while (!_openList.empty() && _openList.size() < MAX_CONTAINER_SIZE && _closedList.size() < MAX_CONTAINER_SIZE) {
				// Find the node with the lowest cost in the open list

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
				// Goal Reached
				if ((currentNode.state.getPosition() - _goal).magnitude() < _deltaSpace ) {
					_currentNode = currentNode;
					//std::cout << "Goal reached in "<<iter<<" iterations." << std::endl;
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

		void PathPlanner::setGoal(Point const& goal)
		{
			_goal = goal;
		}



		void PathPlanner::setPlannedPath()
		{
#ifdef ENABLE_DEBUG_DRAW
			std::vector<Point> drawVect;
			for (auto& node : _openList) {
				drawVect.emplace_back(node.second.state.getPosition());
			}
			view::DebugDraw::instance().points(drawVect);
			drawVect.clear();
			for (auto& node : _closedList) {
				drawVect.emplace_back(std::get<0>(node) * _cellSize, std::get<1>(node) * _cellSize);
			}
			view::DebugDraw::instance().points2(drawVect, sf::Color::Red);
			drawVect.clear();
			for (auto& node : _collidingList) {
				drawVect.emplace_back(std::get<0>(node) * _cellSize, std::get<1>(node) * _cellSize);
			}
			view::DebugDraw::instance().points3(drawVect, sf::Color::White);
#endif // ENABLE_DEBUG_DRAW

			std::vector<VehicleState> path;
			path.reserve(_iterations);
			PathNode* currentNode = &_currentNode;
			path.emplace_back(currentNode->state);
			currentNode = currentNode->parent.get();
			while (currentNode != nullptr && currentNode->parent.get() != currentNode) {
				path.emplace_back(currentNode->state);
				currentNode = currentNode->parent.get();
			}
			std::reverse(path.begin(), path.end());
			_plannedPath=Path(path);
		}

		void PathPlanner::_updateNeightbours(std::vector<const model::Cone*> const& cones, PathNode const& node)
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
					std::vector<VehicleState> interpolatedState = _stepUntilNew(node.state, _deltaSpace / 3.0);
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

							_openQueue.emplace(PQNode{ newNode.fCost,newNodeKey });
						}
						else if (newNode.gCost < _openList[newNodeKey].gCost) {
							// if in open list, replace if cost is lower
							_openList[newNodeKey] = newNode;
							_openQueue.emplace(PQNode{ newNode.fCost,newNodeKey });
						}

					}
				}




			}
		}

		model::VehicleState PathPlanner::_stepByDistance(VehicleState const& state, double distance, double steeringAngle)
		{
			VehicleState copy = state;
			double traveled = 0;
			double dt = 0.01; // small fixed step
			copy.setTargetSpeed(copy.getMaxSpeed());
			copy.setTargetSteeringAngle(steeringAngle);

			while (traveled < distance) {
				copy.updateState(dt);
				traveled += copy.getSpeed() * dt;
			}

			return copy;
		}

		std::tuple<int, int, int> PathPlanner::_discretizePoint(model::Pose const& pose) const
		{
			int x = static_cast<int>(pose.x / _cellSize);
			int y = static_cast<int>(pose.y / _cellSize);
			int angle = static_cast<int>(std::floor(pose.theta / (2 * M_PI / 20.0))) % 20;
			return std::make_tuple(x, y, angle);
		}

		std::pair<bool, model::Pose> PathPlanner::_detectCollision(VehicleState const& state, std::vector<const model::Cone*> const& cones) const
		{
			model::Point vehiclePosition(state.getPosition());
			Angle vehicleOrientation = state.getOrientation();
			if (cones.empty()) return { false, Pose() };
			double radius = cones[0]->getRadius();
			double vhclLength = state.getLength(), vhclWidth = state.getWidth();
			for (auto const& cone : cones) {
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

		std::pair<bool, model::Pose> PathPlanner::_lazyDetectCollision(VehicleState const& state, std::vector<const model::Cone*> const& cones) const
		{
			model::Point vehiclePosition(state.getPosition());
			Angle vehicleOrientation = state.getOrientation();
			if (cones.empty()) return { false, Pose() };
			double radius = cones[0]->getRadius();
			double vhclLength = state.getLength(), vhclWidth = state.getWidth();
			for (auto const& cone : cones) {
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

		std::vector<model::Point> PathPlanner::_getBoundary(VehicleState const& state) const
		{
			throw std::runtime_error("getBoundary not implemented");
		}

		model::Point PathPlanner::_rotatePoint(model::Point const& point, Angle const& angle) const
		{
			return model::Point(point.X() * model::cos(angle) - point.Y() * model::sin(angle), point.X() * model::sin(angle) + point.Y() * model::cos(angle));
		}

		double PathPlanner::_getHeuristics(model::VehicleState const& start, model::Point const& goalPoint)
		{
			//return (Point(start.getPose()) - goalPoint).magnitude(); //+ 10 * (start.getSteeringAngle() * start.getSteeringAngle()) / start.getMaxSteeringAngle();
			return _dubins.simpleDistance(start.getPose(), goalPoint)+ (start.getSteeringAngle() * start.getSteeringAngle()) / start.getMaxSteeringAngle();
		}

		std::vector<model::VehicleState> PathPlanner::_stepUntilNew(VehicleState const& state, double distanceStep)
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