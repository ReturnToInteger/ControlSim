#pragma once
#include "Vehicle.h"
#include "model/controllerLogic/IControllerLogic.h"
#include "model/perception/Perception.h"
#include "model/pathPlanning/PathPlanner.h"
#include "model/items/Cone.h"
#include <cmath>  
#include <cassert>
//#include "model/utils/ModelUtils.h"
#include "model/controllerLogic/ControlCommand.h"
#ifdef ENABLE_DEBUG_DRAW
#include "view/DebugDraw.h"
#endif // ENABLE_DEBUG_DRAW



namespace model {
	Vehicle::Vehicle(std::unique_ptr<model::IControllerLogic> carControl, std::unique_ptr<model::pathPlanning::PathPlanner> pathPlanner) :
		_state(std::make_unique<model::VehicleState>()), _paths(5)
	{
		if (carControl == nullptr) {
			throw std::invalid_argument("IControllerLogic must not be null");
		}
		if (pathPlanner == nullptr) {
			throw std::invalid_argument("Path planner must not be null");
		}
		_pathPlanners.emplace_back(std::move(pathPlanner));
		_control = std::move(carControl);
	}


	// Update method
	void model::Vehicle::update(double dt) {
		if (dt <= 0) {
			dt = 0;
		}
		model::ControlCommand command = _control->drive(*_state, _paths.back());
		_state->setTarget(command);
		_state->updateState(dt);
	}


	bool model::Vehicle::planPath(std::unordered_set<const model::Cone*> const& cones, VehicleState const& state, int i)
	{
		if (i >= _pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
		return _pathPlanners[i]->planPath(cones, state);
	}

	void model::Vehicle::setPlannedPath(int i)
	{
		if (i >= _pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }

		_pathPlanners[i]->setPlannedPath();
		model::Path path=_pathPlanners[i]->getPlannedPath();
		if (path.size() > 2) {
			_paths.push(path);
		}
	}

	FixSizedQueue<Path> model::Vehicle::getPlannedPaths() const
	{
		return _paths;
	}

	std::vector<Point>Vehicle::getGoal()
	{
		return std::vector<Point>();
	}

	void Vehicle::setGoal(Point goal, int i)
	{
		if (i >= _pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
		_pathPlanners[i]->setGoal(goal);
	}

	void Vehicle::clearPath(int i)
	{
		if (i >= _pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
		_pathPlanners[i]->clear();
	}

	void Vehicle::setAllGoals(Point goal)
	{
		//#ifdef ENABLE_DEBUG_DRAW
		//view::DebugDraw::instance().circle(goal, 0.5, sf::Color::Cyan);
		//#endif // ENABLE_DEBUG_DRAW

		for (int i=0; i < _pathPlanners.size(); i++) {
			setGoal(goal, i);
		}
	}


	double Vehicle::getCellSize(int i) const
	{
		if (i >= _pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
		return _pathPlanners[i]->getCellSize();
	}

	void Vehicle::addPlanner()
	{
		if (_pathPlanners.empty()) {
			throw std::runtime_error("No base planner to copy from");
		}
		_pathPlanners.emplace_back(std::make_unique<model::pathPlanning::PathPlanner>(*_pathPlanners.back()));
	}

	Vehicle::~Vehicle() = default;

	void model::Vehicle::setPose(double x, double y, std::optional<double> orientation)
	{
		if (orientation.has_value()) {
			_state->setPose(x, y, orientation.value());
		}
		else {
			_state->setPose(x, y, _state->getOrientation());
		}
	}


}