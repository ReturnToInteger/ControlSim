#pragma once  
#include "model/items/Item.h"  
#include "model/items/Path.h"  
#include <memory>  
#include <optional>
#include "model/utils/Pose.h"
#include "model/utils/Point.h"
#include "model/utils/FixSizedQueue.h"
#include "model/VehicleState.h"
#include "model/pathPlanning/PathPlanner.h"
#include "model/perception/Perception.h"
#include "model/controllerLogic/IControllerLogic.h"
#include <numbers>
#include "model/IVehicle.h"
#include "Vehicle.h"
#include "model/items/Cone.h"
#include <cmath>  
#include <cassert>
//#include "model/utils/ModelUtils.h"
#include "model/controllerLogic/ControlCommand.h"
#ifdef ENABLE_DEBUG_DRAW
#include "view/DebugDraw.h"
#endif // ENABLE_DEBUG_DRAW

namespace model { 

	class IControllerLogic;
	class VehicleState;
	class IVehicleState;
	class Perception;
	class Cone;
	namespace pathPlanning {
		class PathPlanner;
	}

	template<typename T>
	concept FromIVehicleState = std::is_base_of_v<IVehicleState, T>;


	template <FromIVehicleState T>
	class Vehicle : public IVehicle {
	public:
		Vehicle(std::unique_ptr<model::IControllerLogic> carControl, std::unique_ptr<model::pathPlanning::PathPlanner> pathPlanner) :
			_state(std::make_unique<T>()), _paths(maxHistorySize)
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

		// state and control methods

		Pose getPose() const override { return _state->getPose(); }
		Point getPosition() const override { return _state->getPosition(); }
		Angle getOrientation() const override { return _state->getOrientation(); }
		double getLength() const override { return _state->getLength(); }
		double getWidth() const override { return _state->getWidth(); }
		double getSpeed() const override { return _state->getSpeed(); }
		std::unique_ptr<IVehicleState> getStateCopy() const override { return _state->clone(); }
		
		double getWheelBase() const { _state->getWheelBase(); }
		
		// Update method
		void update(double dt) override {
			if (dt <= 0) {
				dt = 0;
			}
			//  Needs to be fixed
			model::ControlCommand ctrl = _control->drive(*_state, _paths.back());
			model::VelocityCommand command{
				.linear = ctrl.normSpeed,
				.angular = ctrl.normSteeringAngle
			};
			_state->setTarget(command);
			_state->updateState(dt);
		}
		void setPose(double x, double y, std::optional<double> orientation) override
		{
			if (orientation.has_value()) {
				_state->setPose(x, y, orientation.value());
			}
			else {
				_state->setPose(x, y, _state->getOrientation());
			}
		}

		// path planning methods

		FixSizedQueue<Path> getPlannedPaths() const override
		{
			return _paths;
		}
		double getCellSize(int i = 0) const override
		{
			if (i >= _pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
			return _pathPlanners[i]->getCellSize();
		}
		std::vector<Point> getGoal() const override
		{
			return {};
		}

		void clearPath(int i) override
		{
			if (i >= _pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
			_pathPlanners[i]->clear();
		}
		bool planPath(std::unordered_set<const model::Cone*> const& cones, IVehicleState const& state, int i) override
		{
			if (i >= _pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
			return _pathPlanners[i]->planPath(cones, state);
		}
		void setPlannedPath(int i) override
		{
			if (i >= _pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }

			_pathPlanners[i]->setPlannedPath();
			model::Path path = _pathPlanners[i]->getPlannedPath();
			if (path.size() > 2) {
				_paths.push(path);
			}
		}
		void setGoal(Point goal, int i) override
		{
			if (i >= _pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
			_pathPlanners[i]->setGoal(goal);
		}
		void setAllGoals(Point goal) override
		{
			//#ifdef ENABLE_DEBUG_DRAW
			//view::DebugDraw::instance().circle(goal, 0.5, sf::Color::Cyan);
			//#endif // ENABLE_DEBUG_DRAW

			for (int i = 0; i < _pathPlanners.size(); i++) {
				setGoal(goal, i);
			}
		}
		void addPlanner() override
		{
			if (_pathPlanners.empty()) {
				throw std::runtime_error("No base planner to copy from");
			}
			_pathPlanners.emplace_back(std::make_unique<model::pathPlanning::PathPlanner>(*_pathPlanners.back()));
		}

		~Vehicle() = default;

	private:  
		Vehicle() = default;
		std::unique_ptr<model::IControllerLogic> _control; 
		std::unique_ptr<T> _state;
		//std::unique_ptr<model::Perception> _perception;
		std::vector<std::unique_ptr<model::pathPlanning::PathPlanner>> _pathPlanners;

		FixSizedQueue<Path> _paths;

	};  
}