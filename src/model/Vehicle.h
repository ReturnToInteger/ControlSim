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
		Vehicle(std::unique_ptr<model::IControllerLogic<T>> carControl, std::unique_ptr<model::pathPlanning::PathPlanner> pathPlanner) :
			m_state(std::make_unique<T>()), m_paths(maxHistorySize)
		{
			if (carControl == nullptr) {
				throw std::invalid_argument("IControllerLogic must not be null");
			}
			if (pathPlanner == nullptr) {
				throw std::invalid_argument("Path planner must not be null");
			}
			m_pathPlanners.emplace_back(std::move(pathPlanner));
			m_control = std::move(carControl);
		}

		// state and control methods

		Pose getPose() const override { return m_state->getPose(); }
		Point getPosition() const override { return m_state->getPosition(); }
		Angle getOrientation() const override { return m_state->getOrientation(); }
		double getLength() const override { return m_state->getLength(); }
		double getWidth() const override { return m_state->getWidth(); }
		double getSpeed() const override { return m_state->getSpeed(); }
		std::unique_ptr<IVehicleState> getStateCopy() const override { return m_state->clone(); }
		
		double getWheelBase() const { m_state->getWheelBase(); }
		
		// Update method
		void update(double dt) override {
			if (dt <= 0) {
				dt = 0;
			}
			//  Needs to be fixed
			model::ControlCommand ctrl = m_control->drive(*m_state, m_paths.back());
			model::VelocityCommand command{
				.linear = ctrl.normSpeed,
				.angular = ctrl.normSteeringAngle
			};
			m_state->setTarget(command);
			m_state->updateState(dt);
		}
		void setPose(double x, double y, std::optional<double> orientation) override
		{
			if (orientation.has_value()) {
				m_state->setPose(x, y, orientation.value());
			}
			else {
				m_state->setPose(x, y, m_state->getOrientation());
			}
		}

		// path planning methods

		FixSizedQueue<Path> getPlannedPaths() const override
		{
			return m_paths;
		}
		double getCellSize(int i = 0) const override
		{
			if (i >= m_pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
			return m_pathPlanners[i]->getCellSize();
		}
		std::vector<Point> getGoal() const override
		{
			return {};
		}

		void clearPath(int i) override
		{
			if (i >= m_pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
			m_pathPlanners[i]->clear();
		}
		bool planPath(std::unordered_set<const model::Cone*> const& cones, IVehicleState const& state, int i) override
		{
			if (i >= m_pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
			return m_pathPlanners[i]->planPath(cones, state);
		}
		void setPlannedPath(int i) override
		{
			if (i >= m_pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }

			m_pathPlanners[i]->setPlannedPath();
			model::Path path = m_pathPlanners[i]->getPlannedPath();
			if (path.size() > 2) {
				m_paths.push(path);
			}
		}
		void setGoal(Point goal, int i) override
		{
			if (i >= m_pathPlanners.size()) { throw std::out_of_range("Index is out of range."); }
			m_pathPlanners[i]->setGoal(goal);
		}
		void setAllGoals(Point goal) override
		{
			//#ifdef ENABLE_DEBUG_DRAW
			//view::DebugDraw::instance().circle(goal, 0.5, sf::Color::Cyan);
			//#endif // ENABLE_DEBUG_DRAW

			for (int i = 0; i < m_pathPlanners.size(); i++) {
				setGoal(goal, i);
			}
		}
		void addPlanner() override
		{
			if (m_pathPlanners.empty()) {
				throw std::runtime_error("No base planner to copy from");
			}
			m_pathPlanners.emplace_back(std::make_unique<model::pathPlanning::PathPlanner>(*m_pathPlanners.back()));
		}

		~Vehicle() = default;

	private:  
		Vehicle() = default;
		std::unique_ptr<model::IControllerLogic<T>> m_control;
		std::unique_ptr<T> m_state;
		//std::unique_ptr<model::Perception> m_perception;
		std::vector<std::unique_ptr<model::pathPlanning::PathPlanner>> m_pathPlanners;

		FixSizedQueue<Path> m_paths;

	};  
}