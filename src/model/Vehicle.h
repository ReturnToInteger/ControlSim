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

#ifndef M_PI  
#define M_PI 3.14159265358979323846 // Define M_PI if not already defined  
#endif

namespace model { 
	
	class IControllerLogic;
	class VehicleState;
	class Perception;
	class Cone;
	namespace pathPlanning {
		class PathPlanner;
	}


	class Vehicle : public Item {  
	public:
		Vehicle(std::unique_ptr<model::IControllerLogic> carControl,std::unique_ptr<model::pathPlanning::PathPlanner> pathPlanner);

		// state and control methods

		void update(double dt);
		Point getPosition() const override { return _state->getPosition(); }
		Angle getOrientation() const { return _state->getOrientation(); }
		Pose getPose() const { return _state->getPose(); }
		void setPose(double x, double y, std::optional<double> orientation);
		double getLength() const { return _state->getLength(); }
		double getWidth() const { return _state->getWidth(); }
		double getWheelBase() const { _state->getWheelBase(); }
		double getSpeed() const { return _state->getSpeed(); }
		VehicleState getStateCopy() const { return *_state; }

		// path planning methods

		void planPath(std::unordered_set<const model::Cone*> const& cones, VehicleState const& state, int i);
		void setPlannedPath(int i); 
		FixSizedQueue<Path> getPlannedPaths() const;
		std::vector<Point> getGoal();
		void setGoal(Point goal, int i);
		void setAllGoals(Point goal);
		void clearPath(int i);
		double getCellSize(int i=0) const;
		void addPlanner();


		~Vehicle();  

	private:  
		Vehicle();
		std::unique_ptr<model::IControllerLogic> _control; 
		std::unique_ptr<model::VehicleState> _state;
		std::unique_ptr<model::Perception> _perception;
		std::vector<std::unique_ptr<model::pathPlanning::PathPlanner>> _pathPlanners;

		FixSizedQueue<Path> _paths;

	};  
}