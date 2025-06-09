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
		//Vehicle(std::unique_ptr<model::VehicleState> vehicleState, std::unique_ptr<controller::IControllerLogic> carControl);
		Vehicle(std::unique_ptr<model::IControllerLogic> carControl,std::unique_ptr<model::pathPlanning::PathPlanner> pathPlanner);

		void update(double dt);
		Point getPosition() const override;
		double getOrientation() const;
		void planPath(std::vector<const model::Cone*> const& cones, VehicleState const& state);
		void setPlannedPath();
		FixSizedQueue<Path> getPlannedPath() const;
		void setGoal(Point goal);
		Pose getPose();
		void setPose(double x, double y, std::optional<double> orientation);
		double getLength() const;
		double getWidth() const;
		double getWheelBase() const { _state->getWheelBase(); }
		void clearPath();
		double getSpeed() const;
		VehicleState getStateCopy() const;
		double getCellSize();


		~Vehicle() = default;  

	private:  
		Vehicle();
		std::unique_ptr<model::IControllerLogic> _control; 
		std::unique_ptr<model::VehicleState> _state;
		std::unique_ptr<model::Perception> _perception;
		std::unique_ptr<model::pathPlanning::PathPlanner> _pathPlanner;

	};  
}