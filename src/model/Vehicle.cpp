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



namespace model {
	/*Vehicle::Vehicle(std::unique_ptr<model::VehicleState> vehicleState, std::unique_ptr<controller::IControllerLogic> carControl) :
		_state(std::move(vehicleState)),
		_control(std::move(carControl))
	{
		assert(_state != nullptr && "VehicleState must not be null");
		assert(_control != nullptr && "IControllerLogic must not be null");
	}*/
	Vehicle::Vehicle(std::unique_ptr<model::IControllerLogic> carControl, std::unique_ptr<model::pathPlanning::PathPlanner> pathPlanner) : 
		_state(std::make_unique<model::VehicleState>())
    {  
		if (carControl == nullptr) {
			throw std::invalid_argument("IControllerLogic must not be null");
		}
		if (pathPlanner == nullptr) {
			throw std::invalid_argument("Path planner must not be null");
		}
		_pathPlanner = std::move(pathPlanner);
        _control= std::move(carControl);  
    }


	// Update method
   void model::Vehicle::update(double dt) {
	   if (dt <= 0) {
		   dt = 0;
	   }
	   model::ControlCommand command=_control->drive(*_state, *_pathPlanner);
	   _state->setTargetSpeed(command.speed);
	   _state->setTargetSteeringAngle(command.steeringAngle);
	   _state->updateState(dt);
   }


   Point Vehicle::getPosition() const
   {
	   return _state->getPosition();
   }

   double Vehicle::getOrientation() const
   {
	   return _state->getOrientation();
   }

   void model::Vehicle::planPath(const std::vector<const model::Cone*>& cones, const VehicleState & state)
   {

	   _pathPlanner->planPath(cones, state);
   }

   void Vehicle::setPlannedPath()
   {
	   _pathPlanner->setPlannedPath();
   }

   Path model::Vehicle::getPlannedPath() const
   {
	   return _pathPlanner->getPlannedPath();
   }

   void Vehicle::setGoal(Point goal)
   {
	   _pathPlanner->setGoal(goal);
   }

   Pose Vehicle::getPose()
   {
	   return _state->getPose();
   }

   void model::Vehicle::setPose(double x, double y, std::optional<double> orientation)
   {
	   if (orientation.has_value()) {
		   _state->setPose(x, y, orientation.value());
	   }
	   else {
		   _state->setPose(x, y, _state->getOrientation());
	   }
   }

   double Vehicle::getLength() const { return _state->getLength(); }

   double Vehicle::getWidth() const { return _state->getWidth(); }

   void Vehicle::clearPath() { _pathPlanner->clear(); }

   double Vehicle::getSpeed() const { return _state->getSpeed(); }

   VehicleState Vehicle::getStateCopy() const
   {
	   return *_state;
   }

   double Vehicle::getCellSize() { return _pathPlanner->getCellSize(); }




   }