#pragma once
#include "Vehicle.h"



namespace model {
	/*Vehicle::Vehicle(std::unique_ptr<model::VehicleState> vehicleState, std::unique_ptr<controller::Controller> carControl) :
		_state(std::move(vehicleState)),
		_control(std::move(carControl))
	{
		assert(_state != nullptr && "VehicleState must not be null");
		assert(_control != nullptr && "Controller must not be null");
	}*/
	Vehicle::Vehicle(std::unique_ptr<controller::Controller> carControl, std::unique_ptr<model::PathPlanner> pathPlanner) : 
		_state(std::make_unique<model::VehicleState>())
    {  
		if (carControl == nullptr) {
			throw std::invalid_argument("Controller must not be null");
		}
		if (pathPlanner == nullptr) {
			throw std::invalid_argument("Path planner must not be null");
		}
		_pathPlanner = std::move(pathPlanner);
        _control= std::move(carControl);  
    }


	// Update method
   void model::Vehicle::update(double dt) {
	   _control->drive(*_state);
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

   void Vehicle::planPath(const std::vector<model::Cone*>& cones)
   {
	   _pathPlanner->planPath(cones, *_state);
   }

   std::vector<model::Point> Vehicle::getPlannedPath() const
   {
	   return _pathPlanner->getPlannedPath();
   }

   void Vehicle::setPose(Point position, std::optional<double> orientation)
   {
	   if (orientation.has_value()) {
		   _state->setPose(position, orientation.value());
	   }
	   else {
		   _state->setPose(position, _state->getOrientation());
	   }
   }




   }