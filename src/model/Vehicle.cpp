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
	Vehicle::Vehicle(std::unique_ptr<controller::Controller> carControl) : _state(std::make_unique<model::VehicleState>())
    {  
		assert(carControl != nullptr && "Controller must not be null");
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