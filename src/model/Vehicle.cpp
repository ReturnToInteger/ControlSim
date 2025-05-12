#pragma once
#include "Vehicle.h"
#include <cassert>
#include "utils/ModelUtils.h"



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

   void Vehicle::steer()
   {

	   //_targetSteeringAngle = _carControl->steer() * _maxSteeringAngle;
   }

   void Vehicle::drive()
   {

	   //_targetSpeed = _carControl->drive() * _maxSpeed;
   }

   void Vehicle::brake()
   {
	   //_brakeAcceleration = _carControl->brake() * _maxBrake;
   }

   Point Vehicle::getPosition() const
   {
	   return _state->getPosition();
   }



   }