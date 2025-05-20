#pragma once  
#include "Item.h"  
#include <memory>  
#include "Vec3.h"  
#include <cmath>  
#include "src\controller\Controller.h"

#ifndef M_PI  
#define M_PI 3.14159265358979323846 // Define M_PI if not already defined  
#endif  
#include "VehicleState.h"



namespace model {  
   class Vehicle : public Item {  
   public:
	   //Vehicle(std::unique_ptr<model::VehicleState> vehicleState, std::unique_ptr<controller::Controller> carControl);
       Vehicle(std::unique_ptr<controller::Controller> carControl);

       void update(double dt);
       Point getPosition() const override;
	   double getOrientation() const;
	   double getLength() const { return _state->getLength(); }
	   double getWidth() const { return _state->getWidth(); }


       ~Vehicle() = default;  

   private:  
       Vehicle();
       std::unique_ptr<controller::Controller> _control; 
	   std::unique_ptr<model::VehicleState> _state;

   };  
}