#pragma once  
#include "Item.h"  
#include <memory>  
#include "Vec3.h"  
#include <cmath>  
#include <cassert>
#include "utils/ModelUtils.h"
#include <optional>
#include "src\controller\Controller.h"

#ifndef M_PI  
#define M_PI 3.14159265358979323846 // Define M_PI if not already defined  
#endif  
#include "VehicleState.h"
#include "perception/Perception.h"
#include "pathPlanner/PathPlanner.h"



namespace model {  
   class Vehicle : public Item {  
   public:
	   //Vehicle(std::unique_ptr<model::VehicleState> vehicleState, std::unique_ptr<controller::Controller> carControl);
       Vehicle(std::unique_ptr<controller::Controller> carControl,std::unique_ptr<model::PathPlanner> pathPlanner);

       void update(double dt);
       Point getPosition() const override;
	   double getOrientation() const;
	   void planPath(const std::vector<model::Cone*>& cones);
	   std::vector<model::Point> getPlannedPath() const;
	   void setPose(Point position, std::optional<double> orientation);
	   double getLength() const { return _state->getLength(); }
	   double getWidth() const { return _state->getWidth(); }
	   void clearPath() { _pathPlanner->clear(); }
	   double getSpeed() const { return _state->getSpeed(); }


       ~Vehicle() = default;  

   private:  
       Vehicle();
       std::unique_ptr<controller::Controller> _control; 
	   std::unique_ptr<model::VehicleState> _state;
	   std::unique_ptr<model::Perception> _perception;
	   std::unique_ptr<model::PathPlanner> _pathPlanner;

   };  
}