#pragma once  
#include "Item.h"  
#include <memory>  
#include "Vec3.h"  
#include <cmath>  
#include <cassert>

#include "utils/ModelUtils.h"
#include <optional>
#include "src/model/controllerLogic/IControllerLogic.h"
#include "src/model/controllerLogic/ControlCommand.h"


#ifndef M_PI  
#define M_PI 3.14159265358979323846 // Define M_PI if not already defined  
#endif  
#include "VehicleState.h"
#include "perception/Perception.h"
#include "pathPlanner/PathPlanner.h"



namespace model {  
   class Vehicle : public Item {  
   public:
	   //Vehicle(std::unique_ptr<model::VehicleState> vehicleState, std::unique_ptr<controller::IControllerLogic> carControl);
       Vehicle(std::unique_ptr<model::IControllerLogic> carControl,std::unique_ptr<model::PathPlanner> pathPlanner);

       void update(double dt);
       Point getPosition() const override;
	   double getOrientation() const;
	   void planPath(const std::vector<const model::Cone*>& cones, const VehicleState & state);
	   void setPlannedPath();
	   std::vector<model::Pose> getPlannedPath() const;
	   Pose getPose();
	   void setPose(double x, double y, std::optional<double> orientation);
	   double getLength() const { return _state->getLength(); }
	   double getWidth() const { return _state->getWidth(); }
	   void clearPath() { _pathPlanner->clear(); }
	   double getSpeed() const { return _state->getSpeed(); }
	   VehicleState getStateCopy() const;


       ~Vehicle() = default;  

   private:  
       Vehicle();
       std::unique_ptr<model::IControllerLogic> _control; 
	   std::unique_ptr<model::VehicleState> _state;
	   std::unique_ptr<model::Perception> _perception;
	   std::unique_ptr<model::PathPlanner> _pathPlanner;

   };  
}