#pragma once  
#include "src/model/controllerLogic/IControllerLogic.h"  
#include <src/model/pathPlanner/PathPlanner.h>


namespace controller  
{  
class KeyboardControl :  
	public model::IControllerLogic  
{  
	// Inherited via IControllerLogic  
	void drive(model::VehicleState& state, model::PathPlanner& pathPlanner) override;

	double detectSpeed();

	double detectAngle();


};  
}
