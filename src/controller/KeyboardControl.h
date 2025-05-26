#pragma once  
#include "src/model/controllerLogic/IControllerLogic.h"  
#include <src/model/pathPlanner/PathPlanner.h>
#include <src/model/controllerLogic/ControlCommand.h>



namespace controller  
{  
class KeyboardControl :  
	public model::IControllerLogic  
{  
	// Inherited via IControllerLogic  
	model::ControlCommand drive(const model::VehicleState& state, const model::PathPlanner& pathPlanner) override;

	double detectSpeed();

	double detectAngle();


};  
}
