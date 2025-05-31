#pragma once  
#include "model/controllerLogic/IControllerLogic.h"  
#include <model/controllerLogic/ControlCommand.h>



namespace controller  
{
	class model::VehicleState;
	class model::PathPlanner;
	class KeyboardControl :  
		public model::IControllerLogic  
	{  
		// Inherited via IControllerLogic  
		model::ControlCommand drive(const model::VehicleState& state, const model::PathPlanner& pathPlanner) override;

		double detectSpeed();

		double detectAngle();


	};  
}
