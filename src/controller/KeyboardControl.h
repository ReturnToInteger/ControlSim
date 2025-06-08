#pragma once  
#include "model/controllerLogic/IControllerLogic.h"  
#include <model/controllerLogic/ControlCommand.h>



namespace controller  
{
	class model::VehicleState;
	class model::pathPlanning::PathPlanner;
	class KeyboardControl :  
		public model::IControllerLogic  
	{  
		// Inherited via IControllerLogic  
		model::ControlCommand drive(model::VehicleState const& state, model::pathPlanning::PathPlanner const& pathPlanner) override;

		double detectSpeed();

		double detectAngle();


	};  
}
