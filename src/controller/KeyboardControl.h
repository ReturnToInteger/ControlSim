#pragma once  
#include "model/controllerLogic/IControllerLogic.h"  
#include <model/controllerLogic/ControlCommand.h>


namespace model {
	class VehicleState;
	namespace pathPlanning {
		class PathPlanner;
	}
}
namespace controller  
{
	class KeyboardControl :  
		public model::IControllerLogic  
	{  
		// Inherited via IControllerLogic  
		model::ControlCommand drive(model::VehicleState const& state, model::pathPlanning::PathPlanner const& pathPlanner) override;

		double detectSpeed();

		double detectAngle();


	};  
}
