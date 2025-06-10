#pragma once  
#include "model/controllerLogic/IControllerLogic.h"  
#include <model/controllerLogic/ControlCommand.h>


namespace model {
	class VehicleState;
	class Path;
}
namespace controller  
{
	class KeyboardControl :  
		public model::IControllerLogic  
	{ 
	public:
		// Inherited via IControllerLogic  
		model::ControlCommand drive(model::VehicleState const& state, model::Path const& path) override;
		~KeyboardControl() = default;
		double detectSpeed();

		double detectAngle();


	};  
}
