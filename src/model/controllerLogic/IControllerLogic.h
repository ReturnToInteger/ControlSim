#pragma once 
#include "model/controllerLogic/ControlCommand.h"

namespace model {
	namespace pathPlanning {
		class PathPlanner;
	}
	class VehicleState;
	class IControllerLogic {
	public:
		IControllerLogic() = default;
		~IControllerLogic() = default;
		virtual ControlCommand drive(VehicleState const& state, pathPlanning::PathPlanner const& pathPlanner) = 0;

	};
}
