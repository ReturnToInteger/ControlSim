#pragma once 
#include "model/VehicleState.h"
#include "model/pathPlanning/PathPlanner.h"
 
namespace model {
	struct ControlCommand;
	class IControllerLogic {
	public:
		IControllerLogic() = default;
		~IControllerLogic() = default;
		virtual ControlCommand drive(const VehicleState& state, const pathPlanning::PathPlanner& pathPlanner) = 0;

	};
}
