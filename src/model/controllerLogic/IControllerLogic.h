#pragma once 
#include "model/controllerLogic/ControlCommand.h"

namespace model {
	class Path;
	class VehicleState;
	class IControllerLogic {
	public:
		IControllerLogic() = default;
		virtual ~IControllerLogic() = default;
		virtual ControlCommand drive(VehicleState const& state, model::Path const& path) = 0;

	};
}
