#pragma once 
#include "model/controllerLogic/ControlCommand.h"


namespace model {
	class Path;
	class VehicleState;
	namespace diffDrive {
		class State;
	}
	class IControllerLogic {
	public:
		IControllerLogic() = default;
		virtual ~IControllerLogic() = default;
		virtual ControlCommand drive(VehicleState const& state, model::Path const& path) = 0;
		virtual ControlCommand drive(model::diffDrive::State const& state, model::Path const& path) = 0;

	};
}
