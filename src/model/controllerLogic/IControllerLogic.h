#pragma once 
#include "model/controllerLogic/ControlCommand.h"


namespace model {
	class Path;
	class VehicleState;
	namespace diffDrive {
		class State;
	}
	template <typename StateType>
	class IControllerLogic {
	public:
		IControllerLogic() = default;
		virtual ~IControllerLogic() = default;
		virtual ControlCommand drive(StateType const& state, model::Path const& path) = 0;
	};
}
