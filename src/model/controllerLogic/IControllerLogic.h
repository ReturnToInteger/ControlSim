#pragma once 
#include "model/VehicleState.h"
#include "model/pathPlanner/PathPlanner.h"
#include "model/controllerLogic/ControlCommand.h"
 
namespace model { 
class IControllerLogic { 
public: 
	IControllerLogic() = default;
	~IControllerLogic() = default;
	virtual model::ControlCommand drive(const model::VehicleState & state, const model::PathPlanner & pathPlanner) = 0;
 
private: 
    // Add private members here 
 
};
} // namespace controller 
