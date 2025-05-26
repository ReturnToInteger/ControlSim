#pragma once 
#include "src/model/VehicleState.h"
#include "src/model/pathPlanner/PathPlanner.h"
#include "src/model/controllerLogic/ControlCommand.h"
 
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
