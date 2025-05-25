#pragma once 
#include "src/model/VehicleState.h"
#include "src/model/pathPlanner/PathPlanner.h"
 
namespace model { 
class IControllerLogic { 
public: 
	IControllerLogic() = default;
	~IControllerLogic() = default;
	virtual void drive(model::VehicleState& state, model::PathPlanner& pathPlanner) = 0;
 
private: 
    // Add private members here 
 
};
} // namespace controller 
