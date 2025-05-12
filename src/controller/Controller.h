#pragma once 
#include "src/model/VehicleState.h"
 
namespace controller { 
class Controller { 
public: 
	Controller() = default;
	~Controller() = default;
	virtual void drive(model::VehicleState& state) = 0;
 
private: 
    // Add private members here 
 
};
} // namespace controller 
