#pragma once  
#include "Controller.h"  


namespace controller  
{  
class KeyboardControl :  
	public Controller  
{  
	// Inherited via Controller  
	void drive(model::VehicleState& state) override;  

};  
}
