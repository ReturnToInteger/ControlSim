#pragma once 
#include "Point.h" 
#include <string>
 
namespace model { 
class IMovementModel { 
public: 
    virtual ~IMovementModel() = default;
    virtual void update(Point& pos, double& orientation, const double dt) = 0;
	virtual void reset() = 0;
 	static const std::string type;
};
} // namespace model 
