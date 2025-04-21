#pragma once 
#include "IMovementModel.h"
#include <cmath> // for std::cos, std::sin, std::tan
#include "Point.h"
 
namespace model { 
class BicycleModel : public IMovementModel { 
private:
    double _speed;
    double _steeringAngle;
public:
    BicycleModel();

    void update(Point& pos, double& orientation, const double dt) override; 
    void reset() override;
    void setSpeed(double speed);
    void setSteering(double angle);
    double getSpeed() const;
    double getSteering() const;
    static const std::string type;
};
} // namespace model 
