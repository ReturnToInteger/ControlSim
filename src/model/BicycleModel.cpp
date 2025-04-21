#include "BicycleModel.h" 
 
namespace model {
	BicycleModel::BicycleModel() : _speed(1), _steeringAngle(0) {}
	
	void BicycleModel::update(Point& pos, double& orientation, const double dt) {
        // simple bicycle model update
        pos += Point(_speed * cos(orientation) * dt, _speed * sin(orientation) * dt);
        orientation += _speed * tan(_steeringAngle) * dt;  // simplified
    }
	void BicycleModel::reset() {
		_speed = 0;
		_steeringAngle = 0;
	}

	void BicycleModel::setSpeed(double speed) { _speed = speed; }
	void BicycleModel::setSteering(double angle) { _steeringAngle = angle; }
	double BicycleModel::getSpeed() const { return _speed; }
	double BicycleModel::getSteering() const { return _steeringAngle; }
	const std::string BicycleModel::type = "BicycleModel";
}