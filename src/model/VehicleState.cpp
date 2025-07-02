#include "VehicleState.h"
#include <cmath>
#include <iostream>

namespace model
{
	VehicleState::VehicleState() : 
		_frontPose(DefaultStartingPosition::x+ DefaultConstraints::wheelBase / 2, DefaultStartingPosition::y,0),
		_centerPose(DefaultStartingPosition::x, DefaultStartingPosition::y,0),
		_rearPose(DefaultStartingPosition::x - DefaultConstraints::wheelBase / 2, DefaultStartingPosition::y,0),
		_centerTwist(0,0,0),
		_speed(0),
		_steeringAngle(0),
		_steeringRate(0),
		_acceleration(0),
		_target(0,0),
		_brakeAcceleration(0)
	{
	}

	void model::VehicleState::setPose(double x, double y, Angle orientation)
	{
		_centerPose.x = x;
		_centerPose.y = y;
		_centerPose.theta = orientation;
		_frontPose.x = _centerPose.x + _constraints.wheelBase / 2 * cos(_centerPose.theta);
		_frontPose.y = _centerPose.y + _constraints.wheelBase / 2 * sin(_centerPose.theta);
		_frontPose.theta = _centerPose.theta + _steeringAngle;
		_rearPose.x = _centerPose.x - _constraints.wheelBase / 2 * cos(_centerPose.theta);
		_rearPose.y = _centerPose.y - _constraints.wheelBase / 2 * sin(_centerPose.theta);
		_rearPose.theta = _centerPose.theta;
	}

	void VehicleState::setPose(Pose pose)
	{
		setPose(pose.x, pose.y, pose.theta);
	}

	void VehicleState::updateState(double dt) {
		_updateControl(dt);
		_updateCoords(_speed, dt);
	}


	void VehicleState::_updateControl(double dt)
	{
		_updateSteering(dt);
		_updateDriving(dt);
	}

	void model::VehicleState::_updateSteering(double dt)
	{
		// Idealistic, set by interpolating (instead of control)
		Angle delta = _target.normSteering*getMaxSteeringAngle() - _steeringAngle;
		double maxStep = _constraints.maxSteeringRate * dt;
		delta = clampRelativeToZero(delta, maxStep);
		_setSteeringRate(radian(delta) / dt); 
		_setSteeringAngle(_steeringAngle + delta);

		// P control
		//_setSteeringRate((_target.steeringAngle - _steeringAngle) / dt);
		//_setSteeringAngle(_steeringAngle + _steeringRate * dt);
	}

	void VehicleState::_updateDriving(double dt)
	{
		_setAcceleration((_target.normSpeed*getMaxSpeed() - _speed) / dt, 0);
		_setSpeed(_speed + _acceleration * dt);
	}

	void VehicleState::_updateCoords(double speed, double dt)
	{
		//Angle beta(std::numbers::pi/2);
		Angle slip = atan(tan(_steeringAngle) / 2);
		_updateCenter(speed, dt, slip);
		_updateFront(dt, slip);
		_updateRear(dt, slip);
	}

	void model::VehicleState::_updateCenter(double speed, double dt, Angle slip)
	{
		_centerTwist.omega = speed * tan(_steeringAngle) * cos(slip) / _constraints.wheelBase;
		_centerPose.theta = _centerPose.theta + _centerTwist.omega * dt;
		_centerTwist.vx = speed * cos(_centerPose.theta + slip);
		_centerTwist.vy = speed * sin(_centerPose.theta + slip);
		_centerPose.x += dt * _centerTwist.vx;
		_centerPose.y += dt * _centerTwist.vy;
	}

	void model::VehicleState::_updateFront(double dt, Angle slip)
	{
		_frontPose.x = _centerPose.x + _constraints.wheelBase / 2 * cos(_centerPose.theta+slip);
		_frontPose.y = _centerPose.y + _constraints.wheelBase / 2 * sin(_centerPose.theta+slip);
		_frontPose.theta = _centerPose.theta + _steeringAngle;
	}

	void model::VehicleState::_updateRear(double dt, Angle slip)
	{
		_rearPose.x = _centerPose.x - _constraints.wheelBase / 2 * cos(_centerPose.theta-slip);
		_rearPose.y = _centerPose.y - _constraints.wheelBase / 2 * sin(_centerPose.theta-slip);
		_rearPose.theta = _centerPose.theta;
	}



	void VehicleState::_setSteeringAngle(Angle angle)
	{
		_steeringAngle = clampRelativeToZero(angle, _constraints.maxSteeringAngle);
		//std::cout << "steering angle: " << _steeringAngle << "\n";
	}

	void VehicleState::_setSteeringRate(double rate)
	{
		_steeringRate = clamp(rate, -_constraints.maxSteeringRate, _constraints.maxSteeringRate);
		//std::cout << "steering rate: " << _steeringRate << "\n";
	}

	void VehicleState::_setSpeed(double speed)
	{
		_speed = clamp(speed, -_constraints.maxSpeed, _constraints.maxSpeed);
		//std::cout << "speed: " << _speed << "\n";
	}

	void VehicleState::_setAcceleration(double acceleration, double brake)
	{


		_acceleration = clamp(acceleration, -_constraints.maxAcceleration, _constraints.maxAcceleration);
		if (_speed > 0) {
			_acceleration -= brake;
		}
		else if (_speed < 0) {
			_acceleration += brake;
		}
		//std::cout << "acceleration: " << _acceleration << "\n";
		/* if (_speed == 0)
		{
			_acceleration = ;
		}*/
	}

}