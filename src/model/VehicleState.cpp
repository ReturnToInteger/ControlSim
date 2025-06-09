#pragma once
#include "VehicleState.h"
#include <cmath>
#include <iostream>

namespace model
{
	VehicleState::VehicleState() :_length(DEF_LENGTH),
		_width(DEF_WIDTH),
		_wheelBase(DEF_WHEELBASE),
		_frontPose(-35+DEF_WHEELBASE / 2, 30,0),
		_centerPose( -35,30,0),
		_rearPose(-35 -DEF_WHEELBASE / 2, 30,0),
		_centerTwist(0,0,0),
		_speed(0),
		_steeringAngle(0),
		_steeringRate(0),
		_acceleration(0),
		_targetSpeed(0),
		_targetSteeringAngle(0),
		_brakeAcceleration(0)
	{
	}
	std::array<Point, 3> VehicleState::getAllPositions() const
	{
		return { Point(_centerPose.x,_centerPose.y), Point(_frontPose.x,_frontPose.y),Point(_rearPose.x,_rearPose.y) };
	}

	std::array<double, 3> VehicleState::getAllOrientations() const
	{
		return { _centerPose.theta,_frontPose.theta,_rearPose.theta };
	}

	Point VehicleState::getPosition() const
	{
		return Point(_centerPose.x, _centerPose.y);
	}

	double VehicleState::getOrientation() const
	{
		return _centerPose.theta;
	}

	void model::VehicleState::setPose(double x, double y, double orientation)
	{
		_centerPose.x = x;
		_centerPose.y = y;
		_centerPose.theta = model::normAngle(orientation);
		_frontPose.x = _centerPose.x + _wheelBase / 2 * cos(_centerPose.theta);
		_frontPose.y = _centerPose.y + _wheelBase / 2 * sin(_centerPose.theta);
		_frontPose.theta = _centerPose.theta + _steeringAngle;
		_rearPose.x = _centerPose.x - _wheelBase / 2 * cos(_centerPose.theta);
		_rearPose.y = _centerPose.y - _wheelBase / 2 * sin(_centerPose.theta);
		_rearPose.theta = _centerPose.theta;
	}

	void VehicleState::setTargetSpeed(double input)
	{
		_targetSpeed = clamp(input, -1, 1)*_maxSpeed;
	}
	void VehicleState::setTargetSteeringAngle(double input)
	{
		_targetSteeringAngle = clamp(input, -1, 1) * _maxSteeringAngle;
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
		_setSteeringRate((_targetSteeringAngle - _steeringAngle) / dt);
		_setSteeringAngle(_steeringAngle + _steeringRate * dt);
	}

	void VehicleState::_updateDriving(double dt)
	{
		_setAcceleration((_targetSpeed - _speed) / dt, 0);
		_setSpeed(_speed + _acceleration * dt);
	}

	void VehicleState::_updateCoords(double speed, double dt)
	{
		//Angle beta(M_PI/2);
		Angle slip = atan(tan(_steeringAngle) / 2);
		_updateCenter(speed, dt, slip);
		_updateFront(dt, slip);
		_updateRear(dt, slip);
	}

	void model::VehicleState::_updateCenter(double speed, double dt, Angle slip)
	{
		_centerTwist.omega = speed * tan(_steeringAngle) * cos(slip) / _wheelBase;
		_centerPose.theta = _centerPose.theta + _centerTwist.omega * dt;
		_centerTwist.vx = speed * cos(_centerPose.theta + slip);
		_centerTwist.vy = speed * sin(_centerPose.theta + slip);
		_centerPose.x += dt * _centerTwist.vx;
		_centerPose.y += dt * _centerTwist.vy;
	}

	void model::VehicleState::_updateFront(double dt, Angle slip)
	{
		_frontPose.x = _centerPose.x + _wheelBase / 2 * cos(_centerPose.theta+slip);
		_frontPose.y = _centerPose.y + _wheelBase / 2 * sin(_centerPose.theta+slip);
		_frontPose.theta = _centerPose.theta + _steeringAngle;
	}

	void model::VehicleState::_updateRear(double dt, Angle slip)
	{
		_rearPose.x = _centerPose.x - _wheelBase / 2 * cos(_centerPose.theta-slip);
		_rearPose.y = _centerPose.y - _wheelBase / 2 * sin(_centerPose.theta-slip);
		_rearPose.theta = _centerPose.theta;
	}



	void VehicleState::_setSteeringAngle(double angle)
	{
		_steeringAngle = clamp(angle, -_maxSteeringAngle, _maxSteeringAngle);
		//std::cout << "steering angle: " << _steeringAngle << std::endl;
	}

	void VehicleState::_setSteeringRate(double rate)
	{
		_steeringRate = clamp(rate, -_maxSteeringRate, _maxSteeringRate);
		//std::cout << "steering rate: " << _steeringRate << std::endl;
	}

	void VehicleState::_setSpeed(double speed)
	{
		_speed = clamp(speed, -_maxSpeed, _maxSpeed);
		//std::cout << "speed: " << _speed << std::endl;
	}

	void VehicleState::_setAcceleration(double acceleration, double brake)
	{


		_acceleration = clamp(acceleration, -_maxAcceleration, _maxAcceleration);
		if (_speed > 0) {
			_acceleration -= brake;
		}
		else if (_speed < 0) {
			_acceleration += brake;
		}
		//std::cout << "acceleration: " << _acceleration << std::endl;
		/* if (_speed == 0)
		{
			_acceleration = ;
		}*/
	}

}