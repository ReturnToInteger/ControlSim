#pragma once
#include "VehicleState.h"
#include <cmath>
#include <iostream>

namespace model
{
	VehicleState::VehicleState() :_length(DEF_LENGTH),
		_width(DEF_WIDTH),
		_front{ DEF_LENGTH / 2,0,0,0,0,0 },
		_center{ 0,0,0,0,0,0 },
		_rear{ -DEF_LENGTH / 2,0,0,0,0,0 },
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
		return { Point(_center[0],_center[1]), Point(_front[0],_front[1]),Point(_rear[0],_rear[1]) };
	}

	std::array<double, 3> VehicleState::getAllOrientations() const
	{
		return { _center[2],_front[2],_rear[2] };
	}

	Point VehicleState::getPosition() const
	{
		return Point(_center[0], _center[1]);
	}

	double VehicleState::getOrientation() const
	{
		return _center[2];
	}

	std::array<double, 6> VehicleState::getCenter() const
	{
		return _center;
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

		_updateSteering(dt);
		_updateDriving(dt);
		_updateCenter(dt);
		_updateFront(dt);
		_updateRear(dt);
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

	void VehicleState::_updateCenter(double dt)
	{
		double beta = atan(tan(_steeringAngle) / 2);
		_center[5] = _speed * tan(_steeringAngle) * cos(beta) / _length;
		_center[2] = _validateOrientation(_center[2] + _center[5] * dt);
		_center[3] = _speed * cos(_center[2] + beta);
		_center[4] = _speed * sin(_center[2] + beta);
		_center[0] += dt * _center[3];
		_center[1] += dt * _center[4];
	}

	void VehicleState::_updateFront(double dt)
	{
		_front[0] = _center[0] + _length / 2 * cos(_center[2]);
		_front[1] = _center[1] + _length / 2 * sin(_center[2]);
		_front[2] = _center[2] + _steeringAngle;
		_front[3] = 0;
		_front[4] = 0;
		_front[5] = 0;
	}

	void VehicleState::_updateRear(double dt)
	{
		_rear[0] = _center[0] - _length / 2 * cos(_center[2]);
		_rear[1] = _center[1] - _length / 2 * sin(_center[2]);
		_rear[2] = _center[2];
		_rear[3] = 0;
		_rear[4] = 0;
		_rear[5] = 0;
	}


	double VehicleState::_validateOrientation(const double& orientation) const
	{
		double a = std::fmod(orientation + M_PI, 2 * M_PI);
		if (a < 0) a += 2 * M_PI;
		return a - M_PI;
	}

	void VehicleState::_setSteeringAngle(double angle)
	{
		_steeringAngle = clamp(angle, -_maxSteeringAngle, _maxSteeringAngle);
		std::cout << "steering angle: " << _steeringAngle << std::endl;
	}

	void VehicleState::_setSteeringRate(double rate)
	{
		_steeringRate = clamp(rate, -_maxSteeringRate, _maxSteeringRate);
		std::cout << "steering rate: " << _steeringRate << std::endl;
	}

	void VehicleState::_setSpeed(double speed)
	{
		_speed = clamp(speed, -_maxSpeed, _maxSpeed);
		std::cout << "speed: " << _speed << std::endl;
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
		std::cout << "acceleration: " << _acceleration << std::endl;
		/* if (_speed == 0)
		{
			_acceleration = ;
		}*/
	}

}