#include "State.h"
#include "model/utils/Angle.h"
#include "model/utils/Point.h"

namespace model::diffDrive {
	model::diffDrive::State::State() : 
		_pose(DefaultStartingPose::x,DefaultStartingPose::y,DefaultStartingPose::theta),
		_twist(0,0,0),
		_accel(0,0,0),
		_arch(0),
		_velocity(0),
		_leftWheel{ 0,0,0,0,0 },
		_rightWheel{ 0,0,0,0,0 }
	{}
	void model::diffDrive::State::updateState(double dt)
	{
		//// TO DO: PID
		//// _setControl();
		//// P*e + I*int(e) + D*der(e)
		//double e_lin = _targetVelo.linear - model::Point(_twist.vx, _twist.vy).magnitude();
		//double e_ang = _targetVelo.angular - _twist.omega;

		_setTorques();
		_setRobotAccel();
		_integrateToSpeed(dt);
		_integrateToPosition(dt);
		_setWheelAccel();
	}
	void State::_setTorques()
	{
		double eps0 = 1e-5;

		_rightWheel.pwm = 0.5;
		_leftWheel.pwm = 0.5;
		_rightWheel.omegaNoLoad = _config.motor.angularVelNoLoad * _rightWheel.pwm;
		//if (_rightWheel.omega > _rightWheel.omegaNoLoad) {
		//	_rightWheel.omega = _rightWheel.omegaNoLoad;
		//}
		double omegaRatio = (abs(_rightWheel.pwm)> eps0) ? model::clamp(_rightWheel.omega / _rightWheel.omegaNoLoad, -1, 1): 0;
		_rightWheel.torque = _config.motor.torqueLocked * _rightWheel.pwm * (1 - omegaRatio);

		//if (_leftWheel.omega > _leftWheel.omegaNoLoad) {
		//	_leftWheel.omega = _leftWheel.omegaNoLoad;
		//}

		_leftWheel.omegaNoLoad = _config.motor.angularVelNoLoad * _leftWheel.pwm;
		omegaRatio = (abs(_leftWheel.pwm) > eps0) ? model::clamp(_leftWheel.omega / _leftWheel.omegaNoLoad, -1, 1) : 0;
		_leftWheel.torque = _config.motor.torqueLocked * _leftWheel.pwm * (1 - omegaRatio);
	}
	void State::_setRobotAccel()
	{
		_accel.epsilon = ((_rightWheel.torque - _leftWheel.torque) * _config.vehicle.radius / _config.wheel.radius)
			/ (_config.vehicle.inertia + 
				(_config.wheel.mass + _config.wheel.inertia/(_config.wheel.radius*_config.wheel.radius)) 
				* 2 * _config.vehicle.radius * _config.vehicle.radius);



		_linearAccel = ((_leftWheel.torque + _rightWheel.torque) / _config.wheel.radius) /
			(_config.vehicle.mass + _config.wheel.mass * 2 + _config.wheel.inertia/(_config.wheel.radius * _config.wheel.radius));
		_accel.ax = _linearAccel * cos(_pose.theta);
		_accel.ay = _linearAccel * sin(_pose.theta);
	}
	void State::_setWheelAccel()
	{

		_rightWheel.omega = (_velocity + _config.vehicle.radius*_twist.omega)/_config.wheel.radius;
		_leftWheel.omega = (_velocity - _config.vehicle.radius * _twist.omega) / _config.wheel.radius;


	}
	void State::_integrateToSpeed(double dt)
	{
		_velocity += _linearAccel * dt;
		_twist.vx = _velocity * cos(_pose.theta);
		_twist.vy = _velocity * sin(_pose.theta);
		_twist.omega += _accel.epsilon * dt;
	}
	void State::_integrateToPosition(double dt)
	{
		_pose.theta += _twist.omega * dt;
		_arch += _velocity * dt;
		_pose.x += _velocity * cos(_pose.theta) * dt;
		_pose.y += _velocity * sin(_pose.theta) * dt;
	}

	Twist State::getVelocity() const
	{
		return _twist;
	}

	void model::diffDrive::State::setPose(double x, double y, Angle orientation)
	{

	}
	void State::setPose(Pose pose)
	{
	}
	double model::diffDrive::State::getMaxSpeed() const
	{
		return 0.0;
	}
}