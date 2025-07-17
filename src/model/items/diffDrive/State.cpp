#include "State.h"
#include "model/utils/Angle.h"
#include "model/utils/Point.h"

namespace model::diffDrive {
	model::diffDrive::State::State() : 
		m_pose(DefaultStartingPose::x,DefaultStartingPose::y,DefaultStartingPose::theta),
		m_twist(0,0,0),
		m_accel(0,0,0),
		m_arch(0),
		m_velocity(0),
		m_leftWheel{ 0,0,0,0,0 },
		m_rightWheel{ 0,0,0,0,0 }
	{}
	void model::diffDrive::State::updateState(double dt)
	{
		//// TO DO: PID
		//// setControl();
		//// P*e + I*int(e) + D*der(e)
		//double e_lin = m_targetVelo.linear - model::Point(m_twist.vx, m_twist.vy).magnitude();
		//double e_ang = m_targetVelo.angular - m_twist.omega;

		setTorques();
		setRobotAccel();
		integrateToSpeed(dt);
		integrateToPosition(dt);
		setWheelAccel();
	}
	void State::setTorques()
	{
		double eps0 = 1e-5;

		m_rightWheel.pwm = 0.5;
		m_leftWheel.pwm = 0.5;
		m_rightWheel.omegaNoLoad = m_config.motor.angularVelNoLoad * m_rightWheel.pwm;
		double omegaRatio = (abs(m_rightWheel.pwm)> eps0) ? m_rightWheel.omega / m_rightWheel.omegaNoLoad: 0;
		m_rightWheel.torque = m_config.motor.torqueLocked * m_rightWheel.pwm * (1 - omegaRatio)- m_rightWheel.omega*0.001;

		m_leftWheel.omegaNoLoad = m_config.motor.angularVelNoLoad * m_leftWheel.pwm;
		omegaRatio = (abs(m_leftWheel.pwm) > eps0) ? m_leftWheel.omega / m_leftWheel.omegaNoLoad : 0;
		m_leftWheel.torque = m_config.motor.torqueLocked * m_leftWheel.pwm * (1 - omegaRatio)- m_leftWheel.omega * 0.001;
	}
	void State::setRobotAccel()
	{
		m_accel.epsilon = ((m_rightWheel.torque - m_leftWheel.torque) * m_config.vehicle.radius / m_config.wheel.radius)
			/ (m_config.vehicle.inertia + 
				(m_config.wheel.mass + m_config.wheel.inertia/(m_config.wheel.radius*m_config.wheel.radius)) 
				* 2 * m_config.vehicle.radius * m_config.vehicle.radius);



		m_linearAccel = ((m_leftWheel.torque + m_rightWheel.torque) / m_config.wheel.radius) /
			(m_config.vehicle.mass + m_config.wheel.mass * 2 + m_config.wheel.inertia/(m_config.wheel.radius * m_config.wheel.radius));
		m_accel.ax = m_linearAccel * cos(m_pose.theta);
		m_accel.ay = m_linearAccel * sin(m_pose.theta);
	}
	void State::setWheelAccel()
	{

		m_rightWheel.omega = (m_velocity + m_config.vehicle.radius*m_twist.omega)/m_config.wheel.radius;
		m_leftWheel.omega = (m_velocity - m_config.vehicle.radius * m_twist.omega) / m_config.wheel.radius;


	}
	void State::integrateToSpeed(double dt)
	{
		m_velocity += m_linearAccel * dt;
		m_twist.vx = m_velocity * cos(m_pose.theta);
		m_twist.vy = m_velocity * sin(m_pose.theta);
		m_twist.omega += m_accel.epsilon * dt;
	}
	void State::integrateToPosition(double dt)
	{
		m_pose.theta += m_twist.omega * dt;
		m_arch += m_velocity * dt;
		m_pose.x += m_velocity * cos(m_pose.theta) * dt;
		m_pose.y += m_velocity * sin(m_pose.theta) * dt;
	}

	Twist State::getVelocity() const
	{
		return m_twist;
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