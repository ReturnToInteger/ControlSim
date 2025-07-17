#include "VehicleState.h"
#include <cmath>
#include <iostream>

namespace model
{
	model::VehicleState::VehicleState() : 
		m_frontPose(DefaultStartingPosition::x+ DefaultConstraints::wheelBase / 2, DefaultStartingPosition::y,0),
		m_centerPose(DefaultStartingPosition::x, DefaultStartingPosition::y,0),
		m_rearPose(DefaultStartingPosition::x - DefaultConstraints::wheelBase / 2, DefaultStartingPosition::y,0),
		m_centerTwist(0,0,0),
		m_speed(0),
		m_steeringAngle(0),
		m_steeringRate(0),
		m_acceleration(0),
		m_target(0,0),
		m_brakeAcceleration(0)
	{
	}

	void model::VehicleState::setPose(double x, double y, Angle orientation)
	{
		m_centerPose.x = x;
		m_centerPose.y = y;
		m_centerPose.theta = orientation;
		m_frontPose.x = m_centerPose.x + m_constraints.wheelBase / 2 * cos(m_centerPose.theta);
		m_frontPose.y = m_centerPose.y + m_constraints.wheelBase / 2 * sin(m_centerPose.theta);
		m_frontPose.theta = m_centerPose.theta + m_steeringAngle;
		m_rearPose.x = m_centerPose.x - m_constraints.wheelBase / 2 * cos(m_centerPose.theta);
		m_rearPose.y = m_centerPose.y - m_constraints.wheelBase / 2 * sin(m_centerPose.theta);
		m_rearPose.theta = m_centerPose.theta;
	}

	void model::VehicleState::setPose(Pose pose)
	{
		setPose(pose.x, pose.y, pose.theta);
	}

	void model::VehicleState::updateState(double dt) {
		updateControl(dt);
		updateCoords(m_speed, dt);
	}


	void model::VehicleState::updateControl(double dt)
	{
		updateSteering(dt);
		updateDriving(dt);
	}

	void model::VehicleState::updateSteering(double dt)
	{
		// Idealistic, set by interpolating (instead of control)
		Angle delta = m_target.angular*getMaxSteeringAngle() - m_steeringAngle;
		double maxStep = m_constraints.maxSteeringRate * dt;
		delta = clampRelativeToZero(delta, maxStep);
		setSteeringRate(radian(delta) / dt); 
		setSteeringAngle(m_steeringAngle + delta);

		// P control
		//setSteeringRate((m_target.steeringAngle - m_steeringAngle) / dt);
		//setSteeringAngle(m_steeringAngle + m_steeringRate * dt);
	}

	void model::VehicleState::updateDriving(double dt)
	{
		setAcceleration((m_target.linear*getMaxSpeed() - m_speed) / dt, 0);
		setSpeed(m_speed + m_acceleration * dt);
	}

	void model::VehicleState::updateCoords(double speed, double dt)
	{
		//Angle beta(std::numbers::pi/2);
		Angle slip = atan(tan(m_steeringAngle) / 2);
		updateCenter(speed, dt, slip);
		updateFront(dt, slip);
		updateRear(dt, slip);
	}

	void model::VehicleState::updateCenter(double speed, double dt, Angle slip)
	{
		m_centerTwist.omega = speed * tan(m_steeringAngle) * cos(slip) / m_constraints.wheelBase;
		m_centerPose.theta = m_centerPose.theta + m_centerTwist.omega * dt;
		m_centerTwist.vx = speed * cos(m_centerPose.theta + slip);
		m_centerTwist.vy = speed * sin(m_centerPose.theta + slip);
		m_centerPose.x += dt * m_centerTwist.vx;
		m_centerPose.y += dt * m_centerTwist.vy;
	}

	void model::VehicleState::updateFront(double dt, Angle slip)
	{
		m_frontPose.x = m_centerPose.x + m_constraints.wheelBase / 2 * cos(m_centerPose.theta);
		m_frontPose.y = m_centerPose.y + m_constraints.wheelBase / 2 * sin(m_centerPose.theta);
		m_frontPose.theta = m_centerPose.theta + m_steeringAngle;
	}

	void model::VehicleState::updateRear(double dt, Angle slip)
	{
		m_rearPose.x = m_centerPose.x - m_constraints.wheelBase / 2 * cos(m_centerPose.theta);
		m_rearPose.y = m_centerPose.y - m_constraints.wheelBase / 2 * sin(m_centerPose.theta);
		m_rearPose.theta = m_centerPose.theta;
	}


	void model::VehicleState::setSteeringAngle(Angle angle)
	{
		m_steeringAngle = clampRelativeToZero(angle, m_constraints.maxSteeringAngle);
		//std::cout << "steering angle: " << m_steeringAngle << "\n";
	}

	void model::VehicleState::setSteeringRate(double rate)
	{
		m_steeringRate = clamp(rate, -m_constraints.maxSteeringRate, m_constraints.maxSteeringRate);
		//std::cout << "steering rate: " << m_steeringRate << "\n";
	}

	void model::VehicleState::setSpeed(double speed)
	{
		m_speed = clamp(speed, -m_constraints.maxSpeed, m_constraints.maxSpeed);
		//std::cout << "speed: " << m_speed << "\n";
	}

	void model::VehicleState::setAcceleration(double acceleration, double brake)
	{


		m_acceleration = clamp(acceleration, -m_constraints.maxAcceleration, m_constraints.maxAcceleration);
		if (m_speed > 0) {
			m_acceleration -= brake;
		}
		else if (m_speed < 0) {
			m_acceleration += brake;
		}
		//std::cout << "acceleration: " << m_acceleration << "\n";
		/* if (m_speed == 0)
		{
			m_acceleration = ;
		}*/
	}

	Twist VehicleState::getVelocity() const
	{
		return m_centerTwist;
	}

	double VehicleState::minimumTurningRadius() const
	{
		return 0.0;
	}

	std::unique_ptr<IVehicleState> VehicleState::clone() const
	{
		 return std::make_unique<VehicleState>(*this);
	}

}