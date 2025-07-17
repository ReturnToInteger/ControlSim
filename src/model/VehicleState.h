
#pragma once
#include <array>
#include <cmath>
#include "model/utils/Point.h"
#include "model/utils/Pose.h"
#include "model/utils/Angle.h"
#include "model/utils/ModelUtils.h"
#include "model/controllerLogic/ControlCommand.h"
#include <numbers>
#include "IVehicleState.h"

namespace model
{
	class VehicleState : public model::IVehicleState
	{
	public:
        struct DefaultStartingPosition {
            static constexpr double x = -35;
            static constexpr double y = 30;
        };
        struct DefaultConstraints {
            static constexpr double length = 3.0;  
            static constexpr double wheelBase = 2.5;  
            static constexpr double width = 1.3;  
            static constexpr double maxSteeringAngle = std::numbers::pi / 6.0;  
            static constexpr double maxSteeringRate = std::numbers::pi / 3.0;  
            static constexpr double maxSpeed = 10.0;  
            static constexpr double maxAcceleration = 3.0;  
            static constexpr double maxBrake = 10.0;  
        };
        struct Constraints {
            double length;
            double wheelBase;
            double width;
            Angle maxSteeringAngle;
            double maxSteeringRate;
            double maxSpeed;
            double maxAcceleration;
            double maxBrake;

            Constraints()
                : length(DefaultConstraints::length),
                wheelBase(DefaultConstraints::wheelBase),
                width(DefaultConstraints::width),
                maxSteeringAngle(DefaultConstraints::maxSteeringAngle),
                maxSteeringRate(DefaultConstraints::maxSteeringRate),
                maxSpeed(DefaultConstraints::maxSpeed),
                maxAcceleration(DefaultConstraints::maxAcceleration),
                maxBrake(DefaultConstraints::maxBrake) {
            }
        };


		VehicleState();
        Point getPosition() const override;
        Angle getOrientation() const override;
        Pose getPose() const override;
        double getSpeed() const override;
        Angle getSteeringAngle() const;
        double getSteeringRate() const;
        double getLength() const override;
        double getWidth() const override;
        double getMaxSpeed() const override;       
        VelocityCommand getTarget() const override;
		void setPose(double x, double y, Angle orientation) override;
        void setPose(Pose pose) override;
        void setTarget(VelocityCommand targetCommand) override;

        double getWheelBase() const;
        Angle getMaxSteeringAngle() const;
        double getMaxSteeringRate() const;
        std::array<Point, 3> getAllPositions() const;
        std::array<Angle, 3> getAllOrientations() const;
        Pose getRearPose() const;
        Pose getFrontPose() const;

        // Update based on state
        void updateState(double dt) override;

    private:

        //states  
        Pose m_centerPose;
        Pose m_frontPose;
        Pose m_rearPose;
        Twist m_centerTwist;
        double m_speed;
        double m_acceleration;
        double m_brakeAcceleration;
        Angle m_steeringAngle;
        double m_steeringRate;
        //parameters and constraints
        Constraints m_constraints;
       //targets
        double m_targetSpeed = 0.0;
        double m_targetSteeringAngle = 0.0;
        VelocityCommand m_target;

		void updateControl(double dt);
        void updateSteering(double dt);
        void updateDriving(double dt);
		void updateCoords(double speed,double dt);
        void updateCenter(double speed, double dt, Angle slip);
        void updateFront(double dt, Angle slip);
        void updateRear(double dt, Angle slip);

        void setSteeringAngle(Angle angle);
        void setSteeringRate(double rate);
        void setSpeed(double speed);
        void setAcceleration(double acceleration, double brake);

    public:



        // Inherited via IVehicleState
        Twist getVelocity() const override;

        double minimumTurningRadius() const override;


        // Inherited via IVehicleState
        std::unique_ptr<IVehicleState> clone() const override;

};

    inline Pose VehicleState::getPose() const { return m_centerPose; }
    inline Pose VehicleState::getRearPose() const { return m_rearPose; }
    inline Pose VehicleState::getFrontPose() const { return m_frontPose; }
    inline std::array<Point, 3> VehicleState::getAllPositions() const
    {
        return { Point(m_centerPose.x,m_centerPose.y), Point(m_frontPose.x,m_frontPose.y),Point(m_rearPose.x,m_rearPose.y) };
    }
    inline std::array<Angle, 3> VehicleState::getAllOrientations() const
    {
        return { m_centerPose.theta,m_frontPose.theta,m_rearPose.theta };
    }
    inline Point VehicleState::getPosition() const
    {
        return Point(m_centerPose.x, m_centerPose.y);
    }
    inline Angle VehicleState::getOrientation() const { return m_centerPose.theta; }
    inline double VehicleState::getSpeed() const { return m_speed; }
    inline Angle VehicleState::getSteeringAngle() const { return m_steeringAngle; }
    inline double VehicleState::getSteeringRate() const { return m_steeringRate; }
    inline double VehicleState::getLength() const { return m_constraints.length; }
    inline double VehicleState::getWidth() const { return m_constraints.width; }
    inline double VehicleState::getWheelBase() const { return m_constraints.wheelBase; }
    inline Angle VehicleState::getMaxSteeringAngle() const { return m_constraints.maxSteeringAngle; }
    inline double VehicleState::getMaxSteeringRate() const { return m_constraints.maxSteeringRate; }
    inline double VehicleState::getMaxSpeed() const { return m_constraints.maxSpeed; }
    inline void VehicleState::setTarget(VelocityCommand targetCommand)
    {
        m_target = targetCommand;
    }
    inline VelocityCommand VehicleState::getTarget() const { return m_target; }
}
