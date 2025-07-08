
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
        Pose _centerPose;
        Pose _frontPose;
        Pose _rearPose;
        Twist _centerTwist;
        double _speed;
        double _acceleration;
        double _brakeAcceleration;
        Angle _steeringAngle;
        double _steeringRate;
        //parameters and constraints
        Constraints _constraints;
       //targets
        double _targetSpeed = 0.0;
        double _targetSteeringAngle = 0.0;
        VelocityCommand _target;

		void _updateControl(double dt);
        void _updateSteering(double dt);
        void _updateDriving(double dt);
		void _updateCoords(double speed,double dt);
        void _updateCenter(double speed, double dt, Angle slip);
        void _updateFront(double dt, Angle slip);
        void _updateRear(double dt, Angle slip);

        void _setSteeringAngle(Angle angle);
        void _setSteeringRate(double rate);
        void _setSpeed(double speed);
        void _setAcceleration(double acceleration, double brake);

    public:



        // Inherited via IVehicleState
        Twist getVelocity() const override;

        double minimumTurningRadius() const override;


        // Inherited via IVehicleState
        std::unique_ptr<IVehicleState> clone() const override;

};

    inline Pose VehicleState::getPose() const { return _centerPose; }
    inline Pose VehicleState::getRearPose() const { return _rearPose; }
    inline Pose VehicleState::getFrontPose() const { return _frontPose; }
    inline std::array<Point, 3> VehicleState::getAllPositions() const
    {
        return { Point(_centerPose.x,_centerPose.y), Point(_frontPose.x,_frontPose.y),Point(_rearPose.x,_rearPose.y) };
    }
    inline std::array<Angle, 3> VehicleState::getAllOrientations() const
    {
        return { _centerPose.theta,_frontPose.theta,_rearPose.theta };
    }
    inline Point VehicleState::getPosition() const
    {
        return Point(_centerPose.x, _centerPose.y);
    }
    inline Angle VehicleState::getOrientation() const { return _centerPose.theta; }
    inline double VehicleState::getSpeed() const { return _speed; }
    inline Angle VehicleState::getSteeringAngle() const { return _steeringAngle; }
    inline double VehicleState::getSteeringRate() const { return _steeringRate; }
    inline double VehicleState::getLength() const { return _constraints.length; }
    inline double VehicleState::getWidth() const { return _constraints.width; }
    inline double VehicleState::getWheelBase() const { return _constraints.wheelBase; }
    inline Angle VehicleState::getMaxSteeringAngle() const { return _constraints.maxSteeringAngle; }
    inline double VehicleState::getMaxSteeringRate() const { return _constraints.maxSteeringRate; }
    inline double VehicleState::getMaxSpeed() const { return _constraints.maxSpeed; }
    inline void VehicleState::setTarget(VelocityCommand targetCommand)
    {
        _target = targetCommand;
    }
    inline VelocityCommand VehicleState::getTarget() const { return _target; }
}
