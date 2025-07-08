#pragma once
#include <array>
#include <cmath>
#include "model/utils/Point.h"
#include "model/utils/Pose.h"
#include "model/utils/Angle.h"
#include "model/utils/ModelUtils.h"
#include "model/controllerLogic/ControlCommand.h"
#include <numbers>
#include "VehicleConfig.h"
#include "model/IVehicleState.h"
#include "model/utils/IDrawableVehicle.h"


namespace model::diffDrive 
{
    struct DiffInput {
        double linearVel;
        double angularVel;
    };
    struct WheelState {
        double omega;
        double epsilon;
        double omegaNoLoad;
        double torque;
        double pwm;
    };
    class State : public model::IVehicleState, public IDrawableVehicle
    {
    public:
        struct DefaultStartingPose {
            static constexpr double x = -35;
            static constexpr double y = 30;
            static constexpr double theta = 0;
        };


        State();
        model::Point getPosition() const override;
        model::Angle getOrientation() const override;
        model::Pose getPose() const override;
        double getSpeed() const override;
        double getLength() const override;
        double getWidth() const override;
        double getMaxSpeed() const override;
        Twist getVelocity() const override;
        model::VelocityCommand getTarget() const override;
        double minimumTurningRadius() const override { return _config.vehicle.radius; }
        std::unique_ptr<IVehicleState> clone() const override { return std::make_unique<State>(*this); }

        void setPose(double x, double y, Angle orientation) override;
        void setPose(Pose pose) override;
        void setTarget(model::VelocityCommand targetCommand) override;

        // Update based on state
        void updateState(double dt) override;

    private:

        //states  
        Pose _pose;
        Twist _twist;
        Accel _accel;
        double _arch;
        double _velocity;
        double _linearAccel;
        double _angularAccel;
        //parameters and constraints
        VehicleConfig _config;
        //targets
        VelocityCommand _targetVelo;
        WheelState _leftWheel;
        WheelState _rightWheel;
        void _setRobotAccel();
        void _setWheelAccel();
        void _setTorques();
        void _integrateToSpeed(double dt);
        void _integrateToPosition(double dt);

    };

    inline Pose State::getPose() const { return _pose; }
    inline Point State::getPosition() const
    {
        return Point(_pose.x, _pose.y);
    }
    inline Angle State::getOrientation() const { return _pose.theta; }
    inline double State::getSpeed() const { return _velocity; }
    inline double State::getLength() const { return _config.vehicle.length; }
    inline double State::getWidth() const { return _config.vehicle.width; }
    inline void State::setTarget(VelocityCommand targetCommand)
    {
        _targetVelo.linear = model::clamp(targetCommand.linear, -1, 1);
        _targetVelo.angular = model::clamp(targetCommand.angular, -1, 1);
    }
    inline VelocityCommand State::getTarget() const { return _targetVelo; }
}
