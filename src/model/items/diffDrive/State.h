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
        double minimumTurningRadius() const override { return m_config.vehicle.radius; }
        std::unique_ptr<IVehicleState> clone() const override { return std::make_unique<State>(*this); }

        void setPose(double x, double y, Angle orientation) override;
        void setPose(Pose pose) override;
        void setTarget(model::VelocityCommand targetCommand) override;

        // Update based on state
        void updateState(double dt) override;

    private:

        //states  
        Pose m_pose;
        Twist m_twist;
        Accel m_accel;
        double m_arch;
        double m_velocity;
        double m_linearAccel;
        double m_angularAccel;
        //parameters and constraints
        VehicleConfig m_config;
        //targets
        VelocityCommand m_targetVelo;
        WheelState m_leftWheel;
        WheelState m_rightWheel;
        void setRobotAccel();
        void setWheelAccel();
        void setTorques();
        void integrateToSpeed(double dt);
        void integrateToPosition(double dt);

    };

    inline Pose State::getPose() const { return m_pose; }
    inline Point State::getPosition() const
    {
        return Point(m_pose.x, m_pose.y);
    }
    inline Angle State::getOrientation() const { return m_pose.theta; }
    inline double State::getSpeed() const { return m_velocity; }
    inline double State::getLength() const { return m_config.vehicle.length; }
    inline double State::getWidth() const { return m_config.vehicle.width; }
    inline void State::setTarget(VelocityCommand targetCommand)
    {
        m_targetVelo.linear = model::clamp(targetCommand.linear, -1, 1);
        m_targetVelo.angular = model::clamp(targetCommand.angular, -1, 1);
    }
    inline VelocityCommand State::getTarget() const { return m_targetVelo; }
}
