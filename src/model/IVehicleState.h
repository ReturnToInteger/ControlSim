#pragma once
#include "model/utils/Point.h"
#include "model/utils/Angle.h"
#include "model/utils/Pose.h"
#include "model/utils/ModelUtils.h"
#include <memory>

namespace model {
	class IVehicleState {
    public:
        struct Velocity {
            double linear;
            double angular;
        };
        virtual model::Point getPosition() const = 0;
        virtual model::Angle getOrientation() const = 0;
        virtual void setPose(double x, double y, Angle orientation) = 0;
        virtual void setPose(Pose pose) = 0;
        virtual model::Pose getPose() const = 0;
        virtual double getSpeed() const = 0;
        virtual double getLength() const = 0;
        virtual double getWidth() const = 0;
        virtual double getMaxSpeed() const = 0;
        virtual Twist getVelocity() const = 0;
        virtual void setTarget(VelocityCommand targetCommand) = 0;
        virtual VelocityCommand getTarget() const = 0;
        virtual double minimumTurningRadius() const = 0;
        virtual std::unique_ptr<IVehicleState> clone() const = 0;


        // Update based on state
        virtual void updateState(double dt) = 0;
        virtual ~IVehicleState() = default;
	};
}