#pragma once
#include "model/utils/Point.h"
#include "model/utils/Angle.h"
namespace model {
    class IDrawableVehicle {
    public:
        virtual ~IDrawableVehicle() = default;

        virtual model::Point getPosition() const = 0;
        virtual model::Angle getOrientation() const = 0;
        virtual double getLength() const = 0;
        virtual double getWidth() const = 0;
    };
}