#pragma once
#include "MotorModel.h"
#include "WheelModel.h"
namespace model::diffDrive {
    struct VehicleConstraints {
        static constexpr double length = 0.25;
        static constexpr double width = 0.25;
        static constexpr double trackWidth = 0.20;
        static constexpr double radius = trackWidth / 2;
        static constexpr double mass = 1;
        static constexpr double inertia = diskInertia(mass, width / 2);
    };

    struct VehicleConfig {
        static constexpr double scale = 5;
        static constexpr VehicleConstraints vehicle;
        static constexpr MotorModel motor;
        static constexpr WheelModel wheel;
    };
}