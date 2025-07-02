#pragma once
#include <cmath>
#include <ostream>
#include "Angle.h"

namespace model {
    struct Pose {
        double x;
        double y;
        Angle theta;
        Pose(double x, double y, Angle theta) : x(x), y(y), theta(theta) {}
        //Pose(model::Point p, double theta) : x(p.X()), y(p.Y()), theta(theta) {}
        Pose() : x(0), y(0), theta(0) {}

        double magnitude() const
        {
            return std::sqrt(x * x + y * y);
        }


        Pose operator+(Pose const& other) const
        {
            return Pose(x + other.x, y + other.y, theta + other.theta);
        }
        Pose operator-(Pose const& other) const
        {
            return Pose(x - other.x, y - other.y, theta - other.theta);
        }
        double operator*(Pose const& other) const
        {
            return x * other.x + y * other.y;
        }


        Pose& operator+=(Pose const& other)
        {
            x += other.x;
            y += other.y;
            theta += other.theta;
            return *this;
        }

        bool operator==(Pose const& other) const
        {
            double dE = 1e-6;
            return abs(x - other.x) < dE && abs(y - other.y) < dE && abs(radian(theta - other.theta)) < dE;
        }

        friend std::ostream& operator<<(std::ostream& os, Pose const& pose) {
            os << "\n" << "x: " << pose.x << "\n" << "y: " << pose.y << "\n" << "theta: " << pose.theta;
            return os;
        }

    };

}