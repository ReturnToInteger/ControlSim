#pragma once
#include <cmath>
#include <ostream>
#include <numbers>
#include "Point.h"
namespace model {
    double normAngle(double const& orientation);

    // Normalized angle between -PI,PI
    class Angle {
    public:
        Angle() : m_a(0) {}
        Angle(double a) : m_a(normAngle(a)) {}

        // Explicit conversion to double (avoid accidental conversions)
        explicit operator double() const 
        {
            return m_a;
        }

        double asRadians() const 
        {
            return m_a;
        }
        double asDegrees() const 
        {
            return m_a / std::numbers::pi * 180;;
        }

        // Arithmetic operators
        Angle operator+(Angle const& other) const 
        {
            return Angle(m_a + other.m_a);
        }
        Angle operator-(Angle const& other) const 
        {
            return Angle(m_a - other.m_a);
        }
        Angle operator+(double val) const 
        {
            return Angle(m_a + val);
        }
        Angle operator-(double val) const 
        {
            return Angle(m_a - val);
        }

        Angle& operator+=(Angle const& other) 
        {
            *this = *this + other;
            return *this;
        }
        Angle& operator-=(Angle const& other) 
        {
            *this = *this - other;
            return *this;
        }
        Angle& operator+=(double val) 
        {
            *this = *this + val;
            return *this;
        }
        Angle& operator-=(double val) 
        {
            *this = *this - val;
            return *this;
        }
        // Multiply Angle by double (scale)
        Angle operator*(double val) const 
        {
            return Angle(m_a * val);
        }

        // Divide Angle by double
        Angle operator/(double val) const 
        {
            return Angle(m_a / val);
        }

        Angle operator-() const 
        {
            return Angle(-m_a);
        }

        // operator* not defined


        bool isClockwiseTo(Angle const& other) const
        {
            return normAngle(m_a - other.m_a) > 0;
        }
        bool isCounterClockwiseTo(Angle const& other) const 
        {
            return normAngle(m_a - other.m_a) < 0;
        }

        friend std::ostream& operator<<(std::ostream& os, Angle const& angle)
        {
            os << angle.m_a;
            return os;
        }


    private:
        double m_a;   // normalized angle in radians
    };

    // Functions for convenience
    inline double sin(Angle const& a) { return std::sin(a.asRadians()); }
    inline double cos(Angle const& a) { return std::cos(a.asRadians()); }
    inline double tan(Angle const& a) { return std::tan(a.asRadians()); }
    inline double radian(Angle const& a) { return a.asRadians(); }
    inline double degree(Angle const& a) { return a.asDegrees(); }
    inline Angle radian(double const& d) { return d/180.0*std::numbers::pi; }
    inline Angle clampRelativeToZero(Angle const& angle, Angle const& maxAbs) 
    {
        double val = angle.asRadians();
        double maxVal = std::abs(maxAbs.asRadians()); // ensure positive

        if (val > maxVal) return Angle(maxVal);
        if (val < -maxVal) return Angle(-maxVal);
        return angle;
    }

    // Non-member operator to support double * Angle
    inline Angle operator*(double val, Angle const& angle)
    {
        return Angle(angle.asRadians() * val);
    }
    // Returns ratio of two angles as double
    inline double operator/(Angle const& lhs, Angle const& rhs)
    {
        return lhs.asRadians() / rhs.asRadians();
    }

    inline model::Point rotatePoint(model::Point const& p, model::Angle const& a) {
        double c = cos(a);
        double s = sin(a);
        return Point(p.X() * c - p.Y() * s, p.X() * s + p.Y() * c);
    }



}