#pragma once
#include <cmath>
#include <ostream>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace model {
    double normAngle(double const& orientation);

    // Normalized angle between -PI,PI
    class Angle {
    public:
        Angle() : _a(0) {}
        Angle(double a) : _a(normAngle(a)) {}

        // Explicit conversion to double (avoid accidental conversions)
        explicit operator double() const 
        {
            return _a;
        }

        double asRadians() const 
        {
            return _a;
        }
        double asDegrees() const 
        {
            return _a / M_PI * 180;;
        }

        // Arithmetic operators
        Angle operator+(Angle const& other) const 
        {
            return Angle(_a + other._a);
        }
        Angle operator-(Angle const& other) const 
        {
            return Angle(_a - other._a);
        }
        Angle operator+(double val) const 
        {
            return Angle(_a + val);
        }
        Angle operator-(double val) const 
        {
            return Angle(_a - val);
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
            return Angle(_a * val);
        }

        // Divide Angle by double
        Angle operator/(double val) const 
        {
            return Angle(_a / val);
        }

        Angle operator-() const 
        {
            return Angle(-_a);
        }

        // operator* not defined


        bool isClockwiseTo(Angle const& other) const
        {
            return normAngle(_a - other._a) > 0;
        }
        bool isCounterClockwiseTo(Angle const& other) const 
        {
            return normAngle(_a - other._a) < 0;
        }

        friend std::ostream& operator<<(std::ostream& os, Angle const& angle)
        {
            os << angle._a;
            return os;
        }


    private:
        double _a;   // normalized angle in radians
    };

    // Functions for convenience
    inline double sin(Angle const& a) { return std::sin(a.asRadians()); }
    inline double cos(Angle const& a) { return std::cos(a.asRadians()); }
    inline double tan(Angle const& a) { return std::tan(a.asRadians()); }
    inline double radian(Angle const& a) { return a.asRadians(); }
    inline double degree(Angle const& a) { return a.asDegrees(); }
    inline Angle radian(double const& d) { return d/180.0*M_PI; }
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




}