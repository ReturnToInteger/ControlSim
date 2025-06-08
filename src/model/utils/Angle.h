#pragma once
#include <cmath>
#include <ostream>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace model {
    double normAngle(double const& orientation);
    struct Angle;
    double sin(Angle const& a);
    double cos(Angle const& a);

    struct Angle {
    public:
        Angle() :_a(0), _sin(0), _cos(1) {}
        Angle(double a) : _a(normAngle(a)), _sin(std::sin(a)), _cos(std::cos(a)) {}
        Angle& operator=(Angle const& other) {
            _a = other._a;
            return *this;
        }
        Angle& operator=(double val) {
            _a = normAngle(val);
            return *this;
        }
        operator double() const {
            return _a;
        }

        Angle operator+(Angle const& other) const {
            return Angle(_a + other._a);
        }
        Angle operator-(Angle const& other) const {
            return Angle(_a - other._a);
        }
        Angle operator+(double const& val) const {
            return Angle(_a + val);
        }
        Angle operator-(double const& val) const {
            return Angle(_a - val);
        }

        bool operator>(Angle const& other) const {
            if (abs(_a - other._a) < M_PI)
                return _a > other._a;
            else if (_a > other._a)
                return false;
            else return true;
        }
        bool operator<(Angle const& other) const {
            if (abs(_a - other._a) < M_PI)
                return _a < other._a;
            else if (_a < other._a)
                return false;
            else return true;
        }
        Angle& operator+=(Angle const& other) {
            _a = normAngle(_a + other._a);
            _sin = sin(_a + other._a);
            _cos = cos(_a + other._a);
            return *this;
        }
        Angle& operator-=(Angle const& other) {
            _a = normAngle(_a - other._a);
            _sin = sin(_a + other._a);
            _cos = cos(_a + other._a);
            return *this;
        }
        Angle& operator+=(double const& val) {
            _a = normAngle(_a + val);
            _sin = sin(_a + val);
            _cos = cos(_a + val);
            return *this;
        }
        Angle& operator-=(double const& val) {
            _a = normAngle(_a - val);
            _sin = sin(_a + val);
            _cos = cos(_a + val);
            return *this;
        }
        friend std::ostream& operator<<(std::ostream& os, Angle const& angle) {
            os << angle._a;
            return os;
        }

        friend double sin(Angle const& a) {
            return a._sin;
        }
        friend double cos(Angle const& a) {
            return a._cos;
        }

    private:
        double _a;
        double _sin;
        double _cos;
    };
}