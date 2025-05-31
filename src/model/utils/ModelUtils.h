#pragma once
#include <string>
#include <chrono>
#include <iostream>
#include <functional> 
#include "Point.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846 
#endif

namespace model {
	struct ControlInput {
		double targetSpeed;
		double steeringAngle;
		ControlInput(double targetSpeed, double steeringAngle) : targetSpeed(targetSpeed), steeringAngle(steeringAngle) {}
	};
    inline double normAngle(const double& orientation);

    struct Angle {
    public:
        Angle() :_a(0), _sin(0), _cos(1) {}
        Angle(double a) : _a(normAngle(a)), _sin(sin(a)),_cos(cos(a)) {}
        Angle& operator=(const Angle& other) {
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

        Angle operator+(const Angle& other) const{
            return Angle(_a + other._a);
        }
        Angle operator-(const Angle& other) const{
            return Angle(_a - other._a);
        }
        Angle operator+(const double& val) const {
            return Angle(_a + val);
        }
        Angle operator-(const double& val) const {
            return Angle(_a - val);
        }

        bool operator>(const Angle& other) const{
            if (abs(_a - other._a) < M_PI)
                return _a > other._a;
            else if (_a > other._a)
                return false;
            else return true;
        }
        bool operator<(const Angle& other) const{
            if (abs(_a - other._a) < M_PI)
                return _a < other._a;
            else if (_a < other._a)
                return false;
            else return true;
        }
        Angle& operator+=(const Angle& other) {
            _a = normAngle(_a + other._a);
            _sin = sin(_a + other._a);
            _cos = cos(_a + other._a);
            return *this;
        }
        Angle& operator-=(const Angle& other) {
            _a = normAngle(_a - other._a);
            _sin = sin(_a + other._a);
            _cos = cos(_a + other._a);
            return *this;
        }
        Angle& operator+=(const double& val) {
            _a = normAngle(_a + val);
            _sin = sin(_a + val);
            _cos = cos(_a + val);
            return *this;
        }
        Angle& operator-=(const double& val) {
            _a = normAngle(_a - val);
            _sin = sin(_a + val);
            _cos = cos(_a + val);
            return *this;
        }
        friend std::ostream& operator<<(std::ostream& os, const Angle& angle) {
            os << angle._a; 
            return os;
        }

        friend double sin(const Angle& a) {
            return a._sin;
        }
        friend double cos(const Angle& a) {
            return a._cos;
        }

    private:
        double _a;
        double _sin;
        double _cos;
    };

    inline double sin(const Angle& a);
    inline double cos(const Angle& a);

    //inline Point rotatePoint(const model::Point& p, const model::Angle& a) {
    //    double c = model::cos(a);
    //    double s = model::sin(a);
    //    return Point(p.X() * c - p.Y() * s, p.X() * s + p.Y() * c);
    //}


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


        Pose operator+(const Pose& other) const
        {
            return Pose(x + other.x, y + other.y, theta + other.theta);
        }
        Pose operator-(const Pose& other) const
        {
            return Pose(x - other.x, y - other.y, theta - other.theta);
        }
        double operator*(const Pose& other) const
        {
            return x * other.x + y * other.y;
        }


        Pose& operator+=(const Pose& other)
        {
            x += other.x;
            y += other.y;
            theta += other.theta;
            return *this;
        }

        bool operator==(const Pose& other) const
        {
            double dE = 1e-6;
            return abs(x - other.x)<dE && abs(y - other.y) < dE && abs(theta - other.theta) < dE;
        }

        friend std::ostream& operator<<(std::ostream& os, const Pose& pose) {
            os << std::endl << "x: " << pose.x << std::endl << "y: " << pose.y << std::endl << "theta: " << pose.theta;
            return os;
        }

	};

    struct Twist {
        double vx;
        double vy;
        double omega;
        Twist(double vx, double vy, double omega) : vx(vx), vy(vy), omega(omega) {}
        Twist() : vx(0), vy(0), omega(0) {}
    };

    template <typename Func, typename... Args>
    std::chrono::duration<double> timeFunction(const std::string& label, Func&& func, Args&&... args) {
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        std::invoke(std::forward<Func>(func), std::forward<Args>(args)...);
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = std::chrono::duration<double>(end - start);
        //std::cout << label << " took " << elapsed.count() << " seconds.\n";
        return elapsed;
    }

    inline double clamp(double value, double min, double max) {
        return std::max(min, std::min(value, max));
    }

    inline double normAngle(const double& orientation)
    {
        double a = std::fmod(orientation + M_PI, 2 * M_PI);
        if (a < 0) a += 2 * M_PI;
        return a - M_PI;
    }

    //inline bool angleIsGreater(double alpha, double beta) {
    //    if (abs(alpha - beta) > M_PI) {

    //    }
    //}


    // From Boost
    template <class T>
    inline void hash_combine(std::size_t& seed, const T& v)
    {
        std::hash<T> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }



}

namespace std
{
	template <typename T1, typename T2, typename T3>
    struct hash<tuple<T1,T2,T3>>
    {
        inline size_t operator()(const tuple<T1,T2,T3>& p) const
        {
            size_t seed = 0;
            model::hash_combine(seed, std::get<0>(p));
            model::hash_combine(seed, std::get<1>(p));
            model::hash_combine(seed, std::get<2>(p));
            return seed;
        }
    };
}