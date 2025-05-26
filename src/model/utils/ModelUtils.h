#pragma once
#include <string>
#include <chrono>
#include <iostream>
#include <functional> 

#ifndef M_PI
#define M_PI 3.14159265358979323846 
#endif

namespace model {
	struct ControlInput {
		double targetSpeed;
		double steeringAngle;
		ControlInput(double targetSpeed, double steeringAngle) : targetSpeed(targetSpeed), steeringAngle(steeringAngle) {}
	};

	struct Pose {
		double x;
		double y;
		double theta;
        Pose(double x, double y, double theta) : x(x), y(y), theta(theta) {}
        Pose(Point p, double theta) : x(p.X()), y(p.Y()), theta(theta) {}
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

	};

    struct Twist {
        double vx;
        double vy;
        double omega;
        Twist(double vx, double vy, double omega) : vx(vx), vy(vy), omega(omega) {}
        Twist() : vx(0), vy(0), omega(0) {}
    };

    template <typename Func, typename... Args>
    void timeFunction(const std::string& label, Func&& func, Args&&... args) {
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        std::invoke(std::forward<Func>(func), std::forward<Args>(args)...);
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = std::chrono::duration<double>(end - start);
        std::cout << label << " took " << elapsed.count() << " seconds.\n";
    }

    inline double clamp(double value, double min, double max) {
        return std::max(min, std::min(value, max));
    }

    inline double validateOrientation(const double& orientation)
    {
        double a = std::fmod(orientation + M_PI, 2 * M_PI);
        if (a < 0) a += 2 * M_PI;
        return a - M_PI;
    }


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
	template <typename T1, typename T2>
    struct hash<pair<T1,T2>>
    {
        inline size_t operator()(const pair<T1,T2>& p) const
        {
            size_t seed = 0;
            model::hash_combine(seed, p.first);
            model::hash_combine(seed, p.second);
            return seed;
        }
    };
}