#pragma once
#include <string>
#include <chrono>
#include <iostream>
#include <functional> 
#include <numbers>
//#include "Point.h"
namespace model {
	//struct ControlInput {
	//	double targetSpeed;
	//	double steeringAngle;
	//	ControlInput(double targetSpeed, double steeringAngle) : targetSpeed(targetSpeed), steeringAngle(steeringAngle) {}
	//};

    struct VelocityCommand {
        double linear;
        double angular;
    };
    struct Twist {
        double vx;
        double vy;
        double omega;
        Twist(double vx, double vy, double omega) : vx(vx), vy(vy), omega(omega) {}
        Twist() : vx(0), vy(0), omega(0) {}
    };
    struct Accel {
        double ax;
        double ay;
        double epsilon;
    };
    struct ObstacleData
    {
        double x; // X coordinate of the obstacle
        double y; // Y coordinate of the obstacle
        double radius; // Radius of the obstacle
    };
    template <typename Func, typename... Args>
    inline std::chrono::duration<double> timeFunction(std::string const& label, Func&& func, Args&&... args) {
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

    inline double normAngle(double const& orientation)
    {
        double a = std::fmod(orientation + std::numbers::pi, 2 * std::numbers::pi);
        if (a < 0) a += 2 * std::numbers::pi;
        return a - std::numbers::pi;
    }

    //inline bool angleIsGreater(double alpha, double beta) {
    //    if (abs(alpha - beta) > std::numbers::pi) {

    //    }
    //}


    // From Boost
    template <class T>
    inline void hash_combine(std::size_t& seed, T const& v)
    {
        std::hash<T> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }



}

namespace std
{
	template <typename T1, typename T2, typename T3, typename T4>
    struct hash<tuple<T1,T2,T3,T4>>
    {
        inline size_t operator()(tuple<T1,T2,T3,T4> const& p) const
        {
            size_t seed = 0;
            model::hash_combine(seed, std::get<0>(p));
            model::hash_combine(seed, std::get<1>(p));
            model::hash_combine(seed, std::get<2>(p));
            model::hash_combine(seed, std::get<3>(p));
            return seed;
        }
    };
}