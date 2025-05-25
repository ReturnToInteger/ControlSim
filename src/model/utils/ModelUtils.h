#pragma once
#include <string>
#include <chrono>
#include <iostream>
#include <functional> 

namespace model {
	struct ControlInput {
		double targetSpeed;
		double steeringAngle;
		ControlInput(double targetSpeed, double steeringAngle) : targetSpeed(targetSpeed), steeringAngle(steeringAngle) {}
	};

	struct Pose {
		double x;
		double y;
		double orientation;
		Pose(double x, double y, double orientation) : x(x), y(y), orientation(orientation) {}
	};

    template <typename Func, typename... Args>
    void timeFunction(const std::string& label, Func&& func, Args&&... args) {
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        std::invoke(std::forward<Func>(func), std::forward<Args>(args)...);
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = std::chrono::duration<double>(end - start);
        std::cout << label << " took " << elapsed.count() << " seconds.\n";
    }

    double clamp(double value, double min, double max);


    // From boost
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