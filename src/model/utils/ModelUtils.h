#pragma once

namespace model {
	struct ControlInput {
		double targetSpeed;
		double steeringAngle;
		ControlInput(double targetSpeed, double steeringAngle) : targetSpeed(targetSpeed), steeringAngle(steeringAngle) {}
	};

    double clamp(double value, double min, double max);

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