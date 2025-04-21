#pragma once
#include <vector>
#include "ObstacleBase.h"
#include "Vehicle.h"
namespace model {
    class World
    {
    private:
		std::vector<ObstacleBase> _obstacles;
		std::vector<Vehicle> _vehicles;
    };
}
