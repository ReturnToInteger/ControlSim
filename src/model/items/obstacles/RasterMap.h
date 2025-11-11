#pragma once
#include "Obstacle.h"
#include "model/utils/Point.h"
#include <vector>

namespace model {
	class IVehicleState;
	class RasterMap :
		public Obstacle
	{
	public:
		bool detectCollision(model::IVehicleState const& vehicle) const override;
	private:
		std::vector<std::vector<int>> data;

	};


}