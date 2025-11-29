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
		model::RasterMap(int width, int height) : data(height, std::vector<float>(width)) {

		}
		bool detectCollision(model::IVehicleState const& vehicle) const override;
	private:
		std::vector<std::vector<float>> data;

	};


}