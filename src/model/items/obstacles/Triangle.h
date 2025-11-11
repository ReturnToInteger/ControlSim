#pragma once
#include "Obstacle.h"
#include "model/utils/Point.h"

namespace model {
	class IVehicleState;
	class Triangle :
		public Obstacle
	{
	public:

		bool detectCollision(model::IVehicleState const& vehicle) const override;

	};

}