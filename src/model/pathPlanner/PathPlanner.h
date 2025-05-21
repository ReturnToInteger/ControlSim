#pragma once
#include <vector>
#include <src/model/Cone.h>
#include <src/model/VehicleState.h>

namespace model {
	class PathPlanner
	{
	public:
		PathPlanner();
		~PathPlanner() = default;
		void planPath(const std::vector<model::Cone*>& cones, const model::VehicleState& vehicleState);
		std::vector<model::Point> getPlannedPath() const { return _plannedPath; }
	private:
		std::vector<model::Point> _plannedPath;
	};


}
