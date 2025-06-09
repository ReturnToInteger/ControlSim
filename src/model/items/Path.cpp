#include "Path.h"
#include <iostream>
#include "model/VehicleState.h"

namespace model {
	Point model::Path::getPosition() const
	{
		return Point();
	}
	Path::Path(std::vector<VehicleState> const& path) : _path(path) { if (_path.size() == 0) std::cout << "No path created" << std::endl; }
}