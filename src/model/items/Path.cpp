#include "Path.h"
#include <iostream>
#include "model/VehicleState.h"

namespace model {
	Point model::Path::getPosition() const
	{
		return {};
	}
	Path::Path(std::vector<VehicleState> const& path) : _path(path) 
	{ 
		if (_path.empty()) 
		{
			std::cout << "No path created\n";
		}
	}
}