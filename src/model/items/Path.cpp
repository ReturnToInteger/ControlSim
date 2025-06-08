#include "Path.h"
#include <iostream>

namespace model {
	Point model::Path::getPosition() const
	{
		return Point();
	}
	Path::Path(std::vector<Pose> const& path) : _path(path) { if (_path.size() == 0) std::cout << "No path created" << std::endl; }
}