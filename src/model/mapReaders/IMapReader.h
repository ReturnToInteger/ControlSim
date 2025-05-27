#pragma once
#include <vector>
#include <string>
#include <src/model/items/Cone.h>
#include <src/model/utils/ObstacleData.h>

namespace model
{
	class IMapReader
	{
	public:
		virtual ~IMapReader() = default;
		// Read the map
		virtual std::vector<Cone> Read() = 0;
	protected:
	};
}


