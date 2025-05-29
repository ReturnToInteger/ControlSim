#pragma once
#include <vector>
#include <string>
#include <model/items/Cone.h>
#include <model/utils/ObstacleData.h>

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


