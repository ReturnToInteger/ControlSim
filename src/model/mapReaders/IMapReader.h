#pragma once
#include <vector>
#include <string>
#include <src/model/Cone.h>
#include <src/model/ObstacleData.h>

namespace model
{
	class IMapReader
	{
	public:
		virtual ~IMapReader() = default;
		// Read the map from a file
		virtual std::vector<Cone> Read() = 0;
	protected:
	};
}


