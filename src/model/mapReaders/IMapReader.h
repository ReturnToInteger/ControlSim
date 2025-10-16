#pragma once
#include <vector>
#include "model/items/obstacles/Cone.h"
#include "model/mapReaders/Map.h"

namespace model
{
	class IMapReader
	{
	public:
		virtual ~IMapReader() = default;
		// Read the map
		virtual model::Map Read() = 0;
		static constexpr double defaultRadius = 0.15;
	protected:
	};
}


