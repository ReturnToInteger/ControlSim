#pragma once
#include <vector>
#include "model/items/Cone.h"

namespace model
{
	class IMapReader
	{
	public:
		virtual ~IMapReader() = default;
		// Read the map
		virtual std::vector<Cone> Read() = 0;
		static constexpr double defaultRadius = 0.15;
	protected:
	};
}


