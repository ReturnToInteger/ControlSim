#pragma once
#include "IMapReader.h"
#include <vector>
#include <memory>
#include "model/items/Cone.h"
#include "model/mapReaders/Map.h"

namespace model{
template <typename Iterable>
    class ManualConeReader :
        public IMapReader
    {
    public:
        ManualConeReader(Iterable const& obstacles);
        model::Map Read() override;
    private:
		const Iterable m_obstacles;
    };

    template<typename Iterable>
    model::ManualConeReader<Iterable>::ManualConeReader(Iterable const& obstacles)
        : obstacles(obstacles)
    {
    }

    template<typename Iterable>
    model::Map model::ManualConeReader<Iterable>::Read()
    {
        model::Map map;
        for (auto const& cone : m_obstacles) {
            map.addObstacle(std::make_unique<model::Cone>(cone));
        }
        return map;
    }

}

