#pragma once
#include "IMapReader.h"
#include <vector>
#include "model/items/Cone.h"

namespace model{
template <typename Iterable>
    class ManualMapReader :
        public IMapReader
    {
    public:
        ManualMapReader(Iterable const& obstacles);
        std::vector<model::Cone> Read() override;
    private:
		const Iterable m_obstacles;
    };

    template<typename Iterable>
    model::ManualMapReader<Iterable>::ManualMapReader(Iterable const& obstacles)
        : obstacles(obstacles)
    {
    }

    template<typename Iterable>
    std::vector<model::Cone> model::ManualMapReader<Iterable>::Read()
    {
        std::vector<model::Cone> cones;
        for (auto const& obstacle : m_obstacles) {
            cones.emplace_back(obstacle);
        }
        return cones;
    }

}

