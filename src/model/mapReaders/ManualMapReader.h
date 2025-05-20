#pragma once
#include "IMapReader.h"

namespace model{
template <typename Iterable>
    class ManualMapReader :
        public IMapReader
    {
    public:
        ManualMapReader(const Iterable& obstacles);
        std::vector<model::Cone> Read() override;
    private:
		const Iterable _obstacles;
    };

    template<typename Iterable>
    model::ManualMapReader<Iterable>::ManualMapReader(const Iterable& obstacles)
        : _obstacles(obstacles)
    {
    }

    template<typename Iterable>
    std::vector<model::Cone> model::ManualMapReader<Iterable>::Read()
    {
        std::vector<model::Cone> cones;
        for (const auto& obstacle : _obstacles) {
            cones.emplace_back(obstacle);
        }
        return cones;
    }

}

