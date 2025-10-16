#pragma once
#include <memory>
#include <vector>

namespace model {
	class Map {
	public:
        Map() = default;

        void addObstacle(std::unique_ptr<Obstacle> obs) {
            m_obstacles.push_back(std::move(obs));
        }

        const std::vector<std::unique_ptr<Obstacle>>& getObstacles() const {
            return m_obstacles;
        }
        int size() const { return m_obstacles.size(); }
    private:
		std::vector<std::unique_ptr<Obstacle>> m_obstacles;
	};
}