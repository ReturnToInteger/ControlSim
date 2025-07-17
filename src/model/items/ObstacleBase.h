#pragma once
#include "Item.h"
#include "model/utils/Point.h"
namespace model {
    class ObstacleBase :
        public Item
    {
    public:
		ObstacleBase(Point position) : m_position(position) {}
		ObstacleBase(double x, double y) : m_position(x, y) {}
		Point getPosition() const override { return m_position; }
		virtual ~ObstacleBase() = default;
	protected:
		const Point m_position;
    };
}
