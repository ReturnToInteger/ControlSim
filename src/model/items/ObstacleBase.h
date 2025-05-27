#pragma once
#include "Item.h"
namespace model {
    class ObstacleBase :
        public Item
    {
    public:
		ObstacleBase(Point position) : _position(position) {}
		ObstacleBase(double x, double y) : _position(x, y) {}
		Point getPosition() const override { return _position; }
		virtual ~ObstacleBase() = default;
	protected:
		const Point _position;
    };
}
