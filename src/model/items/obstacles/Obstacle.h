#pragma once
#include "model/items/Item.h"
#include "model/utils/Point.h"
#include "model/IVehicleState.h"
namespace model {
    class Obstacle :
		public model::Item
    {
    public:
		Obstacle() = default;
		Obstacle(Point position) : m_position(position) {}
		Obstacle(double x, double y) : m_position(x, y) {}
		Point getPosition() const override { return m_position; }
		virtual bool detectCollision(model::IVehicleState const& vehicle) const = 0;
		virtual ~Obstacle()  = default;
	protected:
		const Point m_position;
    };
}
