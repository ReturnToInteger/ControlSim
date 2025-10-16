#pragma once
#include "Obstacle.h"
#include "model/utils/Point.h"

namespace model {
	class IVehicleState;
	enum class ConeType
	{
		LEFT,
		RIGHT,
		UNKNOWN
	};

	class Cone :
		public Obstacle
	{
	public:
		Cone(model::Point position, double radius, ConeType type =ConeType::UNKNOWN) : 
			Obstacle(position), m_radius(radius), m_type(type) {}
		Cone(double positionX, double positionY, double radius, ConeType type = ConeType::UNKNOWN) : 
			Obstacle(positionX, positionY), m_radius(radius), m_type(type) {}
		double getRadius() const { return m_radius; }
		ConeType getType() const { return m_type; }
		void setType(ConeType type) { m_type = type; }
		bool detectCollision(model::IVehicleState const& vehicle) const override;

	private:
		const double m_radius;
		ConeType m_type;

	};

}