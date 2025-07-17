#pragma once
#include "ObstacleBase.h"
#include "model/utils/Point.h"

namespace model {
	enum class ConeType
	{
		LEFT,
		RIGHT,
		UNKNOWN
	};

	class Cone :
		public ObstacleBase
	{
	public:
		Cone(model::Point position, double radius, ConeType type =ConeType::UNKNOWN) : 
			ObstacleBase(position), m_radius(radius), m_type(type) {}
		Cone(double positionX, double positionY, double radius, ConeType type = ConeType::UNKNOWN) : 
			ObstacleBase(positionX, positionY), m_radius(radius), m_type(type) {}
		double getRadius() const { return m_radius; }
		ConeType getType() const { return m_type; }
		void setType(ConeType type) { m_type = type; }

	private:
		const double m_radius;
		ConeType m_type;

	};

}