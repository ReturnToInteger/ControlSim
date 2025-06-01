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
			ObstacleBase(position), _radius(radius), _type(type) {}
		Cone(double positionX, double positionY, double radius, ConeType type = ConeType::UNKNOWN) : 
			ObstacleBase(positionX, positionY), _radius(radius), _type(type) {}
		double getRadius() const { return _radius; }
		ConeType getType() const { return _type; }
		void setType(ConeType type) { _type = type; }

	private:
		const double _radius;
		ConeType _type;

	};

}