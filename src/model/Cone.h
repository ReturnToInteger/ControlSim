#pragma once
#include "ObstacleBase.h"
namespace model {
	class Cone :
		public ObstacleBase
	{
	public:
		Cone(model::Point position, double radius) : ObstacleBase(position), _radius(radius) {}
		Cone(double positionX, double positionY, double radius) : ObstacleBase(positionX, positionY), _radius(radius) {}

	private:
		double _radius;
	};
}