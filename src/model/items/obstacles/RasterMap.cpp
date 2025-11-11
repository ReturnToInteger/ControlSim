#include "RasterMap.h"
#include "model/utils/Point.h"
#include <cassert>

bool model::RasterMap::detectCollision(model::IVehicleState const& vehicle) const
{
	double radius = vehicle.getWidth() / 2;
	model::Point dx = rotatePoint(Point(vehicle.getLength() - radius, 0), vehicle.getOrientation());
	model::Point center = vehicle.getPosition();
	model::Point rear = center - dx;
	model::Point front = center + dx;

	return false;
}
