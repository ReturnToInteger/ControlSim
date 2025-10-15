#include "Cone.h"

bool model::Cone::detectCollision(model::IVehicleState const& vehicle) const
{
	auto vehiclePosition = vehicle.getPosition();
	auto vehicleOrientation = vehicle.getOrientation();
	double vhclLength = vehicle.getLength();
	double vhclWidth = vehicle.getWidth();
	Point relPos(getPosition() - vehiclePosition);
	if (relPos.magnitude() > (vhclLength * 1.5 + getRadius()))
	{
		return false; // If the cone is more than sqrt(2)*length away, no collision
	}
	relPos = rotatePoint(relPos, -vehicleOrientation);
	model::Point clampedPoint(model::clamp(relPos.X(), -vhclLength / 2, vhclLength / 2),
		model::clamp(relPos.Y(), -vhclWidth / 2, vhclWidth / 2));
	double distance = (relPos - clampedPoint).magnitude();
	// within radius and margin for error
	if (distance < getRadius())
	{
		//Point contactPoint = rotatePoint(clampedPoint, vehicleOrientation) + vehiclePosition;
		return true;
	}
	else
		return false;

}
