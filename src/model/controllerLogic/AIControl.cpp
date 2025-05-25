#include "AIControl.h"


namespace model {
	void AIControl::drive(model::VehicleState& state, model::PathPlanner& pathPlanner)
	{
		auto path = pathPlanner.getPlannedPath();
		if (path.size()<_lookAhead+1) {
			state.setTargetSpeed(0);
			state.setTargetSteeringAngle(0);
			return;
		}
		auto targetPoint = path[_lookAhead];
		double targetAngle = atan2(targetPoint.Y() - state.getPosition().Y(), targetPoint.X() - state.getPosition().X());
		double angleDiff = targetAngle - state.getOrientation();
		state.setTargetSpeed(10000000); // Set a constant speed
		state.setTargetSteeringAngle(angleDiff);
	}
}