#include "AIControl.h"


namespace model {
	ControlCommand model::AIControl::drive(const model::VehicleState & state, const model::PathPlanner & pathPlanner)
	{
		auto path = pathPlanner.getPlannedPath();
		if (path.size() < _lookAhead + 1) {
			return ControlCommand(0, 0);
		}
		auto& targetPoint = path[_lookAhead];
		double targetAngle = atan2(targetPoint.Y() - state.getPosition().Y(), targetPoint.X() - state.getPosition().X());
		// This needs to be rewritten to min(
		double angleDiff = validateOrientation(targetAngle - state.getOrientation());
		return ControlCommand(state.getMaxSpeed(), angleDiff);
	}
}