#include "AIControl.h"


namespace model {
	ControlCommand model::AIControl::drive(const model::VehicleState & state, const model::PathPlanner & pathPlanner)
	{
		auto path = pathPlanner.getPlannedPath();
		if (path.size() < _lookAhead + 1) {
			return ControlCommand(0, 0);
		}
		model::Pose currentPose=state.getPose();
		auto& targetPoint = path[_lookAhead];
		double targetAngle = atan2(targetPoint.y - currentPose.y, targetPoint.x - currentPose.x);

		double angleDiff = validateOrientation(targetAngle - currentPose.theta);
		double angleCommand = angleDiff;
		double maxAngle = state.getMaxSteeringAngle();
		if (abs(angleDiff) >= maxAngle *1.0) {
			angleCommand = (angleDiff > 0) ? 1.0 : -1.0;
		}
		else {
			angleCommand = angleDiff/ maxAngle;
		}
		return ControlCommand(1, angleCommand);
	}
	const model::Pose* model::AIControl::getAtRange(double r, const model::Pose& vehiclePose, const std::vector<model::Pose>& path)
	{
		if (path.size()<2) throw std::out_of_range::out_of_range("Path range has to be at least 2");
		double maxDelta = (path[0]-path[1]).magnitude()/2.0;
		std::vector<Pose*> candidatePoses;
		const Pose* withinRange = nullptr;
		double max = 0;
		for (const model::Pose& pose : path) {
			double magnitude = (vehiclePose - pose).magnitude();
			if (r - maxDelta < magnitude && magnitude < r + maxDelta) {
				if (magnitude > max) {
					max = magnitude;
					withinRange = &pose;
				}
			}
		}
		return withinRange;

	}
}