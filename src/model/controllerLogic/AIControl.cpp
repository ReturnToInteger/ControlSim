#include "AIControl.h"
#include "model/VehicleState.h"
#include "model/pathPlanner/PathPlanner.h"
#include "model/utils/ModelUtils.h"
#include <iostream>




namespace model {
	ControlCommand model::AIControl::drive(const model::VehicleState & state, const model::PathPlanner & pathPlanner)
	{
		auto path = pathPlanner.getPlannedPath();
		if (path.size() < _lookAhead + 1) {
			return ControlCommand(0, 0);
		}
		model::Pose currentPose=state.getPose();
		const Pose* targetPose = getAtRange(currentPose, path);
		Angle targetAngle;
		if (!targetPose) return ControlCommand(0, 0);
		targetAngle = atan2(targetPose->y - currentPose.y, targetPose->x - currentPose.x);
		Angle angleDiff = targetAngle - state.getOrientation();
		Angle angleControl = atan(sin(angleDiff)*state.getLength()*2.0/_lookAhead/(path[0]-path[1]).magnitude());
		Angle maxAngle = state.getMaxSteeringAngle();
		double angleCommand = angleControl/maxAngle;
		//if (abs(angleDiff) >= maxAngle *1.0) {
		//	angleCommand = (angleDiff > Angle(0)) ? 1.0 : -1.0;
		//}
		//else {
		//	angleCommand = angleDiff/ maxAngle;
		//}
		return ControlCommand(1, angleCommand);
	}
	const model::Pose* model::AIControl::getAtRange(const model::Pose& vehiclePose, const std::vector<model::Pose>& path) const
	{
		if (path.size()<2) throw std::out_of_range::out_of_range("Path range has to be at least 2");
		double maxDelta = (path[0] - path[1]).magnitude() / 2.0;
		double radius = (path[0] - path[1]).magnitude() * _lookAhead;
		std::vector<Pose*> candidatePoses;
		const Pose* withinRange = nullptr;
		double max = 0;
		for (const model::Pose& pose : path) {
			double magnitude = (vehiclePose - pose).magnitude();
			if (magnitude < radius + maxDelta) {
				if (magnitude > max) {
					max = magnitude;
					withinRange = &pose;
				}
			}
		}
		return withinRange;

	}
}