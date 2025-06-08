#include "AIControl.h"
#include <iostream>

#include "model/VehicleState.h"
#include "model/pathPlanning/PathPlanner.h"
#include "model/utils/ModelUtils.h"
#include "model/utils/Pose.h"
#include "model/utils/Angle.h"
#include "model/utils/Point.h"

#include <stdexcept>




namespace model {
	ControlCommand AIControl::drive(VehicleState const& state, pathPlanning::PathPlanner const& pathPlanner)
	{
		auto path = pathPlanner.getPlannedPath();
		if (path.size() < 2) {
			return ControlCommand(0, 0);
		}
		Pose currentPose=state.getPose();
		const Pose* targetPose = _getAtRange(currentPose, path);
		if (!targetPose) return ControlCommand(0, 0);
		Angle targetAngle;
		targetAngle = atan2(targetPose->y - currentPose.y, targetPose->x - currentPose.x);
		Angle angleDiff = targetAngle - state.getOrientation();
		Angle angleControl = atan(sin(angleDiff)*state.getLength()*2.0/(Point(currentPose) - Point(*targetPose)).magnitude());
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
	Pose const* AIControl::_getAtRange(const model::Pose const& vehiclePose, Path const& path) const
	{
		if (path.size()<2) throw std::out_of_range::out_of_range("Path range has to be at least 2");
		double maxDelta = (path[0] - path[1]).magnitude() / 2.0;
		double radius = (path[0] - path[1]).magnitude() * _lookAhead;
		//std::vector<Pose*> candidatePoses;
		const Pose* withinRange = nullptr;
		double max = 0;
		for (model::Pose const& pose : path) {
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