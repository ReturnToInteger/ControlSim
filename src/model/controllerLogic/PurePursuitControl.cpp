#include "PurePursuitControl.h"
#include <iostream>

#include "model/VehicleState.h"
#include "model/pathPlanning/PathPlanner.h"
#include "model/utils/ModelUtils.h"
#include "model/utils/Pose.h"
#include "model/utils/Angle.h"
#include "model/utils/Point.h"

#include <stdexcept>




namespace model {
	ControlCommand model::PurePursuitControl::drive(VehicleState const& state, model::Path const& path)
	{
		if (path.size() < 2) {
			return ControlCommand(.2, _previous.steeringAngle);
		}
		Pose currentPose=state.getRearPose();
		const VehicleState* targetState = _getAtRange(currentPose, path);
		if (!targetState) return ControlCommand(0, _previous.steeringAngle);
		Pose targetPose = targetState->getRearPose();
		//const Pose targetPose = path[_lookAhead].getRearPose();
		Angle targetAngle;
		targetAngle = atan2(targetPose.y - currentPose.y, targetPose.x - currentPose.x);
		Angle angleDiff = targetAngle - currentPose.theta;
		Angle angleControl = atan(sin(angleDiff)*state.getWheelBase()*2.0/(Point(currentPose) - Point(targetPose)).magnitude());
		Angle maxAngle = state.getMaxSteeringAngle();
		double angleCommand = angleControl/maxAngle;
		//if (abs(angleDiff) >= maxAngle *1.0) {
		//	angleCommand = (angleDiff > Angle(0)) ? 1.0 : -1.0;
		//}
		//else {
		//	angleCommand = angleDiff/ maxAngle;
		//}
		_previous = ControlCommand(1 - std::min(abs(angleCommand), 0.5), angleCommand);
		return _previous;
	}

	// Select the point that is the closest to the intersection of the lookahead radius and the path
	// ... was doing that
	// I need to move logic
	VehicleState const* model::PurePursuitControl::_getAtRange(model::Pose const& vehiclePose, Path const& path) const
	{
		if (path.size()<2) throw std::out_of_range::out_of_range("Path range has to be at least 2");
		//double maxDelta = (path[0].getRearPose() - path[1].getRearPose()).magnitude() / 2.0;
		double radius = (path[0].getRearPose() - path[1].getRearPose()).magnitude() * _lookAhead;
		double maxDelta = (path[0].getRearPose() - path.back().getRearPose()).magnitude()*2;
		double maxSteering = path[0].getMaxSteeringAngle();
		//std::vector<VehicleState*> candidatePoses;
		const VehicleState* withinRange = nullptr;
		double min = INFINITY;
		for (model::VehicleState const& state : path) {
			Pose pose = state.getRearPose();
			double magnitude = (vehiclePose - pose).magnitude();
			Angle targetAngle = atan2(pose.y - vehiclePose.y, pose.x - vehiclePose.x);
			Angle angleDiff = targetAngle - vehiclePose.theta;
			Angle angleControl = atan(sin(angleDiff) * state.getWheelBase() * 2.0 / (Point(vehiclePose) - Point(pose)).magnitude());
			if (abs((double)angleControl)<=maxSteering && magnitude >= radius && magnitude <= maxDelta) {
				if (magnitude < min) {
					min = magnitude;
					withinRange = &state;
				}
			}
		}
		return withinRange;

	}
}