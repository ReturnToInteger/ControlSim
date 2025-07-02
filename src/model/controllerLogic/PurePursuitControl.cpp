#include "PurePursuitControl.h"
#include <iostream>

#include "model/VehicleState.h"
#include "model/pathPlanning/PathPlanner.h"
#include "model/utils/ModelUtils.h"
#include "model/utils/Pose.h"
#include "model/utils/Angle.h"
#include "model/utils/Point.h"
#include "view/DebugDraw.h"
#include <stdexcept>




namespace model {
	ControlCommand model::PurePursuitControl::drive(VehicleState const& state, model::Path const& path)
	{
		if (path.size() < 2) {
			return { defaultSpeedInput, _previous.normSteering };
		}
		Pose currentPose=state.getRearPose();
		const VehicleState* targetState = _getAtRange(currentPose, path);
		if (!targetState) {
			#ifdef ENABLE_DEBUG_DRAW
			view::DebugDraw::instance().circle2(model::Point(-1000.0,-1000.0), static_cast<float>(0.01));
			#endif // ENABLE_DEBUG_DRAW

			return { 0.2, _previous.normSteering };
		}
		#ifdef ENABLE_DEBUG_DRAW
		view::DebugDraw::instance().circle2(targetState->getPosition(), static_cast<float>(0.15));
		#endif // ENABLE_DEBUG_DRAW

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
		_previous = { 1 - std::min(abs(angleCommand), 0.5), angleCommand };
		return _previous;
	}

	// Select the point that is the closest to the intersection of the lookahead radius and the path
	// ... was doing that
	// I need to move logic
	VehicleState const* model::PurePursuitControl::_getAtRange(model::Pose const& vehiclePose, Path const& path) const
	{
		if (path.size()<2) 
		{
			throw std::out_of_range("Path range has to be at least 2");
		}
		//double maxDelta = (path[0].getRearPose() - path[1].getRearPose()).magnitude() / 2.0;
		double radius = (path[0].getRearPose() - path[1].getRearPose()).magnitude() * _lookAhead;
		double maxDelta = (path[0].getRearPose() - path.back().getRearPose()).magnitude()*2;
		Angle maxSteering = path[0].getMaxSteeringAngle();
		//std::vector<VehicleState*> candidatePoses;
		const VehicleState* withinRange = nullptr;
		double minDistance = std::numeric_limits<double>::infinity();			
		Point normal(cos(vehiclePose.theta), sin(vehiclePose.theta));

		for (model::VehicleState const& state : path) {
			Pose pose = state.getRearPose();
			double magnitude = (vehiclePose - pose).magnitude();
			Angle targetAngle = atan2(pose.y - vehiclePose.y, pose.x - vehiclePose.x);
			Angle angleDiff = targetAngle - vehiclePose.theta;
			Angle angleControl = atan(sin(angleDiff) * state.getWheelBase() * 2.0 / (Point(vehiclePose) - Point(pose)).magnitude());
			double distAlongHeading = normal.X() * (pose.x - vehiclePose.x) + normal.Y() * (pose.y - vehiclePose.y);

			bool isSteeringValid = std::abs(radian(angleControl)) <= radian(maxSteering);
			bool isDistanceValid = magnitude >= radius && magnitude <= maxDelta;
			bool isInFront = distAlongHeading > 0;

			if (isSteeringValid && isDistanceValid && isInFront) {
				if (magnitude < minDistance) {
					minDistance = magnitude;
					withinRange = &state; 
					//return withinRange;
				}
			}
		}
		return withinRange;

	}
}