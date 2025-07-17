#include "PurePursuitControl.h"
#include <iostream>

#include "model/VehicleState.h"
#include "model/IVehicleState.h"
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
			return { defaultSpeedInput, m_previous.normSteeringAngle };
		}
		m_wheelbase = state.getWheelBase();
		m_maxSteeringAngle = state.getMaxSteeringRate();
		Pose currentPose=state.getRearPose();
		const model::IVehicleState* targetState = getAtRange(currentPose, path);
		if (!targetState) {
			#ifdef ENABLE_DEBUG_DRAW
			view::DebugDraw::instance().circle2(model::Point(-1000.0,-1000.0), static_cast<float>(0.01));
			#endif // ENABLE_DEBUG_DRAW

			return { 0.2, m_previous.normSteeringAngle };
		}
		#ifdef ENABLE_DEBUG_DRAW
		view::DebugDraw::instance().circle2(targetState->getPosition(), static_cast<float>(0.15));
		#endif // ENABLE_DEBUG_DRAW

		Pose targetPose = rearFromCenter(*targetState,m_wheelbase);
		//const Pose targetPose = path[m_lookAhead].getRearPose();
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
		m_previous = { 1 - std::min(abs(angleCommand), 0.5), angleCommand };
		return m_previous;
	}

	// Select the point that is the closest to the intersection of the lookahead radius and the path
	// ... was doing that
	// I need to move logic
	IVehicleState const* model::PurePursuitControl::getAtRange(model::Pose const& vehiclePose, Path const& path) const
	{
		if (path.size()<2) 
		{
			throw std::out_of_range("Path range has to be at least 2");
		}
		//double maxDelta = (path[0].getRearPose() - path[1].getRearPose()).magnitude() / 2.0;
		double radius = (rearFromCenter(path[0],m_wheelbase) - rearFromCenter(path[1], m_wheelbase)).magnitude() * m_lookAhead;
		double maxDelta = (rearFromCenter(path[0], m_wheelbase) - rearFromCenter(path.back(), m_wheelbase)).magnitude() * 2;
		Angle maxSteering = m_maxSteeringAngle;
		//std::vector<VehicleState*> candidatePoses;
		const IVehicleState* withinRange = nullptr;
		Point normal(cos(vehiclePose.theta), sin(vehiclePose.theta));

		for (auto& state : path) {
			Pose pose = rearFromCenter(*state, m_wheelbase);
			double magnitude = (vehiclePose - pose).magnitude();
			Angle targetAngle = atan2(pose.y - vehiclePose.y, pose.x - vehiclePose.x);
			Angle angleDiff = targetAngle - vehiclePose.theta;
			Angle angleControl = atan(sin(angleDiff) * m_wheelbase * 2.0 / (Point(vehiclePose) - Point(pose)).magnitude());
			double distAlongHeading = normal.X() * (pose.x - vehiclePose.x) + normal.Y() * (pose.y - vehiclePose.y);

			bool isSteeringValid = std::abs(radian(angleControl)) <= radian(maxSteering);
			bool isDistanceValid = magnitude >= radius && magnitude <= maxDelta;
			bool isInFront = distAlongHeading > 0;

			if (isSteeringValid && isDistanceValid && isInFront) {
				withinRange = state.get(); 
				break;
			}
		}
		return withinRange;
	}
	Pose model::PurePursuitControl::rearFromCenter(model::IVehicleState const& center, double wheelbase)
	{
		double offset = wheelbase / 2;

		Pose deltaPose{
			offset * cos(center.getOrientation()),
			offset * sin(center.getOrientation()),
			0
		};

		return center.getPose() - deltaPose;
	}

	ControlCommand PurePursuitControl::drive(model::diffDrive::State const& state, model::Path const& path)
	{
		return ControlCommand();
	}

}