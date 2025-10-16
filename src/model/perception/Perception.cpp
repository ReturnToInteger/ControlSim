#include "Perception.h"
#include "model/items/obstacles/Cone.h"
#include "model/utils/Angle.h"
#include "model/utils/Pose.h"
#ifdef ENABLE_DEBUG_DRAW
#include "view/DebugDraw.h"
#endif // ENABLE_DEBUG_DRAW


model::Perception::Perception(model::Map const & map, Angle m_viewAngle, double depth) : m_map(map), m_viewAngle(m_viewAngle), m_depth(depth)
{
}


// Based on detection params, return a pointer to cones which which fall within the range of the "camera"
std::unordered_set<model::Obstacle const*> model::Perception::detect(model::Pose const& pose)
{
	std::unordered_set<const model::Obstacle*> detectedObjects;
	for (const auto& obstacle : m_map.getObstacles()) {
		model::Point relativePos(obstacle->getPosition().X()- pose.x, obstacle->getPosition().Y()-pose.y);
		if (relativePos.magnitude() <= m_depth) {
			Angle angle= std::atan2(relativePos.Y(), relativePos.X());
			Angle theta = pose.theta;
			if (angle.isClockwiseTo(theta - m_viewAngle / 2) && angle.isCounterClockwiseTo(theta + m_viewAngle / 2)) {
				detectedObjects.emplace(obstacle.get());
			}
		}
		//detectedCones.emplace_back(&cone);
	}
#ifdef ENABLE_DEBUG_DRAW
	std::vector<model::Point> debugDraw;
	debugDraw.emplace_back(pose);
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + m_viewAngle / 2) * m_depth, sin(pose.theta + m_viewAngle / 2) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + m_viewAngle * 5 / 12) * m_depth, sin(pose.theta + m_viewAngle * 5 / 12) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + m_viewAngle / 3) * m_depth, sin(pose.theta + m_viewAngle / 3) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + m_viewAngle / 4) * m_depth, sin(pose.theta + m_viewAngle / 4) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + m_viewAngle / 6) * m_depth, sin(pose.theta + m_viewAngle / 6) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + m_viewAngle / 12) * m_depth, sin(pose.theta + m_viewAngle / 12) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(model::cos(pose.theta) * m_depth, model::sin(pose.theta) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - m_viewAngle / 12) * m_depth, sin(pose.theta - m_viewAngle / 12) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - m_viewAngle / 6) * m_depth, sin(pose.theta - m_viewAngle / 6) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - m_viewAngle / 4) * m_depth, sin(pose.theta - m_viewAngle / 4) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - m_viewAngle / 3) * m_depth, sin(pose.theta - m_viewAngle / 3) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - m_viewAngle * 5 / 12) * m_depth, sin(pose.theta - m_viewAngle * 5 / 12) * m_depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - m_viewAngle / 2) * m_depth, sin(pose.theta - m_viewAngle / 2) * m_depth));
	debugDraw.emplace_back(pose);
	view::DebugDraw::instance().lineStrip(debugDraw);
#endif // ENABLE_DEBUG_DRAW

	return detectedObjects;
}
