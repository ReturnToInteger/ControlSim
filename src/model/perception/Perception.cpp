#include "Perception.h"
#include "model/items/Cone.h"
#include "model/utils/Angle.h"
#include "model/utils/Pose.h"
#ifdef ENABLE_DEBUG_DRAW
#include "view/DebugDraw.h"
#endif // ENABLE_DEBUG_DRAW


model::Perception::Perception(std::vector<model::Cone> const& cones, Angle _viewAngle, double depth) : _cones(cones), _viewAngle(_viewAngle), _depth(depth)
{
}


// Based on detection params, return a pointer to cones which which fall within the range of the "camera"
std::unordered_set<model::Cone const*> model::Perception::detect(model::Pose const& pose)
{
	std::unordered_set<const model::Cone*> detectedCones;
	for (auto& cone : _cones) {
		model::Point relativePos(cone.getPosition().X()- pose.x, cone.getPosition().Y()-pose.y);
		if (relativePos.magnitude() <= _depth) {
			Angle angle= std::atan2(relativePos.Y(), relativePos.X());
			Angle theta = pose.theta;
			if (angle.isClockwiseTo(theta - _viewAngle / 2) && angle.isCounterClockwiseTo(theta + _viewAngle / 2)) {
				detectedCones.emplace(&cone);
			}
		}
		//detectedCones.emplace_back(&cone);
	}
#ifdef ENABLE_DEBUG_DRAW
	std::vector<model::Point> debugDraw;
	debugDraw.emplace_back(Point(pose));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + _viewAngle / 2) * _depth, sin(pose.theta + _viewAngle / 2) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + _viewAngle * 5 / 12) * _depth, sin(pose.theta + _viewAngle * 5 / 12) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + _viewAngle / 3) * _depth, sin(pose.theta + _viewAngle / 3) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + _viewAngle / 4) * _depth, sin(pose.theta + _viewAngle / 4) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + _viewAngle / 6) * _depth, sin(pose.theta + _viewAngle / 6) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta + _viewAngle / 12) * _depth, sin(pose.theta + _viewAngle / 12) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(model::cos(pose.theta) * _depth, model::sin(pose.theta) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - _viewAngle / 12) * _depth, sin(pose.theta - _viewAngle / 12) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - _viewAngle / 6) * _depth, sin(pose.theta - _viewAngle / 6) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - _viewAngle / 4) * _depth, sin(pose.theta - _viewAngle / 4) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - _viewAngle / 3) * _depth, sin(pose.theta - _viewAngle / 3) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - _viewAngle * 5 / 12) * _depth, sin(pose.theta - _viewAngle * 5 / 12) * _depth));
	debugDraw.emplace_back(Point(pose) + Point(cos(pose.theta - _viewAngle / 2) * _depth, sin(pose.theta - _viewAngle / 2) * _depth));
	debugDraw.emplace_back(Point(pose));
	view::DebugDraw::instance().lineStrip(debugDraw);
#endif // ENABLE_DEBUG_DRAW

	return detectedCones;
}
