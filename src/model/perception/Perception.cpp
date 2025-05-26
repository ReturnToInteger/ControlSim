#include "Perception.h"

model::Perception::Perception(const std::vector<model::Cone>& cones, double angle, double depth) : _cones(cones), _angle(angle), _depth(depth)
{
}


std::vector<const model::Cone*> model::Perception::detect(const model::Pose & pose)
{
	std::vector<const model::Cone*> detectedCones;
	for (auto& cone : _cones) {
		model::Point relativePos(cone.getPosition().X()- pose.x, cone.getPosition().Y()-pose.y);
		if (relativePos.magnitude() <= _depth) {
			double angle = std::atan2(relativePos.Y(), relativePos.X());
			if (angle > pose.theta - _angle / 2 && angle < pose.theta + _angle / 2) {
				detectedCones.emplace_back(&cone);
			}
		}

	}
	return detectedCones;
}
