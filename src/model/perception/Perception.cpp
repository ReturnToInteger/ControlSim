#include "Perception.h"

model::Perception::Perception(const std::vector<model::Cone>& cones, double viewAngle, double depth) : _cones(cones), _viewAngle(viewAngle), _depth(depth)
{
}


std::vector<const model::Cone*> model::Perception::detect(const model::Pose & pose)
{
	std::vector<const model::Cone*> detectedCones;
	for (auto& cone : _cones) {
		model::Point relativePos(cone.getPosition().X()- pose.x, cone.getPosition().Y()-pose.y);
		if (relativePos.magnitude() <= _depth) {
			Angle angle= std::atan2(relativePos.Y(), relativePos.X());
			Angle theta = pose.theta;
			if (angle > theta - _viewAngle / 2 && angle < theta + _viewAngle / 2) {
				detectedCones.emplace_back(&cone);
			}
		}

	}
	return detectedCones;
}
