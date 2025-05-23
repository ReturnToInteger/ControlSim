#include "Perception.h"

model::Perception::Perception(std::vector<model::Cone>& cones, double angle, double depth) : _cones(cones), _angle(angle), _depth(depth)
{
}

std::vector<model::Cone*> model::Perception::detect(const model::Point& position, double orientation)
{
	std::vector<model::Cone*> detectedCones;
	for (auto& cone : _cones) {
		model::Point relativePos = cone.getPosition() - position;
		if (relativePos.magnitude() <= _depth) {
			double angle = std::atan2(relativePos.Y(), relativePos.X());
			if (angle > orientation - _angle / 2 && angle < orientation + _angle / 2) {
				cone.setDetected(true);
				detectedCones.emplace_back(&cone);
			}
			else {
				cone.setDetected(false);
			}
		}
		else {
			cone.setDetected(false);
		}

	}
	return detectedCones;
}
