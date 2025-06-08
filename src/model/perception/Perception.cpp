#include "Perception.h"
#include "model/items/Cone.h"
#include "model/utils/Angle.h"
#include "model/utils/Pose.h"

model::Perception::Perception(std::vector<model::Cone> const& cones, double viewAngle, double depth) : _cones(cones), _viewAngle(viewAngle), _depth(depth)
{
}


// Based on detection params, return a pointer to cones which which fall within the range of the "camera"
std::vector<model::Cone const*> model::Perception::detect(model::Pose const& pose)
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
