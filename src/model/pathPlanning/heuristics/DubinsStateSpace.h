#pragma once
#include "model/utils/Point.h"

namespace model {
	class Point;
	struct Pose;
	class Angle;
	namespace pathPlanning {
		enum Direction { LEFT, RIGHT };
		class DubinsStateSpace {
		public:
			DubinsStateSpace() : _turningRadius(0) {}
			DubinsStateSpace(double const& turningRadius) : _turningRadius(turningRadius) {}
			// Dubins distance
			double distance(model::Pose const& start, model::Pose const& end);
			// Simplified Dubins distance without known end orientation
			double simpleDistance(model::Pose const& start, model::Point const& endPoint);
			// Multiple waypoints, rough optimization by brute-force
			double multipleDistance(model::Pose const& start, model::Point const& wayPoint, model::Point const& endPoint);

			Point rotatePoint(Point const& p, Angle const& a);

		private:
			double _turningRadius;
			// CSC distances
			double _RSR(model::Pose const& start, model::Pose const& end);
			double _LSL(model::Pose const& start, model::Pose const& end);
			double _LSR(model::Pose const& start, model::Pose const& end);
			double _RSL(model::Pose const& start, model::Pose const& end);
			// CCC distances
			double _RLR(model::Pose const& start, model::Pose const& end);
			double _LRL(model::Pose const& start, model::Pose const& end);
			// simple CS distance
			double _leftStraight(model::Pose const& start, model::Point const& endPoint);
			double _rightStraight(model::Pose const& start, model::Point const& endPoint);
			// Origins
			model::Point _leftOrigin(model::Pose const& p);
			model::Point _rightOrigin(model::Pose const& p);
			model::Point _startLeftOrig;
			model::Point _startRightOrig;
			model::Point _endLeftOrig;
			model::Point _endRightOrig;
			// Arc length
			double _arcLength(model::Point const& first, model::Point const& origin, model::Point const& second, Direction dir) const;
		};
	}
}
