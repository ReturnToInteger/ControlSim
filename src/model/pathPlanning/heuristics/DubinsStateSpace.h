#pragma once
#include "model/utils/Point.h"

namespace model {
	class Point;
	class Pose;
	class Angle;
	namespace pathPlanning {
		enum Direction { LEFT, RIGHT };
		class DubinsStateSpace {
		public:
			DubinsStateSpace() : _turningRadius(0) {}
			DubinsStateSpace(const double& turningRadius) : _turningRadius(turningRadius) {}
			double distance(const model::Pose& start, const model::Pose& end);
			double simpleDistance(const model::Pose& start, const model::Point endPoint);
			Point rotatePoint(const Point& p, const Angle& a);

		private:
			double _turningRadius;
			// CSC distances
			double _RSR(const model::Pose& start, const model::Pose& end);
			double _LSL(const model::Pose& start, const model::Pose& end);
			double _LSR(const model::Pose& start, const model::Pose& end);
			double _RSL(const model::Pose& start, const model::Pose& end);
			// CCC distances
			double _RLR(const model::Pose& start, const model::Pose& end);
			double _LRL(const model::Pose& start, const model::Pose& end);
			// simple CS distance
			double _leftStraight(const model::Pose& start, const model::Point& endPoint);
			double _rightStraight(const model::Pose& start, const model::Point& endPoint);
			// Origins
			model::Point _leftOrigin(const model::Pose& p);
			model::Point _rightOrigin(const model::Pose& p);
			model::Point _startLeftOrig;
			model::Point _startRightOrig;
			model::Point _endLeftOrig;
			model::Point _endRightOrig;
			// Arc length
			double _arcLength(const model::Point& first, const model::Point& origin, const model::Point& second, Direction dir) const;
		};
	}
}
