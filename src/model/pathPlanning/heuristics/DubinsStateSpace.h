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
			DubinsStateSpace() : m_turningRadius(0) {}
			DubinsStateSpace(double const& turningRadius) : m_turningRadius(turningRadius) {}
			// Dubins distance
			double distance(model::Pose const& start, model::Pose const& end);
			// Simplified Dubins distance without known end orientation
			double simpleDistance(model::Pose const& start, model::Point const& endPoint);
			// Multiple waypoints, rough optimization by brute-force
			double multipleDistance(model::Pose const& start, model::Point const& wayPoint, model::Point const& endPoint);

			static Point rotatePoint(Point const& p, Angle const& a);
			static constexpr double angleDivision = 8;

		private:
			double m_turningRadius;
			// CSC distances
			double RSR(model::Pose const& start, model::Pose const& end);
			double LSL(model::Pose const& start, model::Pose const& end);
			double LSR(model::Pose const& start, model::Pose const& end);
			double RSL(model::Pose const& start, model::Pose const& end);
			// CCC distances
			double RLR(model::Pose const& start, model::Pose const& end);
			double LRL(model::Pose const& start, model::Pose const& end);
			// simple CS distance
			double leftStraight(model::Pose const& start, model::Point const& endPoint);
			double rightStraight(model::Pose const& start, model::Point const& endPoint);
			// Origins
			model::Point leftOrigin(model::Pose const& p) const;
			model::Point rightOrigin(model::Pose const& p) const;
			model::Point m_startLeftOrig;
			model::Point m_startRightOrig;
			model::Point m_endLeftOrig;
			model::Point m_endRightOrig;
			// Arc length
			double arcLength(model::Point const& first, model::Point const& origin, model::Point const& second, Direction dir) const;
		};
	}
}
