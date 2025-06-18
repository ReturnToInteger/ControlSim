#include "DubinsStateSpace.h"  
//#include <array>
#include <vector>
#include <algorithm>
#include "model/utils/Pose.h"

namespace model {
    namespace pathPlanning {
        double DubinsStateSpace::distance(model::Pose const& start, model::Pose const& end)
        {
            //std::cout << "Start: " << start << std::endl;
            //std::cout << "End: " << end << std::endl;
            double relX = start.x - end.x, relY = start.y - end.y;

            _startLeftOrig = _leftOrigin(start);
            _startRightOrig = _rightOrigin(start);
            _endLeftOrig = _leftOrigin(end);
            _endRightOrig = _rightOrigin(end);

            double cost = _RSR(start, end);
            double min = cost;
            cost = _LSL(start, end);
            if (cost < min)
                min = cost;
            cost = _RSL(start, end);
            if (cost < min)
                min = cost;
            cost= _LSR(start, end);
            if (cost < min)
                min = cost;
            //// CCC only valid if distance is less 4r_min
            //if (relX * relX + relY * relY < 16 * _turningRadius * _turningRadius) {
            //    costs.emplace_back(_LRL(start, end));
            //    costs.emplace_back(_RLR(start, end));
            //}

            double distance = min;
            return distance;
        }

        double DubinsStateSpace::simpleDistance(model::Pose const& start, model::Point const& endPoint)
        {
            //std::pair<double, double> costs(1.7e308, 1.7e308);
            // Calc rightside and leftside turning min turning radius origin
            _startLeftOrig = _leftOrigin(start);
            _startRightOrig = _rightOrigin(start);

            // If goal is inside the turning radius, return some default value
            // Right side
            model::Point originToGoal = _startRightOrig - endPoint;
            if (originToGoal * originToGoal < _turningRadius * _turningRadius) return std::max(_arcLength(Point(start), _startRightOrig, endPoint, Direction::RIGHT),_turningRadius*M_PI);
            // Left side
            originToGoal = _startLeftOrig - endPoint;
            if (originToGoal * originToGoal < _turningRadius * _turningRadius) return std::max(_arcLength(Point(start), _startLeftOrig, endPoint, Direction::LEFT), _turningRadius*M_PI);
            // Get distances on both left and right directions, and return the minimum
            double costLeft = _leftStraight(start, endPoint);
            double costRight = _rightStraight(start, endPoint);
            return std::min(costLeft, costRight);
        }

        double DubinsStateSpace::multipleDistance(model::Pose const& start, model::Point const& wayPoint, model::Point const& endPoint)
        {
            double bestCost = INFINITY;
            double bestTheta = 0.0;
            double angleStep = M_PI / 8;

            for (double theta = 0; theta < 2 * M_PI; theta += angleStep) {
                Pose wpPose(wayPoint.X(), wayPoint.Y(), theta);

                double cost1 = distance(start, wpPose);
                double cost2 = simpleDistance(wpPose, endPoint);

                double total = cost1 + cost2;

                if (total < bestCost) {
                    bestCost = total;
                    bestTheta = theta;
                }
            }
            return bestCost;

        }

        model::Point DubinsStateSpace::rotatePoint(Point const& p, Angle const& a)
        {
            double c = cos(a);
            double s = sin(a);
            return Point(p.X() * c - p.Y() * s, p.X() * s + p.Y() * c);
        }

        double DubinsStateSpace::_RSR(model::Pose const& start, model::Pose const& end)
        {
            // Start: right origin, end: right origin
            model::Point originVect(_endRightOrig - _startRightOrig);
            double straightLength = (originVect).magnitude();
            model::Point normalVect(rotatePoint(originVect / straightLength, Angle(M_PI / 2.0)));
            model::Point firstTang = normalVect * _turningRadius + _startRightOrig;
            model::Point secondTang = normalVect * _turningRadius + _endRightOrig;

            double startArcLength = _arcLength(Point(start.x, start.y), _startRightOrig, firstTang, Direction::RIGHT);
            double endArcLength = _arcLength(secondTang, _endRightOrig, model::Point(end.x, end.y), Direction::RIGHT);
            return startArcLength + straightLength + endArcLength;
        }

        double DubinsStateSpace::_LSL(model::Pose const& start, model::Pose const& end)
        {
            // Start: left origin, end: left origin
            model::Point originVect(_endLeftOrig - _startLeftOrig);
            double straightLength = (originVect).magnitude();
            model::Point normalVect(rotatePoint(originVect / straightLength, Angle(-M_PI / 2.0)));
            model::Point firstTang = normalVect * _turningRadius + _startLeftOrig;
            model::Point secondTang = normalVect * _turningRadius + _endLeftOrig;

            double startArcLength = _arcLength(Point(start.x, start.y), _startLeftOrig, firstTang, Direction::LEFT);
            double endArcLength = _arcLength(secondTang, _endLeftOrig, model::Point(end.x, end.y), Direction::LEFT);
            return startArcLength + straightLength + endArcLength;
        }

        double DubinsStateSpace::_LSR(model::Pose const& start, model::Pose const& end)
        {
            // Start left, end right
            model::Point originVect(_endRightOrig - _startLeftOrig);
            double originDistance = originVect.magnitude();
            model::Point originVectNorm = originVect / originDistance;
            // V/D*n = 2r/D
            // nx*originVectNorm.X() + ny*originVectNorm.Y() =2*_turningRadius/originDistance;
            // nx*nx +ny*ny =1;
            double cosPhi = 2 * _turningRadius / originDistance; //from dot product rule
            cosPhi = std::clamp(cosPhi, -1.0, 1.0);
            double sinPhi = std::sqrt(1.0 - cosPhi * cosPhi);
            double nx = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi;
            double ny = originVectNorm.X() * sinPhi + originVectNorm.Y() * cosPhi;
            model::Point normalVect(nx, ny);
            model::Point firstTang = normalVect * _turningRadius + _startLeftOrig;
            model::Point secondTang = normalVect * (-1) * _turningRadius + _endRightOrig;

            double startArcLength = _arcLength(Point(start), _startLeftOrig, firstTang, Direction::LEFT);
            double straightLength = (secondTang - firstTang).magnitude();
            double endArcLength = _arcLength(secondTang, _endRightOrig, Point(end), Direction::RIGHT);

            return startArcLength + straightLength + endArcLength;
        }

        double DubinsStateSpace::_RSL(model::Pose const& start, model::Pose const& end)
        {
            // Start right, end left
            model::Point originVect(_endLeftOrig - _startRightOrig);
            double originDistance = originVect.magnitude();
            model::Point originVectNorm = originVect / originDistance;
            // V/D*n = 2r/D
            // nx*originVectNorm.X() + ny*originVectNorm.Y() =2*_turningRadius/originDistance;
            // nx*nx +ny*ny =1;
            double cosPhi = 2 * _turningRadius / originDistance; //from dot product rule
            cosPhi = std::clamp(cosPhi, -1.0, 1.0);
            double sinPhi = std::sqrt(1.0 - cosPhi * cosPhi);
            double nx = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi;
            double ny = originVectNorm.X() * sinPhi + originVectNorm.Y() * cosPhi;
            model::Point normalVect(nx, ny);
            model::Point firstTang = normalVect * _turningRadius + _startRightOrig;
            model::Point secondTang = normalVect * (-1) * _turningRadius + _endLeftOrig;

            double startArcLength = _arcLength(Point(start), _startRightOrig, firstTang, Direction::RIGHT);
            double straightLength = (secondTang - firstTang).magnitude();
            double endArcLength = _arcLength(secondTang, _endLeftOrig, Point(end), Direction::LEFT);

            return startArcLength + straightLength + endArcLength;
        }

        double DubinsStateSpace::_RLR(model::Pose const& start, model::Pose const& end)
        {
            return 1.7e308;
        }

        double DubinsStateSpace::_LRL(model::Pose const& start, model::Pose const& end)
        {
            return 1.7e308;
        }

        double DubinsStateSpace::_leftStraight(model::Pose const& start, model::Point const& endPoint)
        {
            // Start left, end: endPoint
            model::Point originVect(endPoint - _startLeftOrig);
            double originDistance = originVect.magnitude();
            model::Point originVectNorm = originVect / originDistance;
            // V/D*n = 2r/D
            // nx*originVectNorm.X() + ny*originVectNorm.Y() =_turningRadius/originDistance;
            // nx*nx +ny*ny =1;
            double cosPhi = _turningRadius / originDistance; //r1=_turninRadius, r2=0
            cosPhi = std::clamp(cosPhi, -1.0, 1.0);
            double sinPhi = std::sqrt(1.0 - cosPhi * cosPhi);
            double nx = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi;
            double ny = originVectNorm.X() * sinPhi + originVectNorm.Y() * cosPhi;
            model::Point normalVect(nx, ny);
            model::Point firstTang = normalVect * _turningRadius + _startLeftOrig;

            double sinPhi2 = -sinPhi;

            double nx2 = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi2;
            double ny2 = originVectNorm.X() * sinPhi2 + originVectNorm.Y() * cosPhi;
            model::Point normalVect2(nx2, ny2);
            model::Point secondTang = normalVect2 * _turningRadius + _startLeftOrig;

            double startArcLength = _arcLength(Point(start), _startLeftOrig, firstTang, Direction::LEFT);
            double startArcLength2 = _arcLength(Point(start), _startLeftOrig, secondTang, Direction::LEFT);
            double straightLength = (endPoint - firstTang).magnitude();

            return std::min(startArcLength, startArcLength2) + straightLength;

        }

        double DubinsStateSpace::_rightStraight(model::Pose const& start, model::Point const& endPoint)
        {
            // Start left, end: endPoint
            model::Point originVect(endPoint - _startRightOrig);
            double originDistance = originVect.magnitude();
            model::Point originVectNorm = originVect / originDistance;
            // V/D*n = 2r/D
            // nx*originVectNorm.X() + ny*originVectNorm.Y() =_turningRadius/originDistance;
            // nx*nx +ny*ny =1;
            double cosPhi = _turningRadius / originDistance; //r1=_turninRadius, r2=0
            cosPhi = std::clamp(cosPhi, -1.0, 1.0);
            double sinPhi = std::sqrt(1.0 - cosPhi * cosPhi);
            double nx = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi;
            double ny = originVectNorm.X() * sinPhi + originVectNorm.Y() * cosPhi;
            model::Point normalVect(nx, ny);
            model::Point firstTang = normalVect * _turningRadius + _startRightOrig;

            double sinPhi2 = -sinPhi;

            double nx2 = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi2;
            double ny2 = originVectNorm.X() * sinPhi2 + originVectNorm.Y() * cosPhi;
            model::Point normalVect2(nx2, ny2);
            model::Point secondTang = normalVect2 * _turningRadius + _startRightOrig;

            double startArcLength = _arcLength(Point(start), _startRightOrig, firstTang, Direction::RIGHT);
            double startArcLength2 = _arcLength(Point(start), _startRightOrig, secondTang, Direction::RIGHT);
            double straightLength = (endPoint - firstTang).magnitude();

            return std::min(startArcLength, startArcLength2) + straightLength;

        }


        model::Point DubinsStateSpace::_leftOrigin(model::Pose const& p)
        {
            Angle rightAng = p.theta + M_PI / 2.0;
            Point norm = rotatePoint(Point(1, 0), rightAng);
            return Point(p) + norm * _turningRadius;
        }

        model::Point DubinsStateSpace::_rightOrigin(model::Pose const& p)
        {
            Angle rightAng = p.theta - M_PI / 2.0;
            Point norm = rotatePoint(Point(1, 0), rightAng);
            return Point(p) + norm * _turningRadius;
        }

        double DubinsStateSpace::_arcLength(model::Point const& first, model::Point const& origin, model::Point const& second, Direction dir) const
        {
            Point pVec = first - origin;
            Point tVec = second - origin;
            double firstAtan = atan2(tVec.Y(), tVec.X());
            double secondAtan = atan2(pVec.Y(), pVec.X());
            double theta = firstAtan - secondAtan;
            if (theta < -1e-6 && dir == LEFT)
                theta += 2.0 * M_PI;
            else if (theta > 1e-6 && dir == RIGHT)
                theta -= 2.0 * M_PI;
            //double length = abs(theta * _turningRadius);
            return abs(theta * _turningRadius);
        }

    }
}