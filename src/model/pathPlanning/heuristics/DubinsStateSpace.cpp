#include "DubinsStateSpace.h"  
//#include <array>
#include <vector>
#include <algorithm>
#include "model/utils/Pose.h"
#include <numbers>

namespace model::pathPlanning {
    double DubinsStateSpace::distance(model::Pose const& start, model::Pose const& end)
    {
        //std::cout << "Start: " << start << "\n";
        //std::cout << "End: " << end << "\n";

        m_startLeftOrig = leftOrigin(start);
        m_startRightOrig = rightOrigin(start);
        m_endLeftOrig = leftOrigin(end);
        m_endRightOrig = rightOrigin(end);

        double cost = RSR(start, end);
        double minDistance = cost;
        cost = LSL(start, end);
        minDistance = std::min(minDistance, cost);
        cost = RSL(start, end);
        minDistance = std::min(minDistance, cost);
        cost= LSR(start, end);
        minDistance = std::min(minDistance, cost);
        //// CCC only valid if distance is less 4r_min
        //double relX = start.x - end.x; 
        //double relY = start.y - end.y;
        //if (relX * relX + relY * relY < 16 * m_turningRadius * m_turningRadius) {
        //    costs.emplace_back(LRL(start, end));
        //    costs.emplace_back(RLR(start, end));
        //}

        return minDistance;
    }

    double DubinsStateSpace::simpleDistance(model::Pose const& start, model::Point const& endPoint)
    {
        //std::pair<double, double> costs(1.7e308, 1.7e308);
        // Calc rightside and leftside turning min turning radius origin
        m_startLeftOrig = leftOrigin(start);
        m_startRightOrig = rightOrigin(start);

        // If goal is inside the turning radius, return some default value
        // Right side
        model::Point originToGoal = m_startRightOrig - endPoint;
        if (originToGoal * originToGoal < m_turningRadius * m_turningRadius)
        {
            return std::max(arcLength(Point(start), m_startRightOrig, endPoint, Direction::RIGHT), m_turningRadius * std::numbers::pi);
        }
        // Left side
        originToGoal = m_startLeftOrig - endPoint;
        if (originToGoal * originToGoal < m_turningRadius * m_turningRadius) 
        {
            return std::max(arcLength(Point(start), m_startLeftOrig, endPoint, Direction::LEFT), m_turningRadius * std::numbers::pi);
        }
        // Get distances on both left and right directions, and return the minimum
        double costLeft = leftStraight(start, endPoint);
        double costRight = rightStraight(start, endPoint);
        return std::min(costLeft, costRight);
    }

    double DubinsStateSpace::multipleDistance(model::Pose const& start, model::Point const& wayPoint, model::Point const& endPoint)
    {
        double bestCost = INFINITY;
        //double bestTheta = 0.0;
        double angleStep = std::numbers::pi / angleDivision;

        for (int i = 0; i < angleDivision; i++) {
            double theta = angleStep * i;
            Pose wpPose(wayPoint.X(), wayPoint.Y(), theta);

            double cost1 = distance(start, wpPose);
            double cost2 = simpleDistance(wpPose, endPoint);

            double total = cost1 + cost2;

            if (total < bestCost) {
                bestCost = total;
                //bestTheta = theta;
            }
        }
        return bestCost;

    }

    model::Point DubinsStateSpace::rotatePoint(Point const& p, Angle const& a)
    {
        double c = cos(a);
        double s = sin(a);
        return { p.X() * c - p.Y() * s, p.X() * s + p.Y() * c };
    }

    double DubinsStateSpace::RSR(model::Pose const& start, model::Pose const& end)
    {
        // Start: right origin, end: right origin
        model::Point originVect(m_endRightOrig - m_startRightOrig);
        double straightLength = (originVect).magnitude();
        model::Point normalVect(rotatePoint(originVect / straightLength, Angle(std::numbers::pi / 2.0)));
        model::Point firstTang = normalVect * m_turningRadius + m_startRightOrig;
        model::Point secondTang = normalVect * m_turningRadius + m_endRightOrig;

        double startArcLength = arcLength(Point(start.x, start.y), m_startRightOrig, firstTang, Direction::RIGHT);
        double endArcLength = arcLength(secondTang, m_endRightOrig, model::Point(end.x, end.y), Direction::RIGHT);
        return startArcLength + straightLength + endArcLength;
    }

    double DubinsStateSpace::LSL(model::Pose const& start, model::Pose const& end)
    {
        // Start: left origin, end: left origin
        model::Point originVect(m_endLeftOrig - m_startLeftOrig);
        double straightLength = (originVect).magnitude();
        model::Point normalVect(rotatePoint(originVect / straightLength, Angle(-std::numbers::pi / 2.0)));
        model::Point firstTang = normalVect * m_turningRadius + m_startLeftOrig;
        model::Point secondTang = normalVect * m_turningRadius + m_endLeftOrig;

        double startArcLength = arcLength(Point(start.x, start.y), m_startLeftOrig, firstTang, Direction::LEFT);
        double endArcLength = arcLength(secondTang, m_endLeftOrig, model::Point(end.x, end.y), Direction::LEFT);
        return startArcLength + straightLength + endArcLength;
    }

    double DubinsStateSpace::LSR(model::Pose const& start, model::Pose const& end)
    {
        // Start left, end right
        model::Point originVect(m_endRightOrig - m_startLeftOrig);
        double originDistance = originVect.magnitude();
        model::Point originVectNorm = originVect / originDistance;
        // V/D*n = 2r/D
        // nx*originVectNorm.X() + ny*originVectNorm.Y() =2*m_turningRadius/originDistance;
        // nx*nx +ny*ny =1;
        double cosPhi = 2 * m_turningRadius / originDistance; //from dot product rule
        cosPhi = std::clamp(cosPhi, -1.0, 1.0);
        double sinPhi = std::sqrt(1.0 - cosPhi * cosPhi);
        double nx = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi;
        double ny = originVectNorm.X() * sinPhi + originVectNorm.Y() * cosPhi;
        model::Point normalVect(nx, ny);
        model::Point firstTang = normalVect * m_turningRadius + m_startLeftOrig;
        model::Point secondTang = normalVect * (-1) * m_turningRadius + m_endRightOrig;

        double startArcLength = arcLength(Point(start), m_startLeftOrig, firstTang, Direction::LEFT);
        double straightLength = (secondTang - firstTang).magnitude();
        double endArcLength = arcLength(secondTang, m_endRightOrig, Point(end), Direction::RIGHT);

        return startArcLength + straightLength + endArcLength;
    }

    double DubinsStateSpace::RSL(model::Pose const& start, model::Pose const& end)
    {
        // Start right, end left
        model::Point originVect(m_endLeftOrig - m_startRightOrig);
        double originDistance = originVect.magnitude();
        model::Point originVectNorm = originVect / originDistance;
        // V/D*n = 2r/D
        // nx*originVectNorm.X() + ny*originVectNorm.Y() =2*m_turningRadius/originDistance;
        // nx*nx +ny*ny =1;
        double cosPhi = 2 * m_turningRadius / originDistance; //from dot product rule
        cosPhi = std::clamp(cosPhi, -1.0, 1.0);
        double sinPhi = std::sqrt(1.0 - cosPhi * cosPhi);
        double nx = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi;
        double ny = originVectNorm.X() * sinPhi + originVectNorm.Y() * cosPhi;
        model::Point normalVect(nx, ny);
        model::Point firstTang = normalVect * m_turningRadius + m_startRightOrig;
        model::Point secondTang = normalVect * (-1) * m_turningRadius + m_endLeftOrig;

        double startArcLength = arcLength(Point(start), m_startRightOrig, firstTang, Direction::RIGHT);
        double straightLength = (secondTang - firstTang).magnitude();
        double endArcLength = arcLength(secondTang, m_endLeftOrig, Point(end), Direction::LEFT);

        return startArcLength + straightLength + endArcLength;
    }

    double DubinsStateSpace::RLR(model::Pose const& start, model::Pose const& end)
    {
        return 1.7e308;
    }

    double DubinsStateSpace::LRL(model::Pose const& start, model::Pose const& end)
    {
        return 1.7e308;
    }

    double DubinsStateSpace::leftStraight(model::Pose const& start, model::Point const& endPoint)
    {
        // Start left, end: endPoint
        model::Point originVect(endPoint - m_startLeftOrig);
        double originDistance = originVect.magnitude();
        model::Point originVectNorm = originVect / originDistance;
        // V/D*n = 2r/D
        // nx*originVectNorm.X() + ny*originVectNorm.Y() =m_turningRadius/originDistance;
        // nx*nx +ny*ny =1;
        double cosPhi = m_turningRadius / originDistance; //r1=m_turninRadius, r2=0
        cosPhi = std::clamp(cosPhi, -1.0, 1.0);
        double sinPhi = std::sqrt(1.0 - cosPhi * cosPhi);
        double nx = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi;
        double ny = originVectNorm.X() * sinPhi + originVectNorm.Y() * cosPhi;
        model::Point normalVect(nx, ny);
        model::Point firstTang = normalVect * m_turningRadius + m_startLeftOrig;

        double sinPhi2 = -sinPhi;

        double nx2 = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi2;
        double ny2 = originVectNorm.X() * sinPhi2 + originVectNorm.Y() * cosPhi;
        model::Point normalVect2(nx2, ny2);
        model::Point secondTang = normalVect2 * m_turningRadius + m_startLeftOrig;

        double startArcLength = arcLength(Point(start), m_startLeftOrig, firstTang, Direction::LEFT);
        double startArcLength2 = arcLength(Point(start), m_startLeftOrig, secondTang, Direction::LEFT);
        double straightLength = (endPoint - firstTang).magnitude();

        return std::min(startArcLength, startArcLength2) + straightLength;

    }

    double DubinsStateSpace::rightStraight(model::Pose const& start, model::Point const& endPoint)
    {
        // Start left, end: endPoint
        model::Point originVect(endPoint - m_startRightOrig);
        double originDistance = originVect.magnitude();
        model::Point originVectNorm = originVect / originDistance;
        // V/D*n = 2r/D
        // nx*originVectNorm.X() + ny*originVectNorm.Y() =m_turningRadius/originDistance;
        // nx*nx +ny*ny =1;
        double cosPhi = m_turningRadius / originDistance; //r1=m_turninRadius, r2=0
        cosPhi = std::clamp(cosPhi, -1.0, 1.0);
        double sinPhi = std::sqrt(1.0 - cosPhi * cosPhi);
        double nx = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi;
        double ny = originVectNorm.X() * sinPhi + originVectNorm.Y() * cosPhi;
        model::Point normalVect(nx, ny);
        model::Point firstTang = normalVect * m_turningRadius + m_startRightOrig;

        double sinPhi2 = -sinPhi;

        double nx2 = originVectNorm.X() * cosPhi - originVectNorm.Y() * sinPhi2;
        double ny2 = originVectNorm.X() * sinPhi2 + originVectNorm.Y() * cosPhi;
        model::Point normalVect2(nx2, ny2);
        model::Point secondTang = normalVect2 * m_turningRadius + m_startRightOrig;

        double startArcLength = arcLength(Point(start), m_startRightOrig, firstTang, Direction::RIGHT);
        double startArcLength2 = arcLength(Point(start), m_startRightOrig, secondTang, Direction::RIGHT);
        double straightLength = (endPoint - firstTang).magnitude();

        return std::min(startArcLength, startArcLength2) + straightLength;

    }


    model::Point DubinsStateSpace::leftOrigin(model::Pose const& p) const
    {
        Angle rightAng = p.theta + std::numbers::pi / 2.0;
        Point norm = rotatePoint(Point(1, 0), rightAng);
        return Point(p) + norm * m_turningRadius;
    }

    model::Point DubinsStateSpace::rightOrigin(model::Pose const& p) const
    {
        Angle rightAng = p.theta - std::numbers::pi / 2.0;
        Point norm = rotatePoint(Point(1, 0), rightAng);
        return Point(p) + norm * m_turningRadius;
    }

    double DubinsStateSpace::arcLength(model::Point const& first, model::Point const& origin, model::Point const& second, Direction dir) const
    {
        Point pVec = first - origin;
        Point tVec = second - origin;
        double firstAtan = atan2(tVec.Y(), tVec.X());
        double secondAtan = atan2(pVec.Y(), pVec.X());
        double theta = firstAtan - secondAtan;
        if (theta < -1e-6 && dir == LEFT)
        {
            theta += 2.0 * std::numbers::pi;
        }
        else if (theta > 1e-6 && dir == RIGHT)
        {
            theta -= 2.0 * std::numbers::pi;
        }
        //double length = abs(theta * m_turningRadius);
        return abs(theta * m_turningRadius);
    }

}