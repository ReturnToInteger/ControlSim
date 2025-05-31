#include "DubinsSpace.h"  
#include <array> 
#include <algorithm>

double model::DubinsStateSpace::distance(const model::Pose& start, const model::Pose& end) 
{
    //std::cout << "Start: " << start << std::endl;
    //std::cout << "End: " << end << std::endl;
    double relX = start.x - end.x, relY= start.y - end.y;
    std::vector<double> costs;
    costs.reserve(6);

    _startLeftOrig = _leftOrigin(start);
    _startRightOrig = _rightOrigin(start);
    _endLeftOrig = _leftOrigin(end);
    _endRightOrig = _rightOrigin(end);

    costs.emplace_back(_RSR(start, end));
    costs.emplace_back(_LSL(start, end));
    costs.emplace_back(_RSL(start, end));
    costs.emplace_back(_LSR(start, end));
    // CCC only valid if distance is less 4r_min
    if (relX * relX + relY * relY < 16*_turningRadius * _turningRadius) {
        costs.emplace_back(_LRL(start, end));
        costs.emplace_back(_RLR(start, end));
    }

    double distance = *std::min_element(costs.begin(), costs.end());
    return distance;
}

double model::DubinsStateSpace::simpleDistance(const model::Pose& start, const model::Point endPoint)
{
    std::pair<double,double> costs(1.7e308,1.7e308);
    _startLeftOrig = _leftOrigin(start);
    _startRightOrig = _rightOrigin(start);
    model::Point originToEnd = _startRightOrig - endPoint;
    if (originToEnd * originToEnd < _turningRadius * _turningRadius) return _arcLength(Point(start),_startRightOrig,endPoint, Direction::RIGHT);
    originToEnd = _startLeftOrig - endPoint;
    if (originToEnd * originToEnd < _turningRadius * _turningRadius) return _arcLength(Point(start), _startLeftOrig, endPoint, Direction::LEFT);
    costs.first = _leftStraight(start, endPoint);
    costs.second = _rightStraight(start, endPoint);
    return std::min(costs.first, costs.second);
}

model::Point model::DubinsStateSpace::rotatePoint(const Point& p, const Angle& a)
{
    double c = cos(a);
    double s = sin(a);
    return Point(p.X() * c - p.Y() * s, p.X() * s + p.Y() * c);
}

double model::DubinsStateSpace::_RSR(const model::Pose& start, const model::Pose& end)  
{  
    // Start: right origin, end: right origin
    model::Point originVect(_endRightOrig - _startRightOrig);
    double straightLength = (originVect).magnitude();
    model::Point normalVect(rotatePoint(originVect/straightLength,Angle(M_PI/2.0)));
    model::Point firstTang = normalVect * _turningRadius + _startRightOrig;
    model::Point secondTang = normalVect * _turningRadius + _endRightOrig;

    double startArcLength = _arcLength(Point(start.x, start.y), _startRightOrig, firstTang, Direction::RIGHT);
    double endArcLength= _arcLength(secondTang, _endRightOrig,model::Point(end.x, end.y), Direction::RIGHT);
  return startArcLength+straightLength+endArcLength;
}  

double model::DubinsStateSpace::_LSL(const model::Pose& start, const model::Pose& end)  
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

double model::DubinsStateSpace::_LSR(const model::Pose& start, const model::Pose& end)  
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
    model::Point secondTang= normalVect*(-1) * _turningRadius + _endRightOrig;

    double startArcLength = _arcLength(Point(start), _startLeftOrig, firstTang, Direction::LEFT);
    double straightLength = (secondTang - firstTang).magnitude();
    double endArcLength = _arcLength(secondTang, _endRightOrig, Point(end), Direction::RIGHT);

    return startArcLength+straightLength+endArcLength;
}  

double model::DubinsStateSpace::_RSL(const model::Pose& start, const model::Pose& end)  
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

double model::DubinsStateSpace::_RLR(const model::Pose& start, const model::Pose& end)  
{  
  return 1.7e308;  
}  

double model::DubinsStateSpace::_LRL(const model::Pose& start, const model::Pose& end)  
{  
  return 1.7e308;  
}

double model::DubinsStateSpace::_leftStraight(const model::Pose& start, const model::Point& endPoint)
{
    // Start left, end: endPoint
    model::Point originVect(endPoint- _startLeftOrig);
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

    return std::min(startArcLength,startArcLength2) + straightLength;

}

double model::DubinsStateSpace::_rightStraight(const model::Pose& start, const model::Point& endPoint)
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


model::Point model::DubinsStateSpace::_leftOrigin(const model::Pose& p)
{
    Angle rightAng = p.theta + M_PI / 2.0;
    Point norm = rotatePoint(Point(1, 0), rightAng);
    return Point(p) +norm*_turningRadius;
}

model::Point model::DubinsStateSpace::_rightOrigin(const model::Pose& p)
{
    Angle rightAng = p.theta - M_PI / 2.0;
    Point norm = rotatePoint(Point(1, 0), rightAng);
    return Point(p) + norm * _turningRadius;
}

double model::DubinsStateSpace::_arcLength(const model::Point& first, const model::Point& origin, const model::Point& second, Direction dir) const
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
    return abs(theta*_turningRadius);
}

