#include "model/utils/Point.h"
#include "model/utils/Pose.h"
#include <cmath> 
namespace model
{
	double Point::X() const
	{
		return m_x;
	}
	double Point::Y() const
	{
		return m_y;
	}
	void Point::setX(double x)
	{
		m_x = x;
	}
	void Point::setY(double y)
	{
		m_y = y;
	}
	double Point::magnitude() const
	{
		return std::sqrt(m_x * m_x + m_y * m_y);
	}
	Point::Point()
		: m_x(0), m_y(0)
	{
	}
	Point::Point(double const& x, double const& y) : m_x(x),m_y(y)
	{
	}

	model::Point::Point(Pose const& pose) : m_x(pose.x), m_y(pose.y)
	{
	}

	Point Point::operator+(Point const& other) const
	{
		return { X() + other.X(), Y() + other.Y() };
	}
	Point Point::operator-(Point const& other) const
	{
		return { X() - other.X(), Y() - other.Y() };
	}
	double Point::operator*(Point const& other) const
	{
		return X() * other.X()+ Y() * other.Y();
	}

	Point Point::operator*(double const& scale) const
	{
		return { X() * scale, Y() * scale };
	}

	Point Point::operator/(double const& scale) const
	{
		return { X() / scale, Y() / scale };
	}

	Point& Point::operator+=(Point const& other)
	{
		m_x += other.m_x;
		m_y += other.m_y;
		return *this;
	}

	bool Point::operator==(Point const& other) const
	{
		return abs(m_x - other.m_x) < 1e-8 && abs(m_y - other.m_y) < 1e-8;
	}

	bool Point::operator!=(Point const& other) const
	{
		return abs(m_x - other.m_x) > 1e-8 || abs(m_y - other.m_y) > 1e-8;
	}

	std::ostream& operator<<(std::ostream& os, Point const& point)
	{
		os <<"\n"<< "x: " << point.m_x<<"\n"<<"y: "<<point.m_y; 
		return os;
	}

}
