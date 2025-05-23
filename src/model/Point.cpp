#include "Point.h"
namespace model
{
	double Point::X() const
	{
		return _x;
	}
	double Point::Y() const
	{
		return _y;
	}
	void Point::setX(double x)
	{
		_x = x;
	}
	void Point::setY(double y)
	{
		_y = y;
	}
	double Point::magnitude() const
	{
		return std::sqrt(_x * _x + _y * _y);
	}
	Point::Point()
		: _x(0), _y(0)
	{
	}
	Point::Point(double x, double y)
	{
		setX(x);
		setY(y);
	}

	Point Point::operator+(const Point& other) const
	{
		return Point(X() + other.X(), Y() + other.Y());
	}
	Point Point::operator-(const Point& other) const
	{
		return Point(X() - other.X(), Y() - other.Y());
	}
	double Point::operator*(const Point& other) const
	{
		return X() * other.X()+ Y() * other.Y();
	}

	Point Point::operator*(const double& scale) const
	{
		return Point(X() * scale, Y() * scale);
	}

	Point& Point::operator+=(const Point& other)
	{
		_x += other._x;
		_y += other._y;
		return *this;
	}

	bool Point::operator==(const Point& other) const
	{
		return _x == other._x && _y == other._y;
	}

} 
