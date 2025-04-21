#pragma once

namespace model {
	class Point
	{
	private:
		double _x, _y;
	public:
		Point();
		Point(double x, double y);

		Point operator+(const Point& other) const;
		Point operator-(const Point& other) const;
		double operator*(const Point& other) const;
		Point operator*(const double& scale) const;
		Point& operator+=(const Point& other);

		double X() const;
		double Y() const;
		void setX(double x);
		void setY(double y);

	};
}