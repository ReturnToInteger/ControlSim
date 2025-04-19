#pragma once

namespace model {
	class Point
	{
	private:
		double _x, _y;
	public:
		Point(double x, double y);

		Point operator+(Point& other);
		Point operator+(Point& other);
		Point operator*(Point& other);

		double X();
		double Y();
		void setX(double x);
		void setY(double y);

	};
}