#pragma once
#include <ostream>

namespace model {
	struct Pose;
	class Point
	{
	private:
		double _x, _y;
	public:
		Point();
		Point(const double& x, const double& y);
		Point(const Pose& pose);

		Point operator+(const Point& other) const;
		Point operator-(const Point& other) const;
		double operator*(const Point& other) const;
		Point operator*(const double& scale) const;
		Point operator/(const double& scale) const;
		Point& operator+=(const Point& other);
		bool operator==(const Point& other) const;
		bool operator!=(const Point& other) const;
		friend std::ostream& operator<<(std::ostream& os, const Point& point);


		double X() const;
		double Y() const;
		void setX(double x);
		void setY(double y);
		double magnitude() const;

	};

}