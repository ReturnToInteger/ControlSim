#pragma once
#include <ostream>

namespace model {
	struct Pose;
	class Point
	{
	private:
		double m_x, m_y;
	public:
		Point();
		Point(double const& x, double const& y);
		Point(Pose const& pose);

		Point operator+(Point const& other) const;
		Point operator-(Point const& other) const;
		double operator*(Point const& other) const;
		Point operator*(double const& scale) const;
		Point operator/(double const& scale) const;
		Point& operator+=(Point const& other);
		bool operator==(Point const& other) const;
		bool operator!=(Point const& other) const;
		friend std::ostream& operator<<(std::ostream& os, Point const& point);


		double X() const;
		double Y() const;
		void setX(double x);
		void setY(double y);
		double magnitude() const;

	};

}

// Non-member operator to support double * Point
inline model::Point operator*(double val, model::Point const& point)
{
	return point*val;
}
