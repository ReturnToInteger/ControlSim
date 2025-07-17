#pragma once 
#include <array>
#include <stdexcept>
 
namespace model { 
class Vec3 { 
public:
	Vec3() : m_data{ 0.0, 0.0, 0.0 } {}
	Vec3(double x, double y, double z) : m_data{ x, y, z } {}
	Vec3(Vec3 const& other) : m_data{ other.m_data } {}
	Vec3& operator=(Vec3 const& other) {
		if (this != &other) {
			m_data = other.m_data;
		}
		return *this;
	}
	Vec3 operator+(Vec3 const& other) const {
		return Vec3(m_data[0] + other.m_data[0], m_data[1] + other.m_data[1], m_data[2] + other.m_data[2]);
	}
	Vec3 operator-(Vec3 const& other) const {
		return Vec3(m_data[0] - other.m_data[0], m_data[1] - other.m_data[1], m_data[2] - other.m_data[2]);
	}
	Vec3& operator+=(Vec3 const& other) {
		m_data[0] += other.m_data[0];
		m_data[1] += other.m_data[1];
		m_data[2] += other.m_data[2];
		return *this;
	}
	double X() const { return m_data[0]; }
	double Y() const { return m_data[1]; }
	double Z() const { return m_data[2]; }
	void setX(double x) { m_data[0] = x; }
	void setY(double y) { m_data[1] = y; }
	void setZ(double z) { m_data[2] = z; }

	double& operator[](size_t index) {
		if (index >= m_size || index < 0) {
			throw std::out_of_range("Index out of range");
		}
		return m_data[index];
	}
 
private: 
    static constexpr int m_size=3;
	std::array<double, m_size> m_data;
 
};
} // namespace model 
