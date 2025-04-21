#pragma once 
#include <array>
#include <stdexcept>
 
namespace model { 
class Vec3 { 
public:
	Vec3() : _data{ 0.0, 0.0, 0.0 } {}
	Vec3(double x, double y, double z) : _data{ x, y, z } {}
	Vec3(const Vec3& other) : _data{ other._data } {}
	Vec3& operator=(const Vec3& other) {
		if (this != &other) {
			_data = other._data;
		}
		return *this;
	}
	Vec3 operator+(const Vec3& other) const {
		return Vec3(_data[0] + other._data[0], _data[1] + other._data[1], _data[2] + other._data[2]);
	}
	Vec3 operator-(const Vec3& other) const {
		return Vec3(_data[0] - other._data[0], _data[1] - other._data[1], _data[2] - other._data[2]);
	}
	Vec3& operator+=(const Vec3& other) {
		_data[0] += other._data[0];
		_data[1] += other._data[1];
		_data[2] += other._data[2];
		return *this;
	}
	double X() const { return _data[0]; }
	double Y() const { return _data[1]; }
	double Z() const { return _data[2]; }
	void setX(double x) { _data[0] = x; }
	void setY(double y) { _data[1] = y; }
	void setZ(double z) { _data[2] = z; }

	double& operator[](size_t index) {
		if (index >= _size || index < 0) {
			throw std::out_of_range("Index out of range");
		}
		return _data[index];
	}
 
private: 
    static constexpr int _size=3;
	std::array<double, _size> _data;
 
};
} // namespace model 
