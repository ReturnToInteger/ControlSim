#pragma once
#include "Item.h"
#include <memory>
#include "Vec3.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846 // Define M_PI if not already defined
#endif

#ifndef DEF_LENGTH
#define DEF_LENGTH 1.5
#endif // !DEF_LENGTH

#ifndef DEF_WIDTH
#define DEF_WIDTH 0.5
#endif // !DEF_WIDTH

#ifndef DEF_MAX_STEERING_ANGLE
#define DEF_MAX_STEERING_ANGLE M_PI / 6.0
#endif // !DEF_MAX_STEERING_ANGLE
#ifndef DEF_MAX_STEERING_RATE
#define DEF_MAX_STEERING_RATE M_PI / 3.0
#endif // !DEF_MAX_STEERING_RATE
#ifndef DEF_MAX_SPEED
#define DEF_MAX_SPEED 5.0
#endif // !DEF_MAX_SPEED

namespace model {
    class Vehicle : public Item {
    public:
        Vehicle();
		Vehicle(Point startPos, double orientation = 0.0, double speed = 0.0);
		Vehicle(double positionX, double positionY, double orientation = 0.0, double speed = 0.0);

		//Vehicle(double positionX, double positionY, double orientation, double speed, double maxSpeed=DEF_MAX_SPEED, double length=DEF_LENGTH, double width=DEF_WIDTH,
			 //double maxSteeringAngle=DEF_MAX_STEERING_ANGLE, double maxSteeringRate=DEF_MAX_STEERING_RATE);

        void updateState(double dt);
		void steer(double rate);
		void accelerateTo(double speed);
		void sendInput(double speed, double steeringRate);
		std::array<Point, 3> getAllPositions() const;
		std::array<double, 3> getAllOrientations() const;
		Point getPosition() const override;
		double getOrientation() const;
		std::array<double, 6> getCenter() const;
		std::array<double, 6> getFront() const;
		std::array<double, 6> getRear() const;
		double getSpeed() const { return _speed; }
		double getSteeringAngle() const { return _steeringAngle; }
		double getSteeringRate() const { return _steeringRate; }
		double getLength() const { return _length; }
		double getWidth() const { return _width; }
		double getMaxSteeringAngle() const { return _maxSteeringAngle; }
		double getMaxSteeringRate() const { return _maxSteeringRate; }
		double getMaxSpeed() const { return _maxSpeed; }


        ~Vehicle() = default;

    private:
        //state vectors
        std::array<double, 6> _front; 
		std::array<double, 6> _center;
		std::array<double, 6> _rear;
		//input parameters
        double _speed;
		double _steeringAngle;
        double _steeringRate;
        //geometric parameters
		const double _length; 
		const double _width;

		//constraints
        double _maxSteeringAngle = DEF_MAX_STEERING_ANGLE;
		double _maxSteeringRate = DEF_MAX_STEERING_RATE;
		double _maxSpeed = DEF_MAX_SPEED;


		void _updateSteeringAngle(double dt);
		void _updateCenter(double dt);
		void _updateFront(double dt);
		void _updateRear(double dt);
		void _setVelocity(double speed);
		double _validateOrientation(const double& orientation) const;


    };
}