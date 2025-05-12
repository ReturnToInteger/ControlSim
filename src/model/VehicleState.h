
#pragma once
#include <array>
#include <cmath>
#include "Point.h"
#include "utils/ModelUtils.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846 
#endif

#ifndef DEF_LENGTH  
#define DEF_LENGTH 1.5  
#endif // !DEF_LENGTH  
#ifndef DEF_WIDTH  
#define DEF_WIDTH 0.5  
#endif // !DEF_WIDTH  
#ifndef DEF_MAX_STEERING_ANGLE  
#define DEF_MAX_STEERING_ANGLE M_PI /6.0 
#endif // !DEF_MAX_STEERING_ANGLE  
#ifndef DEF_MAX_STEERING_RATE  
#define DEF_MAX_STEERING_RATE M_PI / 3.0
#endif // !DEF_MAX_STEERING_RATE  
#ifndef DEF_MAX_SPEED  
#define DEF_MAX_SPEED 15.0  
#endif // !DEF_MAX_SPEED  
#ifndef DEF_MAX_ACCELERATION
#define DEF_MAX_ACCELERATION 5.0
#endif // !DEF_MAX_ACCELERATION
#ifndef DEF_MAX_BRAKE
#define DEF_MAX_BRAKE 10.0
#endif // !DEF_MAX_BRAKE
namespace model
{
	class VehicleState
	{
	public:
		VehicleState();
        void steer();
        void drive();
        void brake();
        std::array<Point, 3> getAllPositions() const;
        std::array<double, 3> getAllOrientations() const;
        Point getPosition() const;
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
		void setTargetSpeed(double speed) { _targetSpeed = speed; }
		void setTargetSteeringAngle(double angle) { _targetSteeringAngle = angle; }
        void updateState(double dt);

    private:

        //state vectors  
        std::array<double, 6> _front;
        std::array<double, 6> _center;
        std::array<double, 6> _rear;
        //input parameters  
        double _speed;
        double _acceleration;
        double _steeringAngle;
        double _steeringRate;
        double _brakeAcceleration;
        //geometric parameters  
        const double _length;
        const double _width;

        //constraints  
        double _maxSteeringAngle = DEF_MAX_STEERING_ANGLE;
        double _maxSteeringRate = DEF_MAX_STEERING_RATE;
        double _maxSpeed = DEF_MAX_SPEED;
        double _maxAcceleration = DEF_MAX_ACCELERATION;
        double _maxBrake = DEF_MAX_BRAKE;
        //targets
        double _targetSpeed = 0.0;
        double _targetSteeringAngle = 0.0;

        void _updateSteering(double dt);
        void _updateDriving(double dt);
        void _updateCenter(double dt);
        void _updateFront(double dt);
        void _updateRear(double dt);
        double _validateOrientation(const double& orientation) const;

        void _setSteeringAngle(double angle);
        void _setSteeringRate(double rate);
        void _setSpeed(double speed);
        void _setAcceleration(double acceleration, double brake);


	};
}


