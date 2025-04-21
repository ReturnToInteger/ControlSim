#include "Vehicle.h"



namespace model {



	Vehicle::Vehicle() : _length(DEF_LENGTH),
		_width(DEF_WIDTH),
		_front{ DEF_LENGTH/2,0,0,0,0,0 },
		_center{ 0,0,0,0,0,0 },
		_rear{ -DEF_LENGTH/2,0,0,0,0,0 },
		_speed(0),
		_steeringAngle(0),
		_steeringRate(0)
	{

	}
	Vehicle::Vehicle(Point startPos, double orientation, double speed) : Vehicle(startPos.X(), startPos.Y(), orientation, speed) {}

	Vehicle::Vehicle(double startX, double startY, double orientation, double speed) : _length(DEF_LENGTH),
		_width(DEF_WIDTH),
		_steeringAngle(0),
		_steeringRate(0)
	{
		accelerateTo(speed);
		orientation = _validateOrientation(orientation);
		_front[0] = startX + _length / 2 * cos(orientation);
		_front[1] = startY + _length / 2 * sin(orientation);
		_front[2] = orientation;
		_front[3] = 0;
		_front[4] = 0;
		_front[5] = 0;
		_center[0] = startX;
		_center[1] = startY;
		_center[2] = orientation;
		_center[3] = speed * cos(startX);
		_center[4] = speed * sin(startY);
		_center[5] = 0;
		_rear[0] = startX - _length / 2 * cos(orientation);
		_rear[1] = startY - _length / 2 * sin(orientation);
		_rear[2] = orientation;
		_rear[3] = 0;
		_rear[4] = 0;
		_rear[5] = 0;
	}

	// Update method
   void Vehicle::updateState(double dt) {

	   _updateSteeringAngle(dt);
	   _updateCenter(dt);
	   _updateFront(dt);
	   _updateRear(dt);
   }
   void Vehicle::steer(double rate)
   {
	   //if (isSteering){
		if (abs(rate) < _maxSteeringRate) {
			_steeringRate = rate;
		}
		else
		{
			_steeringRate = (rate >= 0 ? 1 : -1) * _maxSteeringRate;
		}
	   //}
	   //else
	   //{
		  // if (_steeringAngle > 0) {
			 //  _steeringAngle -= _maxSteeringRate*0.1;
		  // }
		  // else if (_steeringAngle < 0) {
			 //  _steeringAngle += _maxSteeringRate*0.1;
		  // }
	   //}
   }

   void Vehicle::accelerateTo(double speed)
   {
	   if (abs(speed) < _maxSpeed) {
		   _speed = speed;
	   }
	   else
	   {
		   _speed = (speed >= 0 ? 1 : -1) * _maxSpeed;
	   }
   }

   void Vehicle::sendInput(double speed, double steeringRate)
   {
	   accelerateTo(speed);
	   steer(steeringRate);
   }


   std::array<Point, 3> Vehicle::getAllPositions() const
   {
	   return { Point(_center[0],_center[1]), Point(_front[0],_front[1]),Point(_rear[0],_rear[1]) };
   }

   std::array<double, 3> Vehicle::getAllOrientations() const
   {
	   return { _center[2],_front[2],_rear[2] };
   }

   Point Vehicle::getPosition() const
   {
	   return Point(_center[0],_center[1]);
   }

   double Vehicle::getOrientation() const
   {
	   return _center[2];
   }
   
   void Vehicle::_updateSteeringAngle(double dt)
   {
	   double temp = _steeringAngle + _steeringRate * dt;
	   if (abs(temp) < _maxSteeringAngle) {
		   _steeringAngle = temp;
	   }
	   else
	   {
		   _steeringAngle = (temp >= 0 ? 1 : -1) * _maxSteeringAngle;
	   }
   }

   void Vehicle::_updateCenter(double dt)
   {
	   double beta = atan(tan(_steeringAngle) / 2);
	   _center[5] = _speed * tan(_steeringAngle) * cos(beta) / _length;
	   _center[2] = _validateOrientation(_center[2] + _center[5] * dt);
	   _center[3] = _speed * cos(_center[2] + beta);
	   _center[4] = _speed * sin(_center[2] + beta);
	   _center[0] += dt * _center[3];
	   _center[1] += dt * _center[4];
   }

   void Vehicle::_updateFront(double dt)
   {
	   _front[0] = _center[0] + _length / 2 * cos(_center[2]);
	   _front[1] = _center[1] + _length / 2 * sin(_center[2]);
	   _front[2] = _center[2] + _steeringAngle;
	   _front[3] = 0;
	   _front[4] = 0;
	   _front[5] = 0;
   }

   void Vehicle::_updateRear(double dt)
   {
	   _rear[0] = _center[0] - _length / 2 * cos(_center[2]);
	   _rear[1] = _center[1] - _length / 2 * sin(_center[2]);
	   _rear[2] = _center[2];
	   _rear[3] = 0;
	   _rear[4] = 0;
	   _rear[5] = 0;
   }


   double Vehicle::_validateOrientation(const double& orientation) const
   {
	   double a = std::fmod(orientation + M_PI, 2 * M_PI);
	   if (a < 0) a += 2 * M_PI;
	   return a - M_PI;
   }

}