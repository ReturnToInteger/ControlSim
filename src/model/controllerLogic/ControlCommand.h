#pragma once
namespace model {
	struct ControlCommand
	{
		double speed;
		double steeringAngle;

		ControlCommand(double s, double a) : speed(s), steeringAngle(a) {}
	};
}
