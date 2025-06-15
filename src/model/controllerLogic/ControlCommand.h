#pragma once
namespace model {
	struct ControlCommand
	{
		double normSpeed;
		double normSteering;

		ControlCommand(double s, double a) : normSpeed(s), normSteering(a) {}
	};
}
