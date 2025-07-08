#include "KeyboardControl.h"
#include <SFML/Window/Keyboard.hpp>
#include <model/pathPlanning/PathPlanner.h>



namespace controller {
	model::ControlCommand controller::KeyboardControl::drive(model::VehicleState const& state, model::Path const& path)
	{

		return { detectSpeed(), detectAngle() };

	}

	double KeyboardControl::detectSpeed()
	{
		double speed = 0.0;
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::W) && !sf::Keyboard::isKeyPressed(sf::Keyboard::S)) 
		{
			speed = 1.0;
		}
		else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S) && !sf::Keyboard::isKeyPressed(sf::Keyboard::W)) 
		{
			speed = -1.0;
		}

		return speed;
	}

	double KeyboardControl::detectAngle() {

		double angle = 0.0;
		if (sf::Keyboard::isKeyPressed(sf::Keyboard::A) && !sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
			angle = -1.0;
		}
		else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D) && !sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
			angle = 1.0;
		}

		return angle;
	}
	model::ControlCommand KeyboardControl::drive(model::diffDrive::State const& state, model::Path const& path)
	{
		return { detectSpeed(), detectAngle() };
	}
}