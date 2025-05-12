#pragma once
#include "KeyboardControl.h"
#include <SFML/Window/Keyboard.hpp>



void controller::KeyboardControl::drive(model::VehicleState& state)
{

	state.setTargetSpeed(detectSpeed());
	state.setTargetSteeringAngle(detectAngle());

}

double controller::KeyboardControl::detectSpeed()
{
	double speed;
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::W) && !sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
		speed = 1.0;
	}
	else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S) && !sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
		speed = -1.0;
	}
	else {
		speed = 0.0;
	}
	return speed;
}

double controller::KeyboardControl::detectAngle() {

	double angle;
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::A) && !sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
		angle = -1.0;
	}
	else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D) && !sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
		angle = 1.0;
	}
	else {
		angle = 0.0;
	}
	return angle;
}