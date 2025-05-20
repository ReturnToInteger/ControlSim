#include "VehicleView.h"
#include <cassert>

void VehicleView::draw(sf::RenderTarget& target, sf::RenderStates states = sf::RenderStates::Default) const
{
	if (_vehicle == nullptr) {
		return;
	}
	auto position = _vehicle->getPosition();

	sf::RectangleShape vehicleShape(sf::Vector2f(_vehicle->getLength(),
		_vehicle->getWidth()));
	vehicleShape.setOrigin(_vehicle->getLength(), _vehicle->getWidth());
	vehicleShape.setFillColor(sf::Color::Red);
	vehicleShape.setPosition(position.X(),position.Y());
	vehicleShape.setRotation(_vehicle->getOrientation() / M_PI * 180);

	target.draw(vehicleShape);
}

VehicleView::VehicleView()
	: _vehicle(nullptr)
{
}

VehicleView::VehicleView(model::Vehicle* vehicle)
{
	_vehicle = vehicle;
}

sf::Vector2f VehicleView::getPosition() const
{
	if (_vehicle == nullptr) {
		return sf::Vector2f(0, 0);
	}
	auto position = _vehicle->getPosition();
	return sf::Vector2f(position.X(),position.Y());
}
