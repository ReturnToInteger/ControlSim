#include "VehicleView.h"
#include <cassert>

void view::VehicleView::draw(sf::RenderTarget& target, sf::RenderStates states = sf::RenderStates::Default) const
{
	if (_vehicle == nullptr) {
		return;
	}
	auto position = _vehicle->getPosition();

	sf::RectangleShape vehicleShape(sf::Vector2f(_vehicle->getLength(),
		_vehicle->getWidth()));
	vehicleShape.setOrigin(_vehicle->getLength(), _vehicle->getWidth());
	vehicleShape.setFillColor(_color);
	vehicleShape.setPosition(position.X(),position.Y());
	vehicleShape.setRotation(_vehicle->getOrientation() / M_PI * 180);

	target.draw(vehicleShape);
}

view::VehicleView::VehicleView()
	: ItemView(), _vehicle(nullptr)
{
}

view::VehicleView::VehicleView(model::Vehicle* vehicle, sf::Color color)
	: ItemView(color),_vehicle(vehicle)
{
}

sf::Vector2f view::VehicleView::getPosition() const
{
	if (_vehicle == nullptr) {
		return sf::Vector2f(0, 0);
	}
	auto position = _vehicle->getPosition();
	return sf::Vector2f(position.X(),position.Y());
}

double view::VehicleView::getOrientation() const
{
	if (_vehicle == nullptr) {
		return 0;
	}
	return _vehicle->getOrientation();
}
