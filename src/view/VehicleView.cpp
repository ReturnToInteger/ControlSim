#include "VehicleView.h"
#include <cassert>
#include "model/Vehicle.h"


void view::VehicleView::draw(sf::RenderTarget& target, sf::RenderStates states = sf::RenderStates::Default) const
{
	if (_vehicle == nullptr) {
		return;
	}
	auto position = _vehicle->getPosition();

	sf::RectangleShape vehicleShape(sf::Vector2f(_vehicle->getLength(),
		_vehicle->getWidth()));

	sf::Sprite sprite(_texture);
	//vehicleSprite.setTextureRect(sf::IntRect(
	//	0, 0,
	//	static_cast<int>(_vehicle->getWidth()) * 10,
	//	static_cast<int>(_vehicle->getLength()) * 10
	//));
	sf::Vector2u size=_texture.getSize();
	sprite.setOrigin(size.x/2.0, size.y/2.0);
	sprite.setRotation(_vehicle->getOrientation() / M_PI * 180+90);
	sprite.setScale(_vehicle->getWidth() / size.x, _vehicle->getLength() / size.y);
	//vehicleSprite.setFillColor(_color);
	sprite.setPosition(position.X(),position.Y());
	target.draw(sprite);
}

view::VehicleView::VehicleView()
	: ItemView(), _vehicle(nullptr)
{
}

view::VehicleView::VehicleView(const model::Vehicle& vehicle, sf::Color color)
	: ItemView(color), _vehicle(&vehicle)
{
	if (!_texture.loadFromFile("car.png"))
		throw std::ios_base::failure("Image not found");
;
	isDetected = true;
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
