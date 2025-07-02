#include "VehicleView.h"
#include <cassert>
#include "model/Vehicle.h"


void view::VehicleView::draw(sf::RenderTarget& target, sf::RenderStates states = sf::RenderStates::Default) const
{
	if (_vehicle == nullptr) {
		return;
	}
	auto position = _vehicle->getPosition();

	sf::RectangleShape vehicleShape(sf::Vector2f(static_cast<float>(_vehicle->getLength()),
		static_cast<float>(_vehicle->getWidth())));

	sf::Sprite sprite(_texture);
	//vehicleSprite.setTextureRect(sf::IntRect(
	//	0, 0,
	//	static_cast<int>(_vehicle->getWidth()) * 10,
	//	static_cast<int>(_vehicle->getLength()) * 10
	//));
	sf::Vector2u size=_texture.getSize();
	sprite.setOrigin(static_cast<float>(size.x/2.0), static_cast<float>(size.y/2.0));
	sprite.setRotation(static_cast<float>(degree(getOrientation())+90));
	sprite.setScale(static_cast<float>(_vehicle->getWidth() / size.x), static_cast<float>(_vehicle->getLength() / size.y));
	//vehicleSprite.setFillColor(_color);
	sprite.setPosition(static_cast<float>(position.X()), static_cast<float>(position.Y()));
	target.draw(sprite);
}

view::VehicleView::VehicleView()
	: _vehicle(nullptr)
{
}

view::VehicleView::VehicleView(model::Vehicle const& vehicle, sf::Color color)
	: ItemView(color), _vehicle(&vehicle)
{
	if (!_texture.loadFromFile("car.png"))
	{
		throw std::ios_base::failure("Image not found");
	}
;
	isDetected = true;
}

sf::Vector2f view::VehicleView::getPosition() const
{
	if (_vehicle == nullptr) {
		return { 0, 0 };
	}
	auto position = _vehicle->getPosition();
	return { static_cast<float>(position.X()), static_cast<float>(position.Y()) };
}

model::Angle view::VehicleView::getOrientation() const
{
	if (_vehicle == nullptr) {
		throw std::runtime_error("Vehicle doesn't exist");
	}
	return _vehicle->getOrientation();
}
