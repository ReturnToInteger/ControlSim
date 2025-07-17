#include "VehicleView.h"
#include <cassert>
#include "model/utils/IDrawableVehicle.h"


void view::VehicleView::draw(sf::RenderTarget& target, sf::RenderStates states = sf::RenderStates::Default) const
{
	if (m_vehicle == nullptr) {
		return;
	}
	auto position = m_vehicle->getPosition();

	sf::RectangleShape vehicleShape(sf::Vector2f(static_cast<float>(m_vehicle->getLength()),
		static_cast<float>(m_vehicle->getWidth())));

	sf::Sprite sprite(m_texture);
	//vehicleSprite.setTextureRect(sf::IntRect(
	//	0, 0,
	//	static_cast<int>(m_vehicle->getWidth()) * 10,
	//	static_cast<int>(m_vehicle->getLength()) * 10
	//));
	sf::Vector2u size=m_texture.getSize();
	sprite.setOrigin(static_cast<float>(size.x/2.0), static_cast<float>(size.y/2.0));
	sprite.setRotation(static_cast<float>(degree(getOrientation())+90));
	sprite.setScale(static_cast<float>(m_vehicle->getWidth() / size.x), static_cast<float>(m_vehicle->getLength() / size.y));
	//vehicleSprite.setFillColor(m_color);
	sprite.setPosition(static_cast<float>(position.X()), static_cast<float>(position.Y()));
	target.draw(sprite);
}

view::VehicleView::VehicleView()
	: m_vehicle(nullptr)
{
}

view::VehicleView::VehicleView(model::IDrawableVehicle const& vehicle, sf::Color color)
	: ItemView(color), m_vehicle(&vehicle)
{
	if (!m_texture.loadFromFile("car.png"))
	{
		throw std::ios_base::failure("Image not found");
	}
;
	isDetected = true;
}

sf::Vector2f view::VehicleView::getPosition() const
{
	if (m_vehicle == nullptr) {
		return { 0, 0 };
	}
	auto position = m_vehicle->getPosition();
	return { static_cast<float>(position.X()), static_cast<float>(position.Y()) };
}

model::Angle view::VehicleView::getOrientation() const
{
	if (m_vehicle == nullptr) {
		throw std::runtime_error("Vehicle doesn't exist");
	}
	return m_vehicle->getOrientation();
}
