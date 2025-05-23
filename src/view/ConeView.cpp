#include "ConeView.h"

void view::ConeView::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
	if (_cone == nullptr) {
		return;
	}
	auto position = _cone->getPosition();
	sf::CircleShape coneShape(_cone->getRadius());
	coneShape.setOrigin(_cone->getRadius(), _cone->getRadius());
	coneShape.setFillColor(_typeToColor(_cone));
	coneShape.setPosition(position.X(), position.Y());
	target.draw(coneShape);
}

view::ConeView::ConeView(model::Cone& cone) : _cone(&cone), ItemView(_typeToColor(&cone))
{
}

view::ConeView::ConeView() : ItemView(), _cone(nullptr)
{
}

sf::Vector2f view::ConeView::getPosition() const
{
	if (_cone == nullptr) {
		return sf::Vector2f(0, 0);
	}
	auto position = _cone->getPosition();
	return sf::Vector2f(position.X(),position.Y());
}

sf::Color view::ConeView::_typeToColor(model::Cone* cone) const
{
	if (cone == nullptr) {
		return sf::Color::Black;
	}
	if (!cone->isDetected()) {
		return sf::Color::White;
	}
	switch (cone->getType())
	{
	case model::ConeType::LEFT:
		return sf::Color::Yellow;
	case model::ConeType::RIGHT:
		return sf::Color::Blue;
	case model::ConeType::UNKNOWN:
		return sf::Color::Green;
	default:
		throw std::invalid_argument("Invalid ConeType in typeToColor");
	}
}
