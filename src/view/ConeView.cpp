#include "ConeView.h"
#include <cassert>

void view::ConeView::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
	if (_cone == nullptr) {
		return;
	}
	auto position = _cone->getPosition();
	sf::CircleShape coneShape(_cone->getRadius());
	coneShape.setOrigin(_cone->getRadius(), _cone->getRadius());
	coneShape.setFillColor(_color);
	coneShape.setPosition(position.X(), position.Y());
	target.draw(coneShape);
}

view::ConeView::ConeView(model::Cone* cone, sf::Color color) : ItemView(color), _cone(cone)
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
