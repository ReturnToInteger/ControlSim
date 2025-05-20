#include "ConeView.h"
#include <cassert>

void ConeView::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
	if (_cone == nullptr) {
		return;
	}
	auto position = _cone->getPosition();
	sf::CircleShape coneShape(_cone->getRadius());
	coneShape.setOrigin(_cone->getRadius(), _cone->getRadius());
	coneShape.setFillColor(sf::Color::Green);
	coneShape.setPosition(position.X(), position.Y());
	target.draw(coneShape);
}

ConeView::ConeView(model::Cone* cone) : _cone(cone)
{
}

ConeView::ConeView() : _cone(nullptr)
{
}

sf::Vector2f ConeView::getPosition() const
{
	if (_cone == nullptr) {
		return sf::Vector2f(0, 0);
	}
	auto position = _cone->getPosition();
	return sf::Vector2f(position.X(),position.Y());
}
