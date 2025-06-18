#include "ConeView.h"
#include "model/items/Cone.h"
//#include <cassert>
#include <stdexcept>


void view::ConeView::draw(sf::RenderTarget& target, sf::RenderStates states) const  
{  
	if (_cone == nullptr) {  
		return;  
	}  
	auto position = _cone->getPosition();  
	_shape.setOrigin(static_cast<float>(_cone->getRadius()), static_cast<float>(_cone->getRadius()));  
	_shape.setFillColor(_typeToColor(_cone));  
	_shape.setPosition(static_cast<float>(position.X()), static_cast<float>(position.Y()));  
	target.draw(_shape);  
}

view::ConeView::ConeView(model::Cone const& cone) : _cone(&cone), ItemView(_typeToColor(&cone)), _shape(_cone->getRadius())
{
}

view::ConeView::ConeView() : ItemView(), _cone(nullptr), _shape((float)0.15)
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

sf::Color view::ConeView::_typeToColor(const model::Cone* cone) const
{
	if (cone == nullptr) {
		return sf::Color::Black;
	}
	if (!isDetected) {
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
