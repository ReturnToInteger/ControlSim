#include "ConeView.h"
#include "model/items/obstacles/Cone.h"
//#include <cassert>
#include <stdexcept>


void view::ConeView::draw(sf::RenderTarget& target, sf::RenderStates states) const  
{  
	if (m_cone == nullptr) {  
		return;  
	}  
	auto position = m_cone->getPosition();  
	m_shape.setOrigin(static_cast<float>(m_cone->getRadius()), static_cast<float>(m_cone->getRadius()));  
	m_shape.setFillColor(typeToColor(m_cone));  
	m_shape.setPosition(static_cast<float>(position.X()), static_cast<float>(position.Y()));  
	target.draw(m_shape);  
}

view::ConeView::ConeView(model::Cone const& cone) : 
	ItemView(typeToColor(&cone)), 
	m_cone(&cone), 
	m_shape(static_cast<float>(m_cone->getRadius()))
{
}

view::ConeView::ConeView() : m_cone(nullptr), m_shape(defRadius)
{
}

sf::Vector2f view::ConeView::getPosition() const
{
	if (m_cone == nullptr) {
		return { 0, 0 };
	}
	auto position = m_cone->getPosition();
	return { static_cast<float>(position.X()),static_cast<float>(position.Y()) };
}

sf::Color view::ConeView::typeToColor(const model::Cone* cone) const
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
