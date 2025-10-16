#include "MapView.h"
#include "MapView.h"

void view::MapView::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
}

view::MapView::MapView(model::Map const& map) : m_map(&map)
{
}

sf::Vector2f view::MapView::getPosition() const
{
	return sf::Vector2f();
}
