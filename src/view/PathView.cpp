#include "PathView.h"

view::PathView::PathView(std::vector<model::Point>& path, sf::Color color) 
   : ItemView(color) // Store the color for later use
{
   _path.reserve(path.size());
   for (auto& point : path) {
       _path.emplace_back(sf::Vertex(sf::Vector2f(point.X(), point.Y()), _color));
   }
}

void view::PathView::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
   target.draw(_path.data(), _path.size(), sf::PrimitiveType::LineStrip);
}

sf::Vector2f view::PathView::getPosition() const
{
   return _path[0].position; // Access the position of the first vertex
}
