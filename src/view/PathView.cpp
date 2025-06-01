#include "PathView.h"
#include "model/items/Path.h"

view::PathView::PathView(const model::Path& path, sf::Color color)
    : ItemView(color), _path(&path)
{

}


void view::PathView::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    sf::VertexArray path;
    path.setPrimitiveType(sf::LinesStrip);
    for (const auto& point : *_path) {
        path.append(sf::Vertex(sf::Vector2f(point.x, point.y),_color));
    }
    
    target.draw(path);
}

sf::Vector2f view::PathView::getPosition() const
{
    auto position = _path->getPosition();
   return sf::Vector2f(position.X(),position.Y()); // Access the position of the first vertex
}
