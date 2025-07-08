#include "PathView.h"
#include "model/items/Path.h"

view::PathView::PathView(model::Path const& path, sf::Color color)
    : ItemView(color), _path(&path)
{

}


void view::PathView::draw(sf::RenderTarget& target, sf::RenderStates states) const
{
    if (_path == nullptr)
    {
        return;
    }
    sf::VertexArray path;
    path.setPrimitiveType(sf::LinesStrip);
    for (auto const& state: *_path) {
        model::Pose point = state->getPose();
        path.append(sf::Vertex(sf::Vector2f(static_cast<float>(point.x), static_cast<float>(point.y)), _color));
    }
    
    target.draw(path);
}

sf::Vector2f view::PathView::getPosition() const
{
    auto position = _path->getPosition();
    return { static_cast<float>(position.X()), static_cast<float>(position.Y()) }; // Access the position of the first vertex
}
