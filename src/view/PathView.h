#pragma once
#include "ItemView.h"
namespace view
{
    class PathView :
        public ItemView
    {
	public:
		PathView() = default;
		PathView(std::vector<model::Point>& path, sf::Color color = sf::Color::Red);
		~PathView() = default;
		void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
		sf::Vector2f getPosition() const override;
	private:
		std::vector<sf::Vertex> _path;
    };
}
