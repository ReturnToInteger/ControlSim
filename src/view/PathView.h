#pragma once
#include "ItemView.h"
#include <model/utils/ModelUtils.h>
//#include <model/items/Path.h>
namespace model { class Path; }
namespace view
{
    class PathView :
        public ItemView
    {
	public:
		PathView() :m_path(nullptr){};
		PathView(model::Path const& path, sf::Color color = {150,150,150});
		~PathView() = default;
		void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
		sf::Vector2f getPosition() const override;
		void setColor(sf::Color color) { m_color = color; }
	private:
		const model::Path* m_path;

    };
}
