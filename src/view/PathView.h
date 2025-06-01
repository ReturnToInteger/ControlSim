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
		PathView() = default;
		PathView(const model::Path& path, sf::Color color = sf::Color::Red);
		~PathView() = default;
		void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
		sf::Vector2f getPosition() const override;
	private:
		const model::Path* _path;

    };
}
