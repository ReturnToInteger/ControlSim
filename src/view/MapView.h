#pragma once
#include <SFML/Graphics.hpp>
#include "view/ItemView.h"

namespace model { class Map; }

namespace view {
    class MapView :
        public ItemView
    {
    public:
        void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
        MapView(model::Map const& map);
        MapView() = default;
        ~MapView() = default;
        [[nodiscard]] sf::Vector2f getPosition() const override;
    private:
        const model::Map* m_map;
    };
}