#pragma once
#include <SFML/Graphics.hpp>
#include "view/ItemView.h"

namespace model { class Cone; }

namespace view {
    class ConeView :
        public ItemView
    {
    public:
        void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
        ConeView(const model::Cone& cone);
        ConeView();
        ~ConeView() = default;
        sf::Vector2f getPosition() const override;
    private:
        sf::Color _typeToColor(const model::Cone * cone) const;
        const model::Cone* _cone;
    };
}