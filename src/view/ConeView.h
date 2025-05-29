#pragma once
#include <SFML/Graphics.hpp>
#include "model/items/Cone.h"
#include "view/ItemView.h"
#include <cassert>
#include <stdexcept>


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