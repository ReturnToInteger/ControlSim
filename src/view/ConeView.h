#pragma once
#include <SFML/Graphics.hpp>
#include "src/model/Cone.h"
#include "src/view/ItemView.h"

namespace view {
    class ConeView :
        public ItemView
    {
    public:
        void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
        ConeView(model::Cone* cone, sf::Color color = sf::Color::Green);
        ConeView();
        ~ConeView() = default;
        sf::Vector2f getPosition() const override;
    private:
        model::Cone* _cone;
    };
}