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
        ConeView(model::Cone const& cone);
        ConeView();
        ~ConeView() = default;
        [[nodiscard]] sf::Vector2f getPosition() const override;
        static constexpr float defRadius = 0.15;
    private:
        sf::Color _typeToColor(const model::Cone * cone) const;
        const model::Cone* _cone;
        mutable sf::CircleShape _shape;
    };
}