#pragma once
#include <SFML/Graphics.hpp>
#include "src/model/Cone.h"

class ConeView :
    public sf::Drawable
{
public:
    void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
    ConeView(model::Cone* cone);
	ConeView();
    ~ConeView() = default;
    sf::Vector2f getPosition() const;
private:
    model::Cone* _cone;
};

