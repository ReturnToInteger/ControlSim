#pragma once
#include <SFML/Graphics.hpp>
//#include "model/items/Item.h"

namespace view {
    class ItemView : public sf::Drawable {
    public:
        ~ItemView() override = default;
		ItemView(sf::Color color = sf::Color::Black) : _color(color) {}
        [[nodiscard]] virtual sf::Vector2f getPosition() const = 0;
        bool isDetected=false;
        //virtual const model::Item* getItem() const = 0;

    protected:
        // Pointer to the model object, if needed
        //model::Item* _item;
		sf::Color _color;
    };
}