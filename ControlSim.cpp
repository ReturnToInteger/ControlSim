#include <SFML/Graphics.hpp>
#include "model/World.h"
#include "model/Car.h"


int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "SFML Test");

    sf::CircleShape shape(50.f);
    shape.setFillColor(sf::Color::Green);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear();
        window.draw(shape);
        window.display();
    }

    return 0;
}