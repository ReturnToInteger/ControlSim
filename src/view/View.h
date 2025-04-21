#pragma once 

#include <SFML/Graphics.hpp>

 
namespace view { 
class View { 
public: 
    View(); 
    ~View(); 
    template <typename... Args>
    View(Args&&... args)
        : window(std::forward<Args>(args)...)  // Forward arguments to RenderWindow constructor
    {
        // Additional setup for View (if necessary)
    }
    void display(const sf::CircleShape& shape);
    View(const View&) = delete;

private:
    sf::RenderWindow window;
};
} // namespace view 
