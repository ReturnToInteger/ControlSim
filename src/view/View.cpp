#include "View.h" 
 
namespace view {



	void View::display(const sf::CircleShape& shape) {
		window.clear();
		window.draw(shape);
		window.display();
	}



}