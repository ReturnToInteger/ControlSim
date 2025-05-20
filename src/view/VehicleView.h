#pragma once  
#include <SFML/Graphics.hpp>  
#include "src/model/Vehicle.h"  
#include "src/model/Point.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846 // Define M_PI if not already defined
#endif

class VehicleView :  
   public sf::Drawable  
{  
public:  
   void draw(sf::RenderTarget& target, sf::RenderStates states) const override; 
   VehicleView();
   VehicleView(model::Vehicle* vehicle);
   ~VehicleView() = default;
   sf::Vector2f getPosition() const;
private:  
   model::Vehicle* _vehicle;
};
