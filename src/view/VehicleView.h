#pragma once  
#include <SFML/Graphics.hpp>  
#include "src/model/Vehicle.h"
#include "src/view/ItemView.h"

// VehicleView holds a non-owning pointer to Vehicle.
// Vehicle lifetime is managed by App via std::unique_ptr.
#ifndef M_PI
#define M_PI 3.14159265358979323846 // Define M_PI if not already defined
#endif

namespace view {
	class VehicleView :
		public ItemView
	{
	public:
		void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
		VehicleView();
		VehicleView(model::Vehicle& vehicle, sf::Color color = sf::Color::Red);
		~VehicleView() = default;
		sf::Vector2f getPosition() const override;
		double getOrientation() const;
	private:
		model::Vehicle* _vehicle;
	};
}