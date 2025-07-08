#pragma once  
#include <SFML/Graphics.hpp>  
#include "view/ItemView.h"
#include <model/utils/Angle.h>
#include "model/utils/IDrawableVehicle.h"

// VehicleView holds a non-owning pointer to Vehicle.
// Vehicle lifetime is managed by App via std::unique_ptr.

namespace model { 
	class IDrawableVehicle;
}
namespace view {
	
	class VehicleView :
		public ItemView
	{
	public:
		void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
		VehicleView();
		VehicleView(model::IDrawableVehicle const& vehicle, sf::Color color = sf::Color::Red);
		~VehicleView() = default;
		sf::Vector2f getPosition() const override;
		model::Angle getOrientation() const;
	private:
		const model::IDrawableVehicle* _vehicle;
		sf::Texture _texture;
	};
}