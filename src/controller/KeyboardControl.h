#pragma once  
#include "model/controllerLogic/IControllerLogic.h"  
#include <model/controllerLogic/ControlCommand.h>
#include <SFML/Window/Keyboard.hpp>



namespace model {
	class IVehicleState;
	class Path;
}
namespace controller  
{
	template <typename StateT>
	class KeyboardControl :  
		public model::IControllerLogic<StateT>  
	{ 
	public:
		// Inherited via IControllerLogic  
		model::ControlCommand drive(StateT const& state, model::Path const& path) override
		{

			return { detectSpeed(), detectAngle() };

		}
		~KeyboardControl() = default;
		static double detectSpeed()
		{
			double speed = 0.0;
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::W) && !sf::Keyboard::isKeyPressed(sf::Keyboard::S))
			{
				speed = 1.0;
			}
			else if (sf::Keyboard::isKeyPressed(sf::Keyboard::S) && !sf::Keyboard::isKeyPressed(sf::Keyboard::W))
			{
				speed = -1.0;
			}

			return speed;
		}

		static double detectAngle() {

			double angle = 0.0;
			if (sf::Keyboard::isKeyPressed(sf::Keyboard::A) && !sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
				angle = -1.0;
			}
			else if (sf::Keyboard::isKeyPressed(sf::Keyboard::D) && !sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
				angle = 1.0;
			}

			return angle;
		}




	};
}
