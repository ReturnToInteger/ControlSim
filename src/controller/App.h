#pragma once
#include <vector>
#include <mutex>
#include "model/items/Cone.h"
#include "model/eventHandlers/InputEventHandler.h"
#include "model/eventHandlers/InputEventPublisher.h"
//#include "model/perception/Perception.h"


namespace model {
	class Vehicle;
	class Perception;
	class IMapReader;
	class VehicleState;
}
namespace view {
	class AppView;
}

namespace controller {
	using model::InputEvent;
	struct InputTranslate {
		void operator()(model::PressedEsc const&) { std::cout << "Pressed ESC." << std::endl; }
		void operator()(model::PressedLShift const&) {std::cout << "Pressed LShift." << std::endl; }
		void operator()(model::Scrolled const& i) {std::cout << "zoom: " << i.delta << std::endl; }
		void operator()(model::ClickedAt const& i) {std::cout << "Clicked at :" << i.x << "; " << i.y << std::endl; }
		void operator()(model::Resized const&) {std::cout << "Resizing..." << std::endl; }
		void operator()(model::LostFocus const&) {std::cout << "Lost Focus." << std::endl; }
		void operator()(model::GainedFocus const&) { std::cout << "Gained Focus." << std::endl; }
		void operator()(model::None const&) {}
	};
    class App : public model::InputEventHandler
    {
    private:
		std::vector<model::Cone> _cones;
		std::unique_ptr<view::AppView> _view;
		std::unique_ptr<model::Vehicle> _vehicle;
		std::unique_ptr<model::Perception> _perception;
		std::mutex _simLock;

		// Need a reader, which will read the map 
		std::unique_ptr<model::IMapReader> _mapReader;
		// Should be moved inside path planner
		void _calcGoal(std::vector<const model::Cone*> const& cones, model::VehicleState const& state);




	public:
		App(std::unique_ptr<model::Vehicle> vehicle,
			std::unique_ptr<model::IMapReader> mapReader,
			std::unique_ptr<view::AppView> view);

		// Need a run method, which will run the game
		void run();

		~App();



		// Inherited via InputEventHandler
		void handleInputEvent(std::string const& src, model::InputEvent const& e) override;

	};
}
