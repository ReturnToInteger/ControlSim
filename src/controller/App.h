#pragma once
#include <vector>
#include <mutex>
#include "model/items/Cone.h"
#include "model/events/InputEventHandler.h"
#include "model/events/InputEventPublisher.h"
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
	using model::events::InputEvent;
	using namespace model;
	struct InputTranslate {
		void operator()(events::PressedEsc const&) { std::cout << "Pressed ESC." << std::endl; }
		void operator()(events::PressedLShift const&) {std::cout << "Pressed LShift." << std::endl; }
		void operator()(events::Scrolled const& i) {std::cout << "zoom: " << i.delta << std::endl; }
		void operator()(events::ClickedAt const& i) {std::cout << "Clicked at :" << i.x << "; " << i.y << std::endl; }
		void operator()(events::Resized const&) {std::cout << "Resizing..." << std::endl; }
		void operator()(events::LostFocus const&) {std::cout << "Lost Focus." << std::endl; }
		void operator()(events::GainedFocus const&) { std::cout << "Gained Focus." << std::endl; }
		void operator()(events::None const&) {}
	};
    class App : public events::InputEventHandler
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
		void handleInputEvent(std::string const& src, events::InputEvent const& e) override;

	};
}
