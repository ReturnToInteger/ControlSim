#pragma once
#include <vector>
#include <array>
#include <mutex>
#include <functional>
#include "model/items/obstacles/Cone.h"
#include "model/items/obstacles/Obstacle.h"
#include "model/mapReaders/Map.h"
#include "controller/events/InputEventHandler.h"
#include "controller/events/InputEventPublisher.h"
#include <unordered_set>
//#include "model/perception/Perception.h"

template<typename ... Ts>                                                 
struct Overload : Ts ... {
	using Ts::operator() ...;

};
template<class... Ts> Overload(Ts...) -> Overload<Ts...>;

namespace model {
	class IVehicle;
	class Perception;
	class IMapReader;
	class VehicleState;
	class IVehicleState;
}
namespace view {
	class AppView;
}

namespace controller {
	using controller::events::InputEvent;
	using namespace model;
	struct InputTranslate {
		void operator()(events::PressedEsc const&) { std::cout << "Pressed ESC.\n"; }
		void operator()(events::PressedLShift const&) {std::cout << "Pressed LShift.\n"; }
		void operator()(events::Scrolled const& i) {std::cout << "zoom: " << i.delta << "\n"; }
		void operator()(events::ClickedAt const& i) {std::cout << "Clicked at :" << i.x << "; " << i.y << "\n"; }
		void operator()(events::Resized const&) {std::cout << "Resized.\n"; }
		void operator()(events::LostFocus const&) {std::cout << "Lost Focus.\n"; }
		void operator()(events::GainedFocus const&) { std::cout << "Gained Focus.\n"; }
		void operator()(events::RightClickDown const& c) { std::cout << "Right Click Down: " <<c.lastX<<"; "<<c.currentX << "\n"; }
		void operator()(events::None const&) {}

	};
    class App : public events::InputEventHandler
    {
	public:
		struct SharedGoals {
			std::array<model::Point, 2> goals;
			std::atomic_bool goalHasChanged;
		};
		App(std::unique_ptr<model::IVehicle> vehicle,
			std::unique_ptr<model::IMapReader> mapReader,
			std::unique_ptr<view::AppView> view, 
			int threadCount = 1);

		void run();

		~App();

		// Inherited via InputEventHandler
		void handleInputEvent(std::string const& src, events::InputEvent const& e) override;

    private:		
		void pathPlanningWorker(std::unordered_set<const model::Obstacle*>& detectedCones, int const index);
		void startPlanningThreads(int threadCount, std::vector<std::thread>& threads, std::function<void(int)> const& loopLambda);
		// Should be moved to model
		bool calcGoal(std::unordered_set<const model::Obstacle*> const& cones, model::IVehicleState const& state, model::Point& currentGoal,double maxDist);

		model::Map m_map;
		std::unique_ptr<view::AppView> m_view;
		std::unique_ptr<model::IVehicle> m_vehicle;
		std::unique_ptr<model::Perception> m_perception;

		double m_frameTime;
		// Threading
		std::mutex m_simLock;
		std::mutex m_pathLock;
		std::condition_variable m_resetEvent;
		const int m_threadCount;
		std::atomic_bool m_running;
		SharedGoals m_sharedGoals;


		// Need a reader, which will read the map 
		std::unique_ptr<model::IMapReader> m_mapReader;

	};
}
