#pragma once
#include <vector>
#include <mutex>
#include "model/items/Cone.h"
//#include "model/perception/Perception.h"


namespace model {
	class Vehicle;
	class Perception;
	class IMapReader;
}
namespace view {
	class AppView;
}

namespace controller {
    class App
    {
    private:
		std::vector<model::Cone> _cones;
		std::unique_ptr<view::AppView> _view;
		std::unique_ptr<model::Vehicle> _vehicle;
		std::unique_ptr<model::Perception> _perception;
		std::mutex _simLock;

		// Need a reader, which will read the map 
		std::unique_ptr<model::IMapReader> _mapReader;
	public:
		App(std::unique_ptr<model::Vehicle> vehicle,
			std::unique_ptr<model::IMapReader> mapReader,
			std::unique_ptr<view::AppView> view);

		// Need a run method, which will run the game
		void run();
		~App();


    };
}
