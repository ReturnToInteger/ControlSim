#pragma once
#include <vector>
#include "ObstacleBase.h"
#include "Vehicle.h"
#include "mapReaders/IMapReader.h"
#include "src/view/AppView.h"
#include "Cone.h"
#include "src/model/utils/Timer.h"


namespace model {
    class App
    {
    private:
		std::vector<Cone> _cones;
		std::unique_ptr<view::AppView> _view;
		std::unique_ptr<Vehicle> _vehicle;
		std::unique_ptr<model::Perception> _perception;

		// Need a reader, which will read the map 
		std::unique_ptr<model::IMapReader> _mapReader;
	public:
		App(std::unique_ptr<Vehicle> vehicle,
			std::unique_ptr<model::IMapReader> mapReader,
			std::unique_ptr<view::AppView> view);

		// Need a run method, which will run the game
		void run();

    };
}
