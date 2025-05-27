#pragma once
#include <vector>
#include "src/model/items/ObstacleBase.h"
#include "Vehicle.h"
#include "mapReaders/IMapReader.h"
#include "src/view/AppView.h"
#include "src/model/items/Cone.h"
#include "src/model/utils/SimpleTimer.h"
#include "src/model/perception/Perception.h"
#include "src/model/pathPlanner/PathPlanner.h"
#include "src/model/mapReaders/ManualMapReader.h"
#include "src/model/utils/ModelUtils.h"
#include <mutex>
#include <thread>


namespace model {
    class App
    {
    private:
		std::vector<Cone> _cones;
		std::unique_ptr<view::AppView> _view;
		std::unique_ptr<Vehicle> _vehicle;
		std::unique_ptr<model::Perception> _perception;
		std::mutex _simLock;

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
