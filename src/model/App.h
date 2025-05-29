#pragma once
#include <vector>
#include "model/items/ObstacleBase.h"
#include "Vehicle.h"
#include "mapReaders/IMapReader.h"
#include "view/AppView.h"
#include "model/items/Cone.h"
#include "model/utils/SimpleTimer.h"
#include "model/perception/Perception.h"
#include "model/pathPlanner/PathPlanner.h"
#include "model/mapReaders/ManualMapReader.h"
#include "model/utils/ModelUtils.h"
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
